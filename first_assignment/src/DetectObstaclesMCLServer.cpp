/* 
	Group 23: Nicola Lorenzon 2087643, Daniele Moschetta 2087640
	
	This is the implementation of the MCL server for the obstacles detection.
*/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <first_assignment/DetectObstaclesAction.h>
#include <first_assignment/MotionControlLawAction.h>
#include "ObstaclesFinder.h"
#include "PointUtils.h"

/**
 * The function activeCb() logs a message indicating that the action server move_base has started and a
 * goal has been sent to it.
 */
void activeCb() {
    ROS_INFO("Action server move_base started, goal sent to it");
}

/**
 * The function doneCb is a callback function that prints the state of the move_base action server when
 * it finishes.
 * 
 * @param state The parameter "state" is of type "actionlib::SimpleClientGoalState" and represents the
 * state of the action server. It provides information about the current state of the action, such as
 * whether it is active, succeeded, aborted, or preempted.
 * @param result The result parameter is a pointer to the result of the move_base action. It contains
 * information about the outcome of the action, such as whether it was successful or not.
 */
void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO("Action server move_base finished in state [%s]", state.toString().c_str());
}

/**
 * The function activeCbMCL() logs a message indicating that the action server motion_law has started
 * and a goal has been sent to it.
 */
void activeCbMCL() {
    ROS_INFO("Action server motion_law started, goal sent to it");
}

/**
 * The function doneCbMCL is a callback function that prints the state of the motion_law action server
 * when it finishes.
 * 
 * @param state The parameter "state" is of type "actionlib::SimpleClientGoalState" and represents the
 * current state of the action server. It provides information about the status of the goal, such as
 * whether it is active, succeeded, aborted, etc.
 * @param result The "result" parameter is a pointer to the result of the motion control law action. It
 * is of type "first_assignment::MotionControlLawResultConstPtr", which is a pointer to a constant
 * object of type "MotionControlLawResult" defined in the "ir2324
 */
void doneCbMCL(const actionlib::SimpleClientGoalState& state, const first_assignment::MotionControlLawResultConstPtr& result) {
    ROS_INFO("Action server motion_law finished in state [%s]", state.toString().c_str());
}

class DetectObstaclesServer {

    private:
        // Variables that we will need to detect if the robot is moving or not.
        float prevX, prevY, prevYaw;

    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<first_assignment::DetectObstaclesAction> as_;
        std::string action_name_;
        first_assignment::DetectObstaclesFeedback feedback_;
        first_assignment::DetectObstaclesResult result_;

    public:
        DetectObstaclesServer(std::string name):as_(nh_, name, boost::bind(&DetectObstaclesServer::executeCb, this, _1), false), action_name_(name){
            as_.start();
            
            prevX = 0;
            prevY = 0;
            prevYaw = 0;
        }

        ~DetectObstaclesServer(void){}

        /**
         * The feedbackCb function checks if the robot is moving by comparing the current x, y, and yaw
         * values with the previous ones.
         * 
         * @param feedback The parameter `feedback` is of type
         * `move_base_msgs::MoveBaseFeedbackConstPtr&`, which is a constant pointer to a
         * `MoveBaseFeedback` message. This message contains feedback information from the move_base
         * action server, such as the current position and orientation of the robot.
         */
        void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback){
            
            // Get back current x, y from the move_base feedback.
            float x = feedback->base_position.pose.position.x;
            float y = feedback->base_position.pose.position.y;
            double yaw;

            // Conversion from Quaternion to yaw (roll = 0, pitch = 0).
            tf2::Quaternion tmpQuaternion(feedback->base_position.pose.orientation.x, feedback->base_position.pose.orientation.y, feedback->base_position.pose.orientation.z, feedback->base_position.pose.orientation.w);
            tmpQuaternion.normalize();
            tf2::Matrix3x3 matrix(tmpQuaternion);
            double roll, pitch;
            matrix.getRPY(roll, pitch, yaw);

            // Check if robot is moving by comparing current x, y and yaw values with the previous ones.
            if (x != prevX || y != prevY || yaw != prevYaw)
                feedback_.current_status = "The robot is moving.";
            else
                feedback_.current_status = "The robot is NOT moving.";

            prevX = x;
            prevY = y;
            prevYaw = yaw;

            as_.publishFeedback(feedback_);
        }

        /**
         * The function feedbackCbMCL updates the current status and publishes the feedback.
         * 
         * @param feedback The parameter "feedback" is a pointer to a constant object of type
         * "first_assignment::MotionControlLawFeedbackConstPtr".
         */
        void feedbackCbMCL(const first_assignment::MotionControlLawFeedbackConstPtr& feedback) {
            feedback_.current_status = feedback->current_status;
            as_.publishFeedback(feedback_);
        }        

        /**
         * The function executes a series of actions including motion control, moving to a target pose,
         * and detecting obstacles.
         * 
         * @param goal The "goal" parameter is of type `first_assignment::DetectObstaclesGoalConstPtr`,
         * which is a constant pointer to an object of type `first_assignment::DetectObstaclesGoal`.
         * This object contains the goal information for the action.
         */
        void executeCb(const first_assignment::DetectObstaclesGoalConstPtr &goal){
            
            // MCL part
            actionlib::SimpleActionClient<first_assignment::MotionControlLawAction> actionMCL("motionControlLaw", true);
            actionMCL.waitForServer();

            first_assignment::MotionControlLawGoal MCLGoal;
            MCLGoal.x_pos = goal->x_pos;
            MCLGoal.y_pos = goal->y_pos;
            MCLGoal.yaw = goal->yaw * M_PI / 180.0;
            actionMCL.sendGoal(MCLGoal, &doneCbMCL, &activeCbMCL, boost::bind(&DetectObstaclesServer::feedbackCbMCL, this, _1));
            actionMCL.waitForResult();
            
            // Check if Pose_B has been reached successfully
            if(actionMCL.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
                feedback_.current_status = "An error occurred during MCL moving";
                as_.publishFeedback(feedback_);
            }

            ROS_INFO("MCL done, switching to move_base server");
            feedback_.current_status = "The robot control has switched to move_base server";
            as_.publishFeedback(feedback_);

            // Move base part
            // Connect to move_base server.
            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseClient("move_base", true);
            moveBaseClient.waitForServer();

            // Create the goal for move_base server.
            move_base_msgs::MoveBaseGoal moveBaseGoal;
            moveBaseGoal.target_pose.header.frame_id = "map";
            moveBaseGoal.target_pose.header.stamp = ros::Time::now();
            
            // Position.
            moveBaseGoal.target_pose.pose.position.x = goal->x_pos;
            moveBaseGoal.target_pose.pose.position.y = goal->y_pos;
            moveBaseGoal.target_pose.pose.position.z = 0.0;
            
            // Orientation.
            float yaw = goal->yaw * M_PI / 180.0;
            tf2::Quaternion tmpQuaternion;
            tmpQuaternion.setRPY(0, 0, yaw);
            tmpQuaternion.normalize();
            moveBaseGoal.target_pose.pose.orientation.x = tmpQuaternion.getX();
            moveBaseGoal.target_pose.pose.orientation.y = tmpQuaternion.getY();
            moveBaseGoal.target_pose.pose.orientation.z = tmpQuaternion.getZ();
            moveBaseGoal.target_pose.pose.orientation.w = tmpQuaternion.getW();

            // Send goal.
            moveBaseClient.sendGoal(moveBaseGoal, &doneCb, &activeCb, boost::bind(&DetectObstaclesServer::feedbackCb, this, _1));
            moveBaseClient.waitForResult();

            bool positionReached = true;
            
            // Check if the action succeded.
            if (moveBaseClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                feedback_.current_status = "Tiago has reached Pose_B and stopped.";
                as_.publishFeedback(feedback_);
            } else {
                feedback_.current_status = "Tiago has NOT reached Pose_B and stopped.";
                positionReached = false;
                as_.publishFeedback(feedback_);
                as_.setAborted();
            }

            // Obstacle detection: done only if the robot has reached the point Pose_B.
            if(positionReached) {
                ROS_INFO("Sending obstacle detection request");
                ros::NodeHandle n;
                sensor_msgs::LaserScanConstPtr laserScanMsg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan", n);
                std::vector<std::pair<float, float>> metricPoints = PointUtils::getMetricPoints(laserScanMsg);
                ROS_INFO("Points found by the laser: %ld", metricPoints.size());
                std::vector<std::pair<float, float>> obstaclesPoints = ObstaclesFinder::findObstacles(metricPoints);

                // Plot points in RViz.
                PointUtils::plotPointsRViz(obstaclesPoints, "base_laser_link");

                // Convert the points to the "map" frame.
                std::vector<std::pair<float, float>> obstaclesPointsMapFrame = PointUtils::transformPoints(obstaclesPoints, "base_laser_link", "map");
                
                result_.x_obstacles.clear(); result_.y_obstacles.clear();
                // Save the results.
                for (int i = 0; i < obstaclesPointsMapFrame.size(); i++) {
                    result_.x_obstacles.push_back(obstaclesPointsMapFrame[i].first);
                    result_.y_obstacles.push_back(obstaclesPointsMapFrame[i].second);
                }

                ROS_INFO("%s: Succeeded", action_name_.c_str());
                as_.setSucceeded(result_);
            }
        }
};

/**
 * This C++ code initializes a ROS node called "DetectObstaclesServer" and starts a server for
 * detecting obstacles.
 * 
 * @param argc The argc parameter is an integer that represents the number of command-line arguments
 * passed to the program. It stands for "argument count".
 * @param argv The `argv` parameter is an array of strings that represents the command-line arguments
 * passed to the program. Each element of the array is a null-terminated string. The first element
 * (`argv[0]`) is usually the name of the program itself.
 * 
 * @return The main function is returning an integer value of 0.
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "DetectObstaclesServer");
    DetectObstaclesServer detectObstacles("detectObstacles");
    ROS_INFO("DetectObstaclesServer ready!");
    ros::spin();
    return 0;
}