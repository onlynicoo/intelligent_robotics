/* 
	Group 23: Nicola Lorenzon 2087643, Daniele Moschetta 2087640
	
	This is the implementation of the server for the obstacles detection.
*/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <first_assignment/DetectObstaclesAction.h>
#include "ObstaclesFinder.h"
#include "PointUtils.h"

/**
 * The function "activeCb" logs a message indicating that the move_base action has started.
 */
void activeCb(){
    ROS_INFO("move_base: started");
}

/**
 * The function "doneCb" is a callback function that prints a message when the move_base action is
 * completed.
 * 
 * @param state The parameter "state" is of type "actionlib::SimpleClientGoalState" and represents the
 * state of the goal that was sent to the move_base action server. It provides information about the
 * current state of the goal, such as whether it is active, succeeded, aborted, or preempted.
 * @param result The "result" parameter is a pointer to the result of the move_base action. It is of
 * type move_base_msgs::MoveBaseResultConstPtr, which is a pointer to a constant MoveBaseResult
 * message. This message contains information about the result of the move_base action, such as whether
 * the
 */
void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result){
    ROS_INFO("move_base: ended");
}

class DetectObstaclesServer{

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
         * The feedbackCb function checks if the robot is moving by comparing current and previous x,
         * y, and yaw values.
         * 
         * @param feedback The parameter "feedback" is a pointer to a constant object of type
         * "move_base_msgs::MoveBaseFeedback". This object contains feedback information from the
         * move_base action server, such as the current position and orientation of the robot.
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
            if(x != prevX || y != prevY || yaw != prevYaw){
                feedback_.current_status = "The robot is moving.";
            } else {
                feedback_.current_status = "The robot is NOT moving.";
            }

            prevX = x;
            prevY = y;
            prevYaw = yaw;

            as_.publishFeedback(feedback_);
        }

        /**
         * The function executes a series of actions using the move_base server, checks if the action
         * succeeded, and then performs obstacle detection using laser scan data.
         * 
         * @param goal The `goal` parameter is of type
         * `first_assignment::DetectObstaclesGoalConstPtr`, which is a constant pointer to an
         * object of type `first_assignment::DetectObstaclesGoal`. This object contains the goal
         * information for the `executeCb` function.
         */
        void executeCb(const first_assignment::DetectObstaclesGoalConstPtr &goal){
            
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
            if(moveBaseClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
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
                sensor_msgs::LaserScanConstPtr laserScanMsg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan", nh_);
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
 * This C++ code initializes a ROS node called "DetectObstaclesServer" and runs a server for detecting
 * obstacles.
 * 
 * @param argc The argc parameter is an integer that represents the number of command-line arguments
 * passed to the program. It stands for "argument count".
 * @param argv The `argv` parameter is an array of strings that contains the command-line arguments
 * passed to the program. Each element of the array represents a separate argument.
 * 
 * @return The main function is returning an integer value of 0.
 */
int main(int argc, char **argv){
    ros::init(argc, argv, "DetectObstaclesServer");
    DetectObstaclesServer detectObstacles("detectObstacles");
    ROS_INFO("DetectObstaclesServer ready!");
    ros::spin();
    return 0;
}