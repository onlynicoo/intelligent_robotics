/* 
	Group 23: Nicola Lorenzon 2087643, Daniele Moschetta 2087640
	
	This is the implementation of the Motion Control Law server
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h> 
#include <first_assignment/MotionControlLawAction.h>

const float NARROW_CORRIDOR_MAX_X = 7.0;
const float NARROW_CORRIDOR_MAX_WIDTH = 2.5;

class MotionControlLawServer {

    private:
        ros::Publisher velocityPub;

    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<first_assignment::MotionControlLawAction> as_;
        std::string action_name_;
        first_assignment::MotionControlLawFeedback feedback_;
        first_assignment::MotionControlLawResult result_;
        
    public:
        MotionControlLawServer(std::string name) : 
        as_(nh_, name, boost::bind(&MotionControlLawServer::executeCb, this, _1), false), action_name_(name) {
            as_.start();
            velocityPub = nh_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1);
        }

        ~MotionControlLawServer(void) {}

        /**
         * The function executes a motion control law to navigate a robot to a specific position,
         * checking if the robot is already in the desired position or has already passed a narrow
         * corridor.
         * 
         * @param goal The `goal` parameter is of type `first_assignment::MotionControlLawGoalConstPtr`,
         * which is a constant pointer to a `MotionControlLawGoal` message from the `first_assignment`
         * package. This message contains the goal position (`x_pos` and
         */
        void executeCb(const first_assignment::MotionControlLawGoalConstPtr &goal) {
            
            result_.position_reached = false;

            geometry_msgs::PoseWithCovarianceStampedConstPtr posMsg = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/robot_pose", nh_);

            // Check if the robot is already in Pose_B
            if (abs(posMsg->pose.pose.position.x - goal->x_pos) < 0.2 &&  abs(posMsg->pose.pose.position.y - goal->y_pos) < 0.2) {
                ROS_INFO("Navigation done: robot already in Pose_B");
                feedback_.current_status = "The robot is already in Pose_B";
                as_.publishFeedback(feedback_);
            }
            // Check if the robot has already exited the narrow corridor
            else if (posMsg->pose.pose.position.x > NARROW_CORRIDOR_MAX_X) {
                ROS_INFO("Navigation done: the robot already passed the narrow corridor");
                feedback_.current_status = "The robot already passed the narrow corridor";
                as_.publishFeedback(feedback_);
            } else {
                ros::Rate r(10);
                while (ros::ok()) {

                    sensor_msgs::LaserScanConstPtr laserScanMsg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan", nh_);

                    // Find the nearest point detected by LaserScan
                    int skippedPoints = 20;
                    float left = laserScanMsg->ranges[skippedPoints], right = laserScanMsg->ranges[laserScanMsg->ranges.size() - skippedPoints - 1];
                    float minDist = laserScanMsg->range_max;
                    int minIndex = laserScanMsg->ranges.size();
                    
                    for (int i = skippedPoints; i < laserScanMsg->ranges.size() - skippedPoints; i++)
                        if (laserScanMsg->ranges[i] < minDist) {
                            minDist = laserScanMsg->ranges[i];
                            minIndex = i;
                        }

                    // Set linear velocity
                    geometry_msgs::Twist velocity;
                    velocity.linear.x = 0.3;

                    // Steer towards the center based on if the nearest point is on the left/right
                    float swing = 30;
                    float angularVel = laserScanMsg->angle_max + laserScanMsg->angle_min - laserScanMsg->angle_increment * swing;
                    if (minIndex > laserScanMsg->ranges.size() / 2)
                        velocity.angular.z = angularVel;
                    else if (minIndex < laserScanMsg->ranges.size() / 2)
                        velocity.angular.z = -1 * angularVel;

                    // Publish the velocities
                    feedback_.current_status = "The robot is moving";
                    as_.publishFeedback(feedback_);
                    velocityPub.publish(velocity);

                    // Check if the robot has reached an open room
                    if (laserScanMsg->ranges[skippedPoints] > NARROW_CORRIDOR_MAX_WIDTH
                        || laserScanMsg->ranges[laserScanMsg->ranges.size() - skippedPoints - 1] > NARROW_CORRIDOR_MAX_WIDTH)
                    {
                        result_.position_reached = true;
                        ROS_INFO("Navigation done: sending back response");
                        feedback_.current_status = "The robot navigated with MCL";
                        as_.publishFeedback(feedback_);
                        break;
                    }
                    ros::spinOnce();
                    r.sleep();
                }
            }
            as_.setSucceeded(result_);
        }
};


/**
 * This is the main function that initializes a ROS node, creates a MotionControlLawServer object, and
 * waits for incoming messages.
 * 
 * @param argc The "argc" parameter stands for "argument count" and represents the number of
 * command-line arguments passed to the program when it is executed.
 * @param argv The `argv` parameter is an array of strings that represents the command-line arguments
 * passed to the program. Each element of the array is a null-terminated string.
 * 
 * @return The main function is returning an integer value of 0.
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "motionControlLawServer");
    MotionControlLawServer MotionControlLawServer("motionControlLaw");
    ROS_INFO("Ready to receive Pose_B goal from DetectObstaclesMCLServer");
    ros::spin();
    return 0;
}