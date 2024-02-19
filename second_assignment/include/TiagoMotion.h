/*
	Group 23: Nicola Lorenzon 2087643, Daniele Moschetta 2087640

    TiagoMotion is a library that implements the methods needed to make Tiago navigate
    and move.
*/

#pragma once

#include <control_msgs/PointHeadAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <first_assignment/DetectObstaclesAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "WorldValues.h"

class TiagoMotion {
    public:
        /**
         * The function "printPose" prints the position and orientation of a given pose in the format
         * "Position: x, y, z; Orientation: yaw, pitch, roll".
         * 
         * @param pose The "pose" parameter is of type geometry_msgs::Pose, which is a message type in ROS
         * (Robot Operating System) used to represent the position and orientation of an object in 3D space. It
         * contains two fields:
         */
        static void printPose(geometry_msgs::Pose pose);
        
        /**
         * The function `moveTorso` moves the torso of a robot up or down based on the `moveUp` parameter.
         * 
         * @param moveUp A boolean value indicating whether the torso should move up or not. If moveUp is true,
         * the torso will move up. If moveUp is false, the torso will move to its default position.
         * 
         * @return a boolean value.
         */
        static bool moveTorso(bool moveUp);

        /**
         * The function `moveHead` moves the head of a robot either to a specified position or to the default
         * position.
         * 
         * @param lookToPos The parameter "lookToPos" is a boolean value that determines whether the head
         * should move to a specific position or to the default position. If "lookToPos" is true, the head will
         * move to the position specified by the "pointToLook" parameter. If "lookToPos" if false, the head will 
         * move to a predefined position.
         * @param pointToLook The parameter "pointToLook" is of type geometry_msgs::PointStamped and represents
         * the target point to look at.
         * 
         * @return a boolean value. It returns true if the head movement was successful and false if there was
         * an error.
         */
        static bool moveHead(bool lookToPos, geometry_msgs::PointStamped pointToLook = geometry_msgs::PointStamped());
        

        /**
         * The function `moveGroupToPoseTarget` plans and executes a motion to move a robot arm to a specified
         * pose target using MoveIt.
         * 
         * @param moveGroup The moveGroup parameter is an instance of the MoveGroupInterface class from the
         * MoveIt! library. It is used to interface with a specific group of joints in the robot's kinematic
         * model.
         * @param plan The "plan" parameter is of type `moveit::planning_interface::MoveGroupInterface::Plan&`,
         * which is a reference to a `Plan` object. This object is used to store the computed motion plan for
         * the robot arm.
         * @param targetPose The targetPose parameter is of type geometry_msgs::Pose and represents the desired
         * pose that the arm should move to. It contains the position and orientation information of the target
         * pose in 3D space.
         * 
         * @return a boolean value. It returns true if the motion plan is successful and the arm is moved to
         * the pose target, and false if there is an error while trying to move the arm to the pose target.
         */
        static bool moveGroupToPoseTarget(moveit::planning_interface::MoveGroupInterface& armGroup, moveit::planning_interface::MoveGroupInterface::Plan& plan, geometry_msgs::Pose& targetPose);
        
        /**
         * The function `moveGroupToNamedTarget` moves a robot arm to a named target using MoveIt library.
         * 
         * @param moveGroup The moveGroup parameter is an instance of the MoveGroupInterface class from the
         * MoveIt! library. It is used to interface with a specific group of joints in the robot's kinematic
         * model.
         * @param plan The "plan" parameter is of type `moveit::planning_interface::MoveGroupInterface::Plan&`,
         * which is a reference to a `Plan` object. This object is used to store the computed motion plan for
         * the robot arm.
         * @param targetName The targetName parameter is a string that represents the name of the target
         * position or configuration that you want the robot arm to move to.
         * 
         * @return a boolean value. It returns true if the motion plan is successfully computed and executed,
         * and false if an error occurs during the process.
         */
        static bool moveGroupToNamedTarget(moveit::planning_interface::MoveGroupInterface& armGroup, moveit::planning_interface::MoveGroupInterface::Plan& plan, std::string targetName);
        
        /**
         * The function moveGroupLinear computes a linear cartesian path for a robot arm and executes it if
         * successful.
         * 
         * @param moveGroup The moveGroup parameter is an instance of the MoveGroupInterface class from the
         * MoveIt! library. It is used to interact with a specific group of joints in the robot's
         * configuration.
         * @param plan The `plan` parameter is of type `moveit::planning_interface::MoveGroupInterface::Plan`
         * and is used to store the computed trajectory for the robot's motion. It is passed by reference to
         * the function so that the computed trajectory can be assigned to it.
         * @param targetPose The targetPose parameter is of type geometry_msgs::Pose and represents the desired
         * end pose of the robot's end effector. It contains the position and orientation of the end effector
         * in 3D space.
         * 
         * @return a float value, which represents the fraction of the computed cartesian path that can be
         * executed successfully.
         */
        static float moveGroupLinear(moveit::planning_interface::MoveGroupInterface& armGroup, moveit::planning_interface::MoveGroupInterface::Plan& plan, geometry_msgs::Pose& targetPose);
        
        /**
         * The function `moveGripper` is used to control the movement of a gripper in a parallel gripper
         * controller.
         * 
         * @param open A boolean value indicating whether the gripper should be opened or closed. If `open` is
         * `true`, the gripper will be opened. If `open` is `false`, the gripper will be closed.
         * 
         * @return a boolean value.
         */
        static bool moveGripper(bool open);
        
        /**
         * The function `moveToPose` moves the Tiago robot to a series of waypoints and then to a goal pose
         * using the `DetectObstaclesAction` action client.
         * 
         * @param waypoints A vector of vectors containing the x, y, and yaw values for each waypoint. Each
         * inner vector represents a single waypoint.
         * @param goalPose The goalPose parameter is a vector of floats that represents the final desired pose
         * of the robot. It contains three elements: goalPose[0] represents the x position, goalPose[1]
         * represents the y position, and goalPose[2] represents the yaw angle (rotation around the z-axis)
         * 
         * @return a boolean value, which is true.
         */
        static bool moveToPose(const std::vector<std::vector<float>>& waypoints, const std::vector<float>& goalPose);
        
        /**
         * The function `moveToPoseWithDetection` moves the robot to a series of waypoints and detects
         * obstacles along the way, saving the detected positions if specified.
         * 
         * @param waypoints A vector of vectors representing the waypoints for the robot to move to. Each inner
         * vector contains three float values: x position, y position, and yaw (orientation).
         * @param goalPose The goalPose parameter is a vector of floats that represents the desired final pose
         * of the robot. It contains three elements: goalPose[0] represents the x position, goalPose[1]
         * represents the y position, and goalPose[2] represents the yaw angle (rotation) of the robot.
         * @param saveDetection A boolean flag indicating whether to save the detected obstacle positions or
         * not.
         * @param bgrPose bgrPose is a reference to a vector of vectors of floats. It is an output parameter
         * that will be filled with the detected positions of the obstacles in the environment. Each inner
         * vector represents the position of an obstacle and contains two float values: the x-coordinate and
         * the y-coordinate.
         * 
         * @return a boolean value, which is true.
         */
        static bool moveToPoseWithDetection(const std::vector<std::vector<float>>& waypoints, const std::vector<float>& goalPose, bool saveDetection, std::vector<std::vector<float>>& bgrPose);
};