/*
	Group 23: Nicola Lorenzon 2087643, Daniele Moschetta 2087640

    TiagoMotion is a library that implements the methods needed to make Tiago navigate
    and move.
*/

#include "TiagoMotion.h"

const float DELAY = 2.0;
const float CARTESIAN_STEP_SIZE = 0.03;
const float CARTESIAN_JUMP_THRESHOLD = 0;
const float GRIPPER_OPEN_JOINT_VALUE = 0.10;
const float GRIPPER_CLOSED_JOINT_VALUE = 0.02;
const float TORSO_DEFAULT_JOINT_VALUE = 0.15;
const float TORSO_UP_JOINT_VALUE = 0.50;
const float HEAD_DEFAULT_JOINT_VALUE = 0;

/**
 * The function "printPose" prints the position and orientation of a given pose in the format
 * "Position: x, y, z; Orientation: yaw, pitch, roll".
 * 
 * @param pose The "pose" parameter is of type geometry_msgs::Pose, which is a message type in ROS
 * (Robot Operating System) used to represent the position and orientation of an object in 3D space. It
 * contains two fields:
 */
void TiagoMotion::printPose(geometry_msgs::Pose pose) {
    tf2::Quaternion orientation;
    tf2::fromMsg(pose.orientation, orientation);
    double roll, pitch, yaw;
    tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    ROS_INFO("Position: x=%f, y=%f, z=%f; Orientation: y=%f, p=%f, r=%f", pose.position.x, pose.position.y, pose.position.z, yaw, pitch, roll);
}

/**
 * The function `getCilinderColor` detects the color of the central pixel in an image and determines if it
 * corresponds to a place table.
 * 
 * @param x The parameter "x" represents the x-coordinate of the point to look at in the map frame.
 * @param y The parameter "y" represents the y-coordinate of the point in the map where the cylinder is
 * located.
 * 
 * @return The function `getCilinderColor` returns an integer value. If a place table is found in the
 * image, the index of the place table (1, 2, or 3) is returned. If no place table is found or an error
 * occurs during the detection process, -1 is returned.
 */
int getCilinderColor(float x, float y) {

    geometry_msgs::PointStamped pointToLook;
    pointToLook.header.frame_id = "map";
    pointToLook.point.x = x;
    pointToLook.point.y = y;
    pointToLook.point.z = WorldValues::PLACE_TABLE_HEIGHT / 2;
    
    TiagoMotion::moveHead(true, pointToLook);

    ros::NodeHandle nh;
    sensor_msgs::ImageConstPtr img_msg = ros::topic::waitForMessage<sensor_msgs::Image>("/xtion/rgb/image_rect_color", nh);

    // Wait for a while to capture the image data
    ros::Duration(1.0).sleep();

    // Convert the ROS image message to a cv::Mat using cv_bridge
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr->image;

    // Access the central pixel of the image
    int centerX = img.cols / 2;
    int centerY = img.rows / 2;

    // Ensure the image is not empty and the central point is within the image bounds
    if (!img.empty() && centerX >= 0 && centerY >= 0 && centerX < img.cols && centerY < img.rows)
    {
        // Access the BGR values of the central pixel
        cv::Vec3b color = img.at<cv::Vec3b>(cv::Point(centerX, centerY));
        uchar blue = color[0];
        uchar green = color[1];
        uchar red = color[2];

        // Print the BGR values
        ROS_INFO("Central pixel color: B=%d, G=%d, R=%d", blue, green, red);

        // Check the color of the object  
        for (int i = 0; i < 3; i++) {
            bool isCorrect = true;
            for (int j = 0; j < 3; j++)
                if ((i!=j) && ((color [i] < 50) || (color[j] > 50)))
                    isCorrect = false;

            // If there are strong indicator for a color than return the index of the corresponding object to place
            if (isCorrect) {
                ROS_INFO("Found place table %d.", i+1);   
                return i+1;
            }
        }
    }
    ROS_ERROR("An error occurred while detecting the place table.");
    return -1;
}

/**
 * The function `moveTorso` moves the torso of a robot up or down based on the `moveUp` parameter.
 * 
 * @param moveUp A boolean value indicating whether the torso should move up or not. If moveUp is true,
 * the torso will move up. If moveUp is false, the torso will move to its default position.
 * 
 * @return a boolean value.
 */
bool TiagoMotion::moveTorso(bool moveUp) {

    // Create client the torso_controller
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> torsoClient("/torso_controller/follow_joint_trajectory", true);
    torsoClient.waitForServer();

    // Set joint values
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("torso_lift_joint");
    trajectory_msgs::JointTrajectoryPoint point;
    if (moveUp)
        point.positions.push_back(TORSO_UP_JOINT_VALUE);
    else
        point.positions.push_back(TORSO_DEFAULT_JOINT_VALUE);
    point.time_from_start = ros::Duration(1.0);
    goal.trajectory.points.push_back(point);

    // Send goal and check the result
    torsoClient.sendGoal(goal);
    if (torsoClient.waitForResult()) {
        ros::Duration(DELAY).sleep();
        ROS_INFO("The torso moved successfully.");
        return true;
    }
    ROS_ERROR("An error occurred while moving the torso.");
    return false;
}


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
bool TiagoMotion::moveHead(bool lookToPos, geometry_msgs::PointStamped pointToLook) {

    if (lookToPos) {

        // Create client for head_controller
        actionlib::SimpleActionClient<control_msgs::PointHeadAction> headClient("/head_controller/point_head_action", true);
        headClient.waitForServer();

        // Set target pose
        geometry_msgs::PointStamped targetPoint;

        targetPoint = pointToLook;
        targetPoint.header.stamp = ros::Time::now();

        // Set target pointing_axis
        control_msgs::PointHeadGoal goal;
        goal.pointing_frame = "/xtion_rgb_optical_frame";

        // Z-axis to point at the target
        goal.pointing_axis.x = 0.0;
        goal.pointing_axis.y = 0.0;
        goal.pointing_axis.z = 1.0;    
        goal.target = targetPoint;
        goal.min_duration = ros::Duration(1.0);

        // Send goal and check the result
        headClient.sendGoal(goal);
        if (headClient.waitForResult()) {
            ros::Duration(DELAY).sleep();
            ROS_INFO("The head moved successfully to requested pose.");
            return true;
        }
        ROS_ERROR("An error occurred while moving the head to requested pose.");
        return false;
        
    } else {

        // Create client for head_controller
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> headClient("/head_controller/follow_joint_trajectory", true);
        headClient.waitForServer();

        // Set joint values
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.joint_names.push_back("head_1_joint");
        goal.trajectory.joint_names.push_back("head_2_joint");
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.push_back(HEAD_DEFAULT_JOINT_VALUE);
        point.positions.push_back(HEAD_DEFAULT_JOINT_VALUE);
        point.time_from_start = ros::Duration(1.0);
        goal.trajectory.points.push_back(point);

        // Send goal and check result
        headClient.sendGoal(goal);
        if (headClient.waitForResult()) {
            ros::Duration(DELAY).sleep();
            ROS_INFO("The head moved successfully to default pose.");
            return true;
        }
        ROS_ERROR("An error occurred while moving the head to default pose.");
        return false;
    }
}

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
bool TiagoMotion::moveGroupToPoseTarget(moveit::planning_interface::MoveGroupInterface& moveGroup, moveit::planning_interface::MoveGroupInterface::Plan& plan, geometry_msgs::Pose& targetPose) {

    // Compute motion plan
    moveGroup.setPoseTarget(targetPose);
    moveit::core::MoveItErrorCode planResult = moveGroup.plan(plan);
    if (planResult) {

        // If it is successful, execute it
        ROS_INFO("The arm is moving to pose target. (%s)", moveit::core::MoveItErrorCode::toString(planResult));
        moveGroup.move();
        ros::Duration(DELAY).sleep();
        return true;
    }
    ROS_ERROR("An error occurred while trying to move the arm to pose target. (%s)", moveit::core::MoveItErrorCode::toString(planResult));
    return false;
}

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
bool TiagoMotion::moveGroupToNamedTarget(moveit::planning_interface::MoveGroupInterface& moveGroup, moveit::planning_interface::MoveGroupInterface::Plan& plan, std::string targetName) {

    // Compute motion plan
    moveGroup.setNamedTarget(targetName);
    moveit::core::MoveItErrorCode planResult = moveGroup.plan(plan);
    if (planResult) {

        // If it is successful, execute it
        ROS_INFO("The arm is moving to named target '%s'. (%s)", targetName.c_str(), moveit::core::MoveItErrorCode::toString(planResult));
        moveGroup.move();
        ros::Duration(DELAY).sleep();
        return true;
    }
    ROS_ERROR("An error occurred while trying to move the arm to named target '%s'. (%s)", targetName.c_str(), moveit::core::MoveItErrorCode::toString(planResult));
    return false;
}

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
float TiagoMotion::moveGroupLinear(moveit::planning_interface::MoveGroupInterface& moveGroup, moveit::planning_interface::MoveGroupInterface::Plan& plan, geometry_msgs::Pose& targetPose) {
    
    // Compute cartesian path
    moveit_msgs::RobotTrajectory path;
    float pathFraction = moveGroup.computeCartesianPath(std::vector<geometry_msgs::Pose>{targetPose}, CARTESIAN_STEP_SIZE, CARTESIAN_JUMP_THRESHOLD, path);
    if (pathFraction > 0) {

        // If it is successful, execute it
        ROS_INFO("Arm is moving linearly, %d%% of the path will be completed.", int(pathFraction * 100));
        plan.trajectory_ = path;
        moveGroup.execute(plan);
        ros::Duration(DELAY).sleep();
        return pathFraction;
    }
    ROS_ERROR("Arm is NOT moving linearly.");
    return pathFraction;
}

/**
 * The function `moveGripper` is used to control the movement of a gripper in a parallel gripper
 * controller.
 * 
 * @param open A boolean value indicating whether the gripper should be opened or closed. If `open` is
 * `true`, the gripper will be opened. If `open` is `false`, the gripper will be closed.
 * 
 * @return a boolean value.
 */
bool TiagoMotion::moveGripper(bool open) {

    // Create client for parallel_gripper_controller
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gripperClient("/parallel_gripper_controller/follow_joint_trajectory", true);
    gripperClient.waitForServer();

    // Set joint values
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
    goal.trajectory.joint_names.push_back("gripper_right_finger_joint");
    trajectory_msgs::JointTrajectoryPoint point;
    if (open) {
        point.positions.push_back(GRIPPER_OPEN_JOINT_VALUE);
        point.positions.push_back(GRIPPER_OPEN_JOINT_VALUE);
    } else {
        point.positions.push_back(GRIPPER_CLOSED_JOINT_VALUE);
        point.positions.push_back(GRIPPER_CLOSED_JOINT_VALUE);
    }
    point.time_from_start = ros::Duration(2.5);
    goal.trajectory.points.push_back(point);

    // Send goal and check result
    gripperClient.sendGoal(goal);
    if (gripperClient.waitForResult()) {
        ros::Duration(DELAY).sleep();
        ROS_INFO("Gripper moved successfully.");
        return true;
    }
    ROS_ERROR("An error occurred while moving the gripper.");
    return false;
}

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
bool TiagoMotion::moveToPose(const std::vector<std::vector<float>>& waypoints, const std::vector<float>& goalPose) {
    
	// Initialize the DetectObstaclesClient
	actionlib::SimpleActionClient<first_assignment::DetectObstaclesAction> actionDetectObstalces("detectObstacles", true);
    ROS_INFO("Waiting for Tiago move server to start.");
	actionDetectObstalces.waitForServer();

    ROS_INFO("Started moving.");

    // Create goal to move the robot
	first_assignment::DetectObstaclesAction goal_pose;

    for (int i = 0; i < waypoints.size(); i++) {

        // Send the goal to the server
        first_assignment::DetectObstaclesGoal goal;
        goal.x_pos = waypoints[i][0];
        goal.y_pos = waypoints[i][1];
        goal.yaw = waypoints[i][2];

        // Push the goal to the move_base node
        actionDetectObstalces.sendGoal(goal);
        actionDetectObstalces.waitForResult();
    }

    first_assignment::DetectObstaclesGoal goal;
    goal.x_pos = goalPose[0];
    goal.y_pos = goalPose[1];
    goal.yaw = goalPose[2];

    // Push the goal to the move_base node
    actionDetectObstalces.sendGoal(goal);
    actionDetectObstalces.waitForResult();

    return true;
}

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
bool TiagoMotion::moveToPoseWithDetection(const std::vector<std::vector<float>>& waypoints, const std::vector<float>& goalPose, bool saveDetection, std::vector<std::vector<float>>& bgrPose) {

	// Initialize the DetectObstaclesClient
	actionlib::SimpleActionClient<first_assignment::DetectObstaclesAction> actionDetectObstalces("detectObstacles", true);
	actionDetectObstalces.waitForServer();

    ROS_INFO("Started moving.");

    // Create goal to move the robot
	first_assignment::DetectObstaclesAction goal_pose;

    for (int i = 0; i < waypoints.size(); i++) {

        // Send the goal to the server
        first_assignment::DetectObstaclesGoal goal;
        goal.x_pos = waypoints[i][0];
        goal.y_pos = waypoints[i][1];
        goal.yaw = waypoints[i][2];

        // Push the goal to the move_base node
        actionDetectObstalces.sendGoal(goal);
        actionDetectObstalces.waitForResult();
    }

    first_assignment::DetectObstaclesGoal goal;
    goal.x_pos = goalPose[0];
    goal.y_pos = goalPose[1];
    goal.yaw = goalPose[2];

    // Push the goal to the move_base node
    actionDetectObstalces.sendGoal(goal);
    actionDetectObstalces.waitForResult();

    ROS_INFO("Detection pose reached.");

    // If needed to save scan detection
    if (saveDetection == true) {
        bgrPose = std::vector<std::vector<float>>(3);
        first_assignment::DetectObstaclesResult pts = *actionDetectObstalces.getResult();
        
        std::vector<float> xs = pts.x_obstacles;
        std::vector<float> ys = pts.y_obstacles;

        ROS_INFO("Started place positions detection.");
        
        // For each detected pose
        for (int i = 0; i < xs.size(); i++) {
            // If it's in the room 
            if (xs[i] > WorldValues::WALL_X) {
                // Check the color and save the index
                int seenCilinder = getCilinderColor(xs[i], ys[i]);
                if (seenCilinder != -1) {
                    bgrPose[seenCilinder-1] = std::vector<float>{xs[i], ys[i]};
                }
            }
        }   
    }
    ROS_INFO("Place positions detected successfully.");
    return true;
}