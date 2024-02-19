/* 
	Group 23: Nicola Lorenzon 2087643, Daniele Moschetta 2087640

    nodeC is an action client that handles the pick and place routine.
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <shape_msgs/SolidPrimitive.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/Pose.h>
#include <second_assignment/PickAndPlaceAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "TiagoMotion.h"
#include "WorldValues.h"

const float DELAY = 1.0;
const float COLLISION_PADDING = 0.05;
const float APPROACH_DISTANCE = 0.30;
const float LEAVE_DISTANCE = 0.02;

class PickAndPlaceServer {

    private:
        /**
         * The function `attachObjectToGripper` attaches or detaches an object to/from a gripper in a
         * simulated environment using ROS.
         * 
         * @param attach A boolean value indicating whether to attach or detach the object to/from the
         * gripper. If `attach` is true, the object will be attached. If `attach` is false, the object
         * will be detached.
         * @param objectId The `objectId` parameter is an integer that represents the ID of the object
         * you want to attach or detach to the gripper. The specific object IDs and their corresponding
         * models and link names are defined in the `switch` statement in the code.
         * 
         * @return a boolean value.
         */
        bool attachObjectToGripper(bool attach, int objectId) {
            std::string serviceName = "/link_attacher_node/attach";
            if (!attach)
                serviceName = "/link_attacher_node/detach";
            ros::ServiceClient attachClient = nh_.serviceClient<gazebo_ros_link_attacher::Attach>(serviceName);

            gazebo_ros_link_attacher::Attach attachRequest;
            attachRequest.request.model_name_1 = "tiago";
            attachRequest.request.link_name_1 = "arm_7_link";

            switch(objectId) {
                case 1:
                    attachRequest.request.model_name_2 = "Hexagon";
                    attachRequest.request.link_name_2 = "Hexagon_link";
                    break;
                case 2:
                    attachRequest.request.model_name_2 = "Triangle";
                    attachRequest.request.link_name_2 = "Triangle_link";
                    break;
                case 3:
                    attachRequest.request.model_name_2 = "cube";
                    attachRequest.request.link_name_2 = "cube_link";
                    break;
                default:
                    break;
            }

            if(attachClient.call(attachRequest)) {
                if (attach)
                ROS_INFO("Object link successfully attached.");
                else
                    ROS_INFO("Object link successfully detached.");
                return true;
            }
            if (attach)
                ROS_ERROR("An error occurred while attaching object link.");
            else
                ROS_ERROR("An error occurred while detaching object link.");
            return false;
        }

    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<second_assignment::PickAndPlaceAction> as_;
        std::string action_name_;
        second_assignment::PickAndPlaceFeedback feedback_;
        second_assignment::PickAndPlaceResult result_;

    public:
        PickAndPlaceServer(std::string name):as_(nh_, name, boost::bind(&PickAndPlaceServer::executeCb, this, _1), false), action_name_(name){
            as_.start();
        }
        ~PickAndPlaceServer(void){}

        /**
         * The executeCb function is responsible for executing the pick and place routine based on the
         * given goal.
         * 
         * @param goal The "goal" parameter is a pointer to a constant object of type
         * "PickAndPlaceGoalConstPtr".
         */
        void executeCb(const second_assignment::PickAndPlaceGoalConstPtr &goal){

            result_.succeded = false;

            // Move group setup
            moveit::planning_interface::MoveGroupInterface moveGroup("arm_torso");
            moveGroup.setPoseReferenceFrame("base_footprint");
            moveGroup.setPlanningTime(10.0);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
            std::vector<std::string> collisionObjectsIds;

            if (goal->pick) {

                feedback_.current_status = "Pick routine has started.";
                as_.publishFeedback(feedback_);

                // Define a collision object for every object you detect and for the table
                int targetIndex = -1;
                float targetHeight = -1, targetYawOffset = 0;
                moveit_msgs::CollisionObject targetCollisionObject;
                std::vector<geometry_msgs::Pose> poses = goal->poses.poses;

                // Add objects collision objects
                for (int i = 0; i < goal->ids.size(); i++) {

                    if (goal->ids[i] == goal->target_id)
                        targetIndex = i;
                    
                    // Create a collision object
                    moveit_msgs::CollisionObject curCollisionObject;
                    curCollisionObject.header.frame_id = "base_footprint";
                    shape_msgs::SolidPrimitive primitive;
                    tf2::Quaternion triangleOrientation;

                    switch (goal->ids[i]) {
                        case 1:
                            curCollisionObject.id = "blue_hexagon";
                            primitive.type = primitive.CYLINDER;
                            primitive.dimensions.resize(2);
                            primitive.dimensions[0] = WorldValues::BLUE_HEXAGON_HEIGHT + COLLISION_PADDING;  // Height
                            primitive.dimensions[1] = WorldValues::BLUE_HEXAGON_RADIUS + COLLISION_PADDING;  // Radius
                            poses[i].position.z -= WorldValues::BLUE_HEXAGON_HEIGHT / 2;
                            if (goal->target_id == goal->ids[i]) {
                                targetHeight = WorldValues::BLUE_HEXAGON_HEIGHT;
                                targetYawOffset = M_PI / 4;
                            }
                            break;
                        case 2:
                            curCollisionObject.id = "green_triangle";
                            primitive.type = primitive.BOX;
                            primitive.dimensions.resize(3);
                            primitive.dimensions[0] = WorldValues::GREEN_TRIANGLE_BASE_X + COLLISION_PADDING;  // Base X
                            primitive.dimensions[1] = WorldValues::GREEN_TRIANGLE_BASE_Y + COLLISION_PADDING;  // Base Y
                            primitive.dimensions[2] = WorldValues::GREEN_TRIANGLE_HEIGHT + COLLISION_PADDING;  // Height

                            // Adjust position error due to tag placing in the triangle
                            tf2::fromMsg(poses[i].orientation, triangleOrientation);
                            double roll, pitch, yaw;
                            tf2::Matrix3x3(triangleOrientation).getRPY(roll, pitch, yaw);
                            poses[i].position.x += std::sin(yaw) * WorldValues::GREEN_TRIANGLE_BASE_Y/4;
                            poses[i].position.y -= std::cos(yaw) * WorldValues::GREEN_TRIANGLE_BASE_Y/4;

                            poses[i].position.z -= WorldValues::GREEN_TRIANGLE_HEIGHT / 2;
                            if (goal->target_id == goal->ids[i])
                                targetHeight = WorldValues::GREEN_TRIANGLE_HEIGHT;
                            break;
                        case 3:
                            curCollisionObject.id = "red_cube";
                            primitive.type = primitive.BOX;
                            primitive.dimensions.resize(3);
                            primitive.dimensions[0] = WorldValues::RED_CUBE_SIZE + COLLISION_PADDING;
                            primitive.dimensions[1] = WorldValues::RED_CUBE_SIZE + COLLISION_PADDING;
                            primitive.dimensions[2] = WorldValues::RED_CUBE_SIZE + COLLISION_PADDING;
                            poses[i].position.z -= WorldValues::RED_CUBE_SIZE / 2;
                            if (goal->target_id == goal->ids[i])
                                targetHeight = WorldValues::RED_CUBE_SIZE;
                            break;
                        default:
                            if (goal->ids[i] > 7)
                                break;
                            curCollisionObject.id = "gold_obs_" + std::to_string(goal->ids[i] - 4);
                            primitive.type = primitive.CYLINDER;
                            primitive.dimensions.resize(2);
                            primitive.dimensions[0] = WorldValues::GOLD_OBS_HEIGHT + COLLISION_PADDING;  // Height
                            primitive.dimensions[1] = WorldValues::GOLD_OBS_RADIUS + COLLISION_PADDING;  // Radius
                            poses[i].position.z -= WorldValues::GOLD_OBS_HEIGHT / 2;
                            break;
                    }

                    curCollisionObject.primitives.push_back(primitive);
                    curCollisionObject.primitive_poses.push_back(poses[i]);
                    curCollisionObject.operation = curCollisionObject.ADD;
                    planningSceneInterface.applyCollisionObject(curCollisionObject);
                    collisionObjectsIds.push_back(curCollisionObject.id);

                    if (goal->target_id == goal->ids[i])
                        targetCollisionObject = curCollisionObject;
                }

                if (targetHeight != -1) {

                    // Add pick table collision object
                    moveit_msgs::CollisionObject tableCollisionObject;
                    tableCollisionObject.header.frame_id = "map";
                    shape_msgs::SolidPrimitive primitive;
                    tableCollisionObject.id = "pick_table";

                    // Shape
                    primitive.type = primitive.BOX;
                    primitive.dimensions.resize(3);
                    primitive.dimensions[0] = WorldValues::PICK_TABLE_BASE_X + COLLISION_PADDING;
                    primitive.dimensions[1] = WorldValues::PICK_TABLE_BASE_Y + COLLISION_PADDING;
                    primitive.dimensions[2] = WorldValues::PICK_TABLE_HEIGHT + COLLISION_PADDING;

                    // Pose
                    geometry_msgs::Pose tablePose;
                    tablePose.position.x = WorldValues::PICK_TABLE_X;
                    tablePose.position.y = WorldValues::PICK_TABLE_Y;
                    tablePose.position.z = WorldValues::PICK_TABLE_HEIGHT / 2;
                    tablePose.orientation.w = 1.0; // Default orientation (no rotation)
                    tableCollisionObject.primitives.push_back(primitive);
                    tableCollisionObject.primitive_poses.push_back(tablePose);
                    tableCollisionObject.operation = tableCollisionObject.ADD;
                    planningSceneInterface.applyCollisionObject(tableCollisionObject);
                    collisionObjectsIds.push_back(tableCollisionObject.id);

                    // Ensure collision objects are updated before proceeding
                    ros::Duration(DELAY).sleep();

                    // Assign an initial configuration to the Tiago’s arm
                    moveGroup.rememberJointValues("safe_pose");

                    // Make the arm move in a target position
                    geometry_msgs::Pose targetPose = poses[targetIndex];

                    // Point gripper
                    tf2::Quaternion targetOrientation;
                    tf2::fromMsg(targetPose.orientation, targetOrientation);
                    double roll, pitch, yaw;
                    tf2::Matrix3x3(targetOrientation).getRPY(roll, pitch, yaw);
                    roll = 0;
                    pitch = M_PI / 2.0;
                    yaw += targetYawOffset;
                    targetOrientation.setRPY(roll, pitch, yaw);
                    tf2::convert(targetOrientation, targetPose.orientation);

                    // Set approach height
                    geometry_msgs::Pose approachPose = targetPose;
                    approachPose.position.z += targetHeight / 2 + APPROACH_DISTANCE;

                    // Move arm
                    TiagoMotion::moveGroupToPoseTarget(moveGroup, plan, approachPose);

                    // Reduce the collision object of the target object
                    planningSceneInterface.removeCollisionObjects(std::vector<std::string>{targetCollisionObject.id});
                    moveit_msgs::CollisionObject exactCollisionObject = targetCollisionObject;
                    for (int i = 0; i < exactCollisionObject.primitives[0].dimensions.size(); i++)
                        exactCollisionObject.primitives[0].dimensions[i] -= COLLISION_PADDING;
                    planningSceneInterface.applyCollisionObject(exactCollisionObject);

                    // Ensure collision objects are updated before proceeding
                    ros::Duration(DELAY).sleep();

                    // Through a linear movement complete the grasping
                    targetPose.position.z += WorldValues::TIAGO_GRIPPER_LENGTH;
                    TiagoMotion::moveGroupLinear(moveGroup, plan, targetPose);

                    // Remove the target object from the collision objects
                    planningSceneInterface.removeCollisionObjects(std::vector<std::string>{exactCollisionObject.id});

                    // Attach virtually the object to the gripper
                    attachObjectToGripper(true, goal->target_id);

                    // Close the gripper
                    TiagoMotion::moveGripper(false);

                    // Come back to the previous target position through a linear movement of the arm
                    TiagoMotion::moveGroupLinear(moveGroup, plan, approachPose);

                    // Add collision obstacle around the gripper
                    moveit_msgs::CollisionObject pickedCollisionObject;
                    pickedCollisionObject.header.frame_id = "arm_tool_link";
                    shape_msgs::SolidPrimitive pickedPrimitive;
                    pickedCollisionObject.id = "picked_object";

                    // Shape
                    pickedPrimitive = targetCollisionObject.primitives[0];

                    // Pose
                    geometry_msgs::Pose pickedPose;
                    pickedPose.position.x = WorldValues::TIAGO_GRIPPER_LENGTH;
                    pickedPose.position.y = 0;
                    pickedPose.position.z = 0;

                    // Orientation
                    tf2::Quaternion pickedOrientation;
                    pickedOrientation.setRPY(0, M_PI / 2.0, 0);
                    tf2::convert(pickedOrientation, pickedPose.orientation);
                    pickedCollisionObject.primitives.push_back(pickedPrimitive);
                    pickedCollisionObject.primitive_poses.push_back(pickedPose);
                    pickedCollisionObject.operation = pickedCollisionObject.ADD;
                    planningSceneInterface.applyCollisionObject(pickedCollisionObject);
                    moveGroup.attachObject("picked_object", "arm_tool_link", {"gripper_link", "gripper_base_link", "gripper_tool_link", "gripper_left_finger_link", "gripper_right_finger_link"});

                    // Ensure collision objects are updated before proceeding
                    ros::Duration(DELAY).sleep();

                    // Move the arm to a secure pose
                    TiagoMotion::moveGroupToNamedTarget(moveGroup, plan, "safe_pose");

                    result_.succeded = true;
                } else
                    ROS_ERROR("Object requested not found on the table.");

                feedback_.current_status = "Pick routine has ended.";
                as_.publishFeedback(feedback_);

            } else {

                feedback_.current_status = "Place routine has started.";
                as_.publishFeedback(feedback_);

                // Create collision object for place object that will be added to the scene later
                moveit_msgs::CollisionObject objectCollisionObject;
                objectCollisionObject.header.frame_id = "map";
                shape_msgs::SolidPrimitive objectPrimitive;
                tf2::Quaternion triangleOrientation;
                float targetHeight = -1;

                switch (goal->target_id) {
                    case 1:
                        objectCollisionObject.id = "blue_hexagon";
                        objectPrimitive.type = objectPrimitive.CYLINDER;
                        objectPrimitive.dimensions.resize(2);
                        objectPrimitive.dimensions[0] = WorldValues::BLUE_HEXAGON_HEIGHT + COLLISION_PADDING;  // Height
                        objectPrimitive.dimensions[1] = WorldValues::BLUE_HEXAGON_RADIUS + COLLISION_PADDING;  // Radius
                        targetHeight = WorldValues::BLUE_HEXAGON_HEIGHT;
                        break;
                    case 2:
                        objectCollisionObject.id = "green_triangle";
                        objectPrimitive.type = objectPrimitive.BOX;
                        objectPrimitive.dimensions.resize(3);
                        objectPrimitive.dimensions[0] = WorldValues::GREEN_TRIANGLE_BASE_X + COLLISION_PADDING;  // Base X
                        objectPrimitive.dimensions[1] = WorldValues::GREEN_TRIANGLE_BASE_Y + COLLISION_PADDING;  // Base Y
                        objectPrimitive.dimensions[2] = WorldValues::GREEN_TRIANGLE_HEIGHT + COLLISION_PADDING;  // Height
                        targetHeight = WorldValues::GREEN_TRIANGLE_HEIGHT;
                        break;
                    case 3:
                        objectCollisionObject.id = "red_cube";
                        objectPrimitive.type = objectPrimitive.BOX;
                        objectPrimitive.dimensions.resize(3);
                        objectPrimitive.dimensions[0] = WorldValues::RED_CUBE_SIZE + COLLISION_PADDING;
                        objectPrimitive.dimensions[1] = WorldValues::RED_CUBE_SIZE + COLLISION_PADDING;
                        objectPrimitive.dimensions[2] = WorldValues::RED_CUBE_SIZE + COLLISION_PADDING;
                        targetHeight = WorldValues::RED_CUBE_SIZE;
                        break;
                    default:
                        break;
                }

                if (targetHeight != -1) {

                    moveGroup.setPoseReferenceFrame("map");

                    moveGroup.rememberJointValues("safe_pose");

                    // Add pose tables collision objects
                    moveit_msgs::CollisionObject tableCollisionObject;
                    tableCollisionObject.header.frame_id = "map";
                    shape_msgs::SolidPrimitive tablePrimitive;

                    // Shape
                    tablePrimitive.type = tablePrimitive.CYLINDER;
                    tablePrimitive.dimensions.resize(2);
                    tablePrimitive.dimensions[0] = WorldValues::PLACE_TABLE_HEIGHT + COLLISION_PADDING;    // Height
                    tablePrimitive.dimensions[1] = WorldValues::PLACE_TABLE_RADIUS + COLLISION_PADDING;      // Radius

                    // Pose
                    geometry_msgs::Pose tablePose;
                    tablePose.position.z = WorldValues::PLACE_TABLE_HEIGHT / 2;
                    tablePose.orientation.w = 1.0; // Default orientation (no rotation)

                    // Blue
                    tableCollisionObject.id = "place_table_b";
                    tablePose.position.x = WorldValues::PLACE_TABLE_BLUE_X;
                    tablePose.position.y = WorldValues::PLACE_TABLE_BLUE_Y;
                    tableCollisionObject.primitives.push_back(tablePrimitive);
                    tableCollisionObject.primitive_poses.push_back(tablePose);
                    tableCollisionObject.operation = tableCollisionObject.ADD;
                    planningSceneInterface.applyCollisionObject(tableCollisionObject);
                    collisionObjectsIds.push_back(tableCollisionObject.id);

                    // Green
                    tableCollisionObject.id = "place_table_g";
                    tablePose.position.x = WorldValues::PLACE_TABLE_GREEN_X;
                    tablePose.position.y = WorldValues::PLACE_TABLE_GREEN_Y;
                    tableCollisionObject.primitives.push_back(tablePrimitive);
                    tableCollisionObject.primitive_poses.push_back(tablePose);
                    tableCollisionObject.operation = tableCollisionObject.ADD;
                    planningSceneInterface.applyCollisionObject(tableCollisionObject);
                    collisionObjectsIds.push_back(tableCollisionObject.id);

                    // Red
                    tableCollisionObject.id = "place_table_r";
                    tablePose.position.x = WorldValues::PLACE_TABLE_RED_X;
                    tablePose.position.y = WorldValues::PLACE_TABLE_RED_Y;
                    tableCollisionObject.primitives.push_back(tablePrimitive);
                    tableCollisionObject.primitive_poses.push_back(tablePose);
                    tableCollisionObject.operation = tableCollisionObject.ADD;
                    planningSceneInterface.applyCollisionObject(tableCollisionObject);
                    collisionObjectsIds.push_back(tableCollisionObject.id);

                    // Ensure collision objects are updated before proceeding
                    ros::Duration(DELAY).sleep();

                    // Place the object on the top of the table
                    geometry_msgs::Pose placePose;
                    placePose.position.x = goal->place_pose.position.x;
                    placePose.position.y = goal->place_pose.position.y;
                    placePose.position.z = WorldValues::PLACE_TABLE_HEIGHT + targetHeight / 2;

                    // Point gripper down
                    tf2::Quaternion placeOrientation;
                    placeOrientation.setRPY(0, M_PI / 2.0, 0);
                    tf2::convert(placeOrientation, placePose.orientation);

                    // Move arm to approach pose
                    geometry_msgs::Pose approachPose = placePose;
                    approachPose.position.z += targetHeight / 2 + APPROACH_DISTANCE;
                    TiagoMotion::moveGroupToPoseTarget(moveGroup, plan, approachPose);

                    // Detach the object collision obstacle
                    moveGroup.detachObject("picked_object");
                    planningSceneInterface.removeCollisionObjects(std::vector<std::string>{"picked_object"});

                    // Ensure collision objects are updated before proceeding
                    ros::Duration(DELAY).sleep();
                    
                    // Move arm to place pose
                    placePose.position.z += WorldValues::TIAGO_GRIPPER_LENGTH + LEAVE_DISTANCE;
                    TiagoMotion::moveGroupLinear(moveGroup, plan, placePose);

                    // Open the gripper.
                    TiagoMotion::moveGripper(true);

                    // Detach the object via the Gazebo plugin
                    attachObjectToGripper(false, goal->target_id);

                    // Move arm to approach pose
                    TiagoMotion::moveGroupLinear(moveGroup, plan, approachPose);

                    // Reset object orientation
                    tf2::Quaternion objectOrientation;
                    objectOrientation.setRPY(0, 0, 0);
                    tf2::convert(objectOrientation, placePose.orientation);

                    // Add placed object to collision objects
                    objectCollisionObject.primitives.push_back(objectPrimitive);
                    objectCollisionObject.primitive_poses.push_back(placePose);
                    objectCollisionObject.operation = objectCollisionObject.ADD;
                    planningSceneInterface.applyCollisionObject(objectCollisionObject);

                    collisionObjectsIds.push_back(objectCollisionObject.id);
                    // Ensure collision objects are updated before proceeding
                    ros::Duration(DELAY).sleep();

                    // Move arm to safe pose
                    TiagoMotion::moveGroupToNamedTarget(moveGroup, plan, "safe_pose");

                    result_.succeded = true;
                }

                feedback_.current_status = "Place routine has ended.";
                as_.publishFeedback(feedback_);
            }

            // Remove collision objects from scene
            planningSceneInterface.removeCollisionObjects(collisionObjectsIds);
            
            // Ensure collision objects are updated before proceeding
            ros::Duration(DELAY).sleep();

            if (result_.succeded)
                ROS_INFO("%s: Succeeded.", action_name_.c_str());
            else
                ROS_ERROR("%s: Failed.", action_name_.c_str());
            as_.setSucceeded(result_);
        }
};

/**
 * This main function initializes a ROS node, creates a PickAndPlaceServer object,
 * prints a message to the console, and enters a loop to process ROS messages.
 * 
 * @param argc The argc parameter is an integer that represents the number of command line arguments
 * passed to the program. It stands for "argument count".
 * @param argv The `argv` parameter is an array of strings that contains the command-line arguments
 * passed to the program. Each element of the array represents a separate argument.
 * 
 * @return The main function is returning an integer value of 0.
 */
int main(int argc, char **argv){
    ros::init(argc, argv, "nodeC");
    PickAndPlaceServer PickAndPlace("PickAndPlace");
    ROS_INFO("PickAndPlaceServer ready!");
    ros::spin();
    return 0;
}