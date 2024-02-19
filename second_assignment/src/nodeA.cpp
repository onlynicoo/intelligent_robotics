/* 
	Group 23: Nicola Lorenzon 2087643, Daniele Moschetta 2087640

    nodeA sends requests to other nodes in order to make them perform the pick and place tasks.
*/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <second_assignment/PickAndPlaceAction.h>
#include <second_assignment/TagDetectionSrv.h>
#include <tiago_iaslab_simulation/Objs.h>
#include "TiagoMotion.h"
#include "WorldValues.h"

/**
 * The function activeCbPnP prints a message indicating that the PickAndPlace process has started.
 */
void activeCbPnP(){
	ROS_INFO("PickAndPlace: started.");
}

/**
 * The function feedbackCbPnP prints the current status of the Tiago robot.
 * 
 * @param feedback The parameter "feedback" is of type
 * "second_assignment::PickAndPlaceFeedbackConstPtr&", which is a constant pointer to an object of type
 * "second_assignment::PickAndPlaceFeedback".
 */
void feedbackCbPnP(const second_assignment::PickAndPlaceFeedbackConstPtr& feedback){
    ROS_INFO("Tiago status: %s", feedback->current_status.c_str());
}

/**
 * The function doneCbPnP is a callback function that prints a message indicating whether the
 * PickAndPlace tasks were successfully performed or not by the Tiago robot.
 * 
 * @param state The state parameter is of type actionlib::SimpleClientGoalState and represents the
 * current state of the goal. It provides information about whether the goal was successfully achieved
 * or not.
 * @param result The "result" parameter is a pointer to the result of the PickAndPlace action. It is of
 * type "second_assignment::PickAndPlaceResultConstPtr", which is a constant pointer to a
 * PickAndPlaceResult object defined in the "second_assignment" namespace.
 */
void doneCbPnP(const actionlib::SimpleClientGoalState& state, const second_assignment::PickAndPlaceResultConstPtr& result){

    if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Tiago performed PickAndPlace tasks.");
    }
    else{
        ROS_INFO("Tiago HAS NOT performed PickAndPlace tasks.");
    }
}

/**
 * Main function that performs a pick and place routine for three objects based on their IDs.
 * 
 * @param argc The argc parameter is an integer that represents the number of command line arguments
 * passed to the program. It stands for "argument count".
 * @param argv The `argv` parameter is an array of strings that contains the command-line arguments
 * passed to the program. Each element of the array represents a separate argument.
 * 
 * @return The main function is returning an integer value of 0.
 */
int main(int argc, char** argv){

    ros::init(argc, argv, "nodeA");
    ros::NodeHandle n;

    std::vector<int> objs;

    if(argc != 4) {
        // If no arguments are given, request ID order to human_objects_srv
        ros::ServiceClient humanClient = n.serviceClient<tiago_iaslab_simulation::Objs>("/human_objects_srv");
        ros::service::waitForService("/human_objects_srv");
        
        tiago_iaslab_simulation::Objs objsSrv; 
        objsSrv.request.ready = true;
        objsSrv.request.all_objs = true; // to get all IDs at once

        if(humanClient.call(objsSrv)) {
            objs = objsSrv.response.ids;
            ROS_INFO("The order is: %d, %d, %d", (int)objs[0], (int)objs[1], (int)objs[2]);
        }
        else {
            ROS_ERROR("An error occured while retrieving IDs from human_objects_srv.");
            return 1;
        }
    } else {
        // Otherwise, use the given order
        objs.push_back(std::stoi(argv[1]));
        objs.push_back(std::stoi(argv[2]));
        objs.push_back(std::stoi(argv[3]));
    }
    
    // Pose for looking the table
    geometry_msgs::PointStamped lookToTable;
    lookToTable.header.frame_id = "map";
    lookToTable.point.x = WorldValues::PICK_TABLE_X;
    lookToTable.point.y = WorldValues::PICK_TABLE_Y;
    lookToTable.point.z = WorldValues::PICK_TABLE_HEIGHT / 2;

    for (int i = 0; i < 3; i ++) {
        // Use the correct waypoints depending on the object to pick
        std::vector<std::vector<float>> waypoints;
        if (objs[i] == 1) {
            waypoints.push_back(WorldValues::PICK_WAYPOINT_POSE_1);
            TiagoMotion::moveToPose(waypoints, WorldValues::PICK_TABLE_BLUE_POSE);
        
        } else if (objs[i] == 2 ) {
            waypoints.push_back(WorldValues::PICK_WAYPOINT_POSE_1);
            waypoints.push_back(WorldValues::PICK_WAYPOINT_POSE_2);
            TiagoMotion::moveToPose(waypoints, WorldValues::PICK_TABLE_GREEN_POSE);

        } else if (objs[i] == 3) {
            waypoints.push_back(WorldValues::PICK_WAYPOINT_POSE_1);
            TiagoMotion::moveToPose(waypoints, WorldValues::PICK_TABLE_RED_POSE);
        }

        // Move torso and head to look towards the pick table
        TiagoMotion::moveTorso(true);

        TiagoMotion::moveHead(true, lookToTable);

        // Detect objects on the table
        ros::ServiceClient clientB = n.serviceClient<second_assignment::TagDetectionSrv>("TagDetection");
        geometry_msgs::PoseArray objects_pose;
        std::vector<int> objects_id;

        second_assignment::TagDetectionSrv srv;
        srv.request.detect = true;
        if(clientB.call(srv)) {
            objects_pose = srv.response.poses;
            objects_id = srv.response.ids;
            ROS_INFO("Detection done: print detected object poses.");
            for (int i = 0; i < srv.response.ids.size(); i++) {
                ROS_INFO("ID: %d, x_pos: %f, y_pos: %f, z_pos: %f, x_orient: %f, y_orient: %f, z_orient: %f, w_orient: %f", 
                    srv.response.ids[i], 
                    objects_pose.poses[i].position.x,
                    objects_pose.poses[i].position.y, 
                    objects_pose.poses[i].position.z,
                    objects_pose.poses[i].orientation.x,
                    objects_pose.poses[i].orientation.y,
                    objects_pose.poses[i].orientation.z, 
                    objects_pose.poses[i].orientation.w);
            }
	    }
        
        // Pick routine
        actionlib::SimpleActionClient<second_assignment::PickAndPlaceAction> actionPickAndPlace("PickAndPlace", true);        
        ROS_INFO("Waiting for PickAndPlace to start.");
        actionPickAndPlace.waitForServer();
        second_assignment::PickAndPlaceGoal goal;
        goal.pick = true;
        goal.target_id = objs[i];
        goal.ids = objects_id;
        goal.poses = objects_pose;
        actionPickAndPlace.sendGoal(goal, &doneCbPnP, &activeCbPnP, &feedbackCbPnP);
        actionPickAndPlace.waitForResult();

        // Move torso and head to look straight
        TiagoMotion::moveHead(false);
        TiagoMotion::moveTorso(false);
        
        // Place routine
        geometry_msgs::Pose placePose;
        std::vector<std::vector<float>> waypoints_1;
        if (objs[i] == 1) {
            waypoints_1.push_back(WorldValues::PLACE_WAYPOINT_POSE_1);
            TiagoMotion::moveToPose(waypoints_1, WorldValues::PLACE_TABLE_BLUE_POSE);
            placePose.position.x = WorldValues::PLACE_TABLE_BLUE_X;
            placePose.position.y = WorldValues::PLACE_TABLE_BLUE_Y;

        } else if (objs[i] == 2 ) {
            waypoints_1.push_back(WorldValues::PLACE_WAYPOINT_POSE_2);
            waypoints_1.push_back(WorldValues::PLACE_WAYPOINT_POSE_1);
            TiagoMotion::moveToPose(waypoints_1, WorldValues::PLACE_TABLE_GREEN_POSE);
            placePose.position.x = WorldValues::PLACE_TABLE_GREEN_X;
            placePose.position.y = WorldValues::PLACE_TABLE_GREEN_Y;

        } else if (objs[i] == 3) {
            waypoints_1.push_back(WorldValues::PLACE_WAYPOINT_POSE_1);
            TiagoMotion::moveToPose(waypoints_1, WorldValues::PLACE_TABLE_RED_POSE);
            placePose.position.x = WorldValues::PLACE_TABLE_RED_X;
            placePose.position.y = WorldValues::PLACE_TABLE_RED_Y;
        }

        // Perform correct place
        ROS_INFO("Waiting for PickAndPlace to start.");
        actionPickAndPlace.waitForServer();
        goal.pick = false;
        goal.target_id = objs[i];
        goal.place_pose = placePose;
        actionPickAndPlace.sendGoal(goal, &doneCbPnP, &activeCbPnP, &feedbackCbPnP);
        actionPickAndPlace.waitForResult();

        // Move torso to default position
        TiagoMotion::moveTorso(false);
    }

    return 0;
}