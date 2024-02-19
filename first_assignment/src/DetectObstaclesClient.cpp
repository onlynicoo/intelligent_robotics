/* 
	Group 23: Nicola Lorenzon 2087643, Daniele Moschetta 2087640
	
	This is the implementation of the client for the obstacles detection.
*/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <first_assignment/DetectObstaclesAction.h>

/**
 * The function "activeCb" logs a message indicating that the "Detect obstacles server" has started.
 */
void activeCb(){
	ROS_INFO("Detect obstacles server: started.");
}

/**
 * The function feedbackCb prints the current status of the Tiago robot.
 * 
 * @param feedback The parameter "feedback" is a constant pointer to a
 * first_assignment::DetectObstaclesFeedback message. This message contains feedback information
 * from the action server. In this case, it seems to contain a field called "current_status" which is
 * of type string.
 */
void feedbackCb(const first_assignment::DetectObstaclesFeedbackConstPtr& feedback){
	ROS_INFO("Tiago status: %s", feedback->current_status.c_str());
}

/**feedback
 * The function "doneCb" is a callback function that prints a message about the success or failure of a
 * goal and displays the positions of detected obstacles.
 * 
 * @param state The parameter "state" is of type actionlib::SimpleClientGoalState and represents the
 * current state of the goal. It provides information about whether the goal was successfully achieved
 * or not.
 * @param result The "result" parameter is of type
 * "first_assignment::DetectObstaclesResultConstPtr&", which is a pointer to a constant object
 * of type "first_assignment::DetectObstaclesResult". This object contains the result of the
 * detect obstacles action.
 */
void doneCb(const actionlib::SimpleClientGoalState& state, const first_assignment::DetectObstaclesResultConstPtr& result){
	
	// Check if the goal is completely reached or no and print a message about it.
	if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Tiago successfully detected %d obstacles from Pose_B.", (int)result->x_obstacles.size());
	else
		ROS_INFO("Tiago was NOT able to reach and complete the goal.");
	
	// Print all obstacles positions (if any).
	if (result->x_obstacles.size() != 0){
		ROS_INFO("Detected obstacles positions in the map reference frame:" );
		for (int i = 0; i < result->x_obstacles.size(); i++)
			ROS_INFO("(%lf, %lf)", (float)result->x_obstacles[i], (float)result->y_obstacles[i]);
	}
}

/**
 * This C++ code initializes a client to communicate with a server and sends a goal to detect obstacles
 * at a specified position and orientation.
 * 
 * @param argc The argc parameter is an integer that represents the number of command line arguments
 * passed to the program. It stands for "argument count".
 * @param argv The `argv` parameter is an array of strings that contains the command-line arguments
 * passed to the program. In this case, `argv` is expected to have 4 elements:
 * 
 * @return The main function is returning an integer value of 0.
 */
int main(int argc, char** argv){
	
	// Read the arguments for Pose_B: pose_x, pose_y and yaw.
	ros::init(argc, argv, "DetectObstaclesClient");
	if(argc != 4){
		ROS_INFO("Please insert: pose_x, pose_y and yaw (the first two are the coordinates of the point Pose_B and the yaw is the angle in degrees).");
		return 1;
	}
	
	// Initialize the DetectObstaclesClient.
	actionlib::SimpleActionClient<first_assignment::DetectObstaclesAction> actionDetectObstalces("detectObstacles", true);
	
	// Connect to the server.
    ROS_INFO("Waiting for DetectObstaclesServer to start.");

	// Wait for the server to start.
	actionDetectObstalces.waitForServer();

    // Send the goal to the server.
	first_assignment::DetectObstaclesGoal goal;
	goal.x_pos = atof(argv[1]);
	goal.y_pos = atof(argv[2]);
	goal.yaw = atof(argv[3]);

	// Push the goal to the move_base node.
	actionDetectObstalces.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

	// Wait the goal to finish.
	actionDetectObstalces.waitForResult();
	
	return 0;
}