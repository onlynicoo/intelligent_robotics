/* 
	Group 23: Nicola Lorenzon 2087643, Daniele Moschetta 2087640

	nodeB is a service that handles the AprilTag object detection.
*/

#include <ros/ros.h>
#include <boost/make_shared.hpp>
#include <second_assignment/TagDetectionSrv.h>
#include "apriltag_ros/AprilTagDetectionArray.h"
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include "geometry_msgs/PoseArray.h"
#include <string>

namespace NodeBService {

	/**
	 * The function handles a detection request by subscribing to tag detection data, transforming the
	 * poses from the camera frame to the base_link frame, and returning the transformed poses and
	 * detected IDs.
	 * 
	 * @param inRequest The input request object of type `second_assignment::TagDetectionSrv::Request`. It
	 * contains the data required for the tag detection processing.
	 * @param outResponse The `outResponse` parameter is of type
	 * `second_assignment::TagDetectionSrv::Response`, which is a custom message type defined in the
	 * `second_assignment` package. It contains the response data that will be sent back to the client who
	 * made the detection request.
	 * 
	 * @return a boolean value.
	 */
	bool handleDetectionRequest(second_assignment::TagDetectionSrv::Request &inRequest, second_assignment::TagDetectionSrv::Response &outResponse) {
		ROS_INFO("Processing detection request...");

		// Subscription to tag detection data
		ros::NodeHandle nh;
		boost::shared_ptr<const apriltag_ros::AprilTagDetectionArray> tagDataPtr;
		tagDataPtr = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections", nh, ros::Duration(20));

		if (!tagDataPtr) {
			ROS_WARN("NO AprilTag data received.");
			return false;
		}
		ROS_INFO("AprilTag data received.");
		int detectedTagCount = tagDataPtr->detections.size();

		// Transformation from the camera frame to the base_link frame
		tf::TransformListener baseLinkTransformListener;
		tf::StampedTransform cameraToBaseLinkTransform;
		
		std::string camFrameID;
		if (tagDataPtr->detections.empty()) {
			camFrameID = "";
		} else {
			camFrameID = tagDataPtr->detections[0].pose.header.frame_id;
		}	
		
		if (!camFrameID.empty()) {
			baseLinkTransformListener.waitForTransform("base_footprint", camFrameID, ros::Time(0), ros::Duration(5.0));
			baseLinkTransformListener.lookupTransform("base_footprint", camFrameID, ros::Time(0), cameraToBaseLinkTransform);
			ROS_INFO("Transformation to the base_footprint frame acquired.");
		}

		outResponse.poses.header.frame_id = "base_footprint";

		for (const auto &detection : tagDataPtr->detections) {
			tf::Pose originalPose, transformedPose;
			geometry_msgs::Pose finalPose;
			int detectedID = detection.id[0];
			ROS_INFO("Processing tag ID: %d", detectedID);

			tf::poseMsgToTF(detection.pose.pose.pose, originalPose);
			transformedPose = cameraToBaseLinkTransform * originalPose;
			tf::poseTFToMsg(transformedPose, finalPose);

			outResponse.poses.poses.push_back(finalPose);
			outResponse.ids.push_back(detectedID);
		}
		ROS_INFO("Detection handling complete.");
		return true;
	}
}

/**
 * The main function initializes a ROS node, advertises a service, and spins the node to handle
 * incoming requests.
 * 
 * @param argc The argc parameter is an integer that represents the number of command line arguments
 * passed to the program. It stands for "argument count".
 * @param argv The `argv` parameter is an array of strings that contains the command-line arguments
 * passed to the program. Each element of the array represents a separate argument.
 * 
 * @return The main function is returning an integer value of 0.
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "nodeB");
	ros::NodeHandle mainNodeHandle;
	ros::ServiceServer detectionServer = mainNodeHandle.advertiseService("/TagDetection", NodeBService::handleDetectionRequest);
    ROS_INFO("Service nodeB ready!");
    ros::spin();
    return 0;
}