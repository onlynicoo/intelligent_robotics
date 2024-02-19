// Group 23: Nicola Lorenzon 2087643, Daniele Moschetta 2087640

#pragma once
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class PointUtils {
    public:
        static float getDistance(const std::pair<float, float> &point1, const std::pair<float, float> &point2);
        static std::vector<std::pair<float, float>> getMetricPoints(const sensor_msgs::LaserScanConstPtr &msg);
        static std::vector<std::pair<float, float>> transformPoints(const std::vector<std::pair<float, float>>& sourcePoints, std::string sourceFrame, std::string targetFrame);
        static void plotPointsRViz(const std::vector<std::pair<float, float>>& feetPos, std::string frame_id);
};