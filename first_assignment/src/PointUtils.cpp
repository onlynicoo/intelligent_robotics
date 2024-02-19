/* 
	Group 23: Nicola Lorenzon 2087643, Daniele Moschetta 2087640
	
	This is the implementation of the PointUtils library.
*/

#include "../include/PointUtils.h"

/**
 * The function calculates the Euclidean distance between two points in a 2D space.
 * 
 * @param point1 A pair of floats representing the coordinates of the first point.
 * @param point2 The above code defines a function named `getDistance` in the `PointUtils` class. This
 * function takes two parameters, `point1` and `point2`, both of which are pairs of floats representing
 * coordinates in a 2D space.
 * 
 * @return the distance between two points as a float value.
 */
float PointUtils::getDistance(const std::pair<float, float> &point1, const std::pair<float, float> &point2) {
    return sqrt(pow(point1.first - point2.first, 2) + pow(point1.second - point2.second, 2));
}

/**
 * The function "getMetricPoints" takes in a laser scan message and returns a vector of metric points
 * representing the valid ranges and angles from the scan.
 * 
 * @param msg The parameter `msg` is of type `sensor_msgs::LaserScanConstPtr`, which is a pointer to a
 * constant `sensor_msgs::LaserScan` message. This message contains information about a laser scan,
 * including the measured ranges and angles.
 * 
 * @return a vector of pairs of floats, representing the metric points computed from the laser scan
 * data.
 */
std::vector<std::pair<float, float>> PointUtils::getMetricPoints(const sensor_msgs::LaserScanConstPtr &msg) {
    std::vector<std::pair<float, float>> metricPoints;
    // Scan all the measured ranges
    for (int i = 0; i < msg->ranges.size(); i++) {
        float range = msg->ranges[i];
        // Check if the range is valid
        if (range >= msg->range_min && range <= msg->range_max) {
            // Compute the angle
            float angle = msg->angle_min + (float)i * msg->angle_increment;
            // Convert and add the metric point
            std::pair<float, float> point(range * std::cos(angle), range * std::sin(angle));
            metricPoints.push_back(point);
            // ROS_INFO("angle %f, range %f -> (%f, %f)", angle, range, point.first, point.second);
        }
    }
    return metricPoints;
}

/**
 * The function `transformPoints` takes a vector of points, a source frame, and a target frame, and
 * transforms each point from the source frame to the target frame using the tf2 library.
 * 
 * @param points A vector of pairs of floats representing the x and y coordinates of points in the
 * source frame.
 * @param sourceFrame The sourceFrame parameter is the frame of reference in which the input points are
 * defined. It represents the coordinate system in which the points are given.
 * @param targetFrame The targetFrame parameter is the frame to which the points will be transformed.
 * It represents the coordinate frame in which the transformed points will be expressed.
 * 
 * @return a vector of pairs of floats, which represents the transformed points.
 */
std::vector<std::pair<float, float>> PointUtils::transformPoints(const std::vector<std::pair<float, float>>& points, std::string sourceFrame, std::string targetFrame) {
    
    std::vector<std::pair<float, float>> transformedPoints;

    try {
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        tfBuffer.setUsingDedicatedThread(true);
        
        for (int i = 0; i < points.size(); i++) {
            geometry_msgs::PointStamped pointStamped;
            pointStamped.header.frame_id = sourceFrame;
            pointStamped.point.x = points[i].first;
            pointStamped.point.y = points[i].second;
            pointStamped.point.z = 0.0;
            geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform(targetFrame, sourceFrame, ros::Time(0), ros::Duration(1.0));

            // Transform the point
            tf2::doTransform(pointStamped, pointStamped, transformStamped);
            transformedPoints.emplace_back(pointStamped.point.x, pointStamped.point.y);
        }
    } catch (const std::exception& ex) {
        transformedPoints = points;
        ROS_ERROR("Exception: %s", ex.what());
        ROS_ERROR("Points could not be converted from %s frame to %s frame", sourceFrame.c_str(), targetFrame.c_str());
    }

    return transformedPoints;
}

/**
 * The function plots green points for feet positions and blue points for people positions in RViz.
 * 
 * @param feetPos A vector of pairs of floats representing the x and y coordinates of the feet
 * positions.
 */
void PointUtils::plotPointsRViz(const std::vector<std::pair<float, float>>& feetPos, std::string frame_id) {

    // Green points for feet positions
    visualization_msgs::Marker feetPoints;
    feetPoints.header.frame_id = frame_id;
    feetPoints.action = visualization_msgs::Marker::ADD;
    feetPoints.header.stamp = ros::Time::now();
    feetPoints.pose.orientation.w = 1.0;
    feetPoints.id = 0;
    feetPoints.type = visualization_msgs::Marker::POINTS;
    feetPoints.scale.x = 0.05;
    feetPoints.scale.y = 0.05;
    feetPoints.color.g = 1.0;
    feetPoints.color.a = 1.0;

    // Blue points for people positions
    visualization_msgs::Marker peoplePoints = feetPoints;
    peoplePoints.id = 1;
    peoplePoints.color.g = 0.0;
    peoplePoints.color.b = 1.0;

    for (int i = 0; i < feetPos.size(); i++) {        
        geometry_msgs::Point p;
        p.x = feetPos[i].first;
        p.y = feetPos[i].second;
        p.z = 0;
        feetPoints.points.push_back(p);
    }

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1000); 

    // Plot to RViz
    ros::Rate r(10); 
    for (int i = 0; i < 5; i++) {
        pub.publish(feetPoints);
        r.sleep();
    }
}