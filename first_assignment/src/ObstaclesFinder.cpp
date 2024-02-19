/* 
	Group 23: Nicola Lorenzon 2087643, Daniele Moschetta 2087640
	
	This is the implementation of the ObstaclesFinder library.
*/

#include "../include/ObstaclesFinder.h"

const int UNLABELED = -1;
const float MAX_OBSTACLE_RADIUS = 0.5;
const float DISTANCE_TOLERANCE = 0.05;

/**
 * The function "findNeighbors" takes in a point index, a vector of points, and a maximum distance, and
 * returns a vector of indices of points that are within the maximum distance from the given point.
 * 
 * @param pointIndex The index of the point for which we want to find neighbors.
 * @param points A vector of pairs of floats representing the coordinates of points in a 2D space.
 * @param maxDistance The maxDistance parameter represents the maximum distance allowed between two
 * points for them to be considered neighbors.
 * 
 * @return a vector of integers, which represents the indices of the neighboring points within a given
 * maximum distance from a specified point.
 */
std::vector<int> findNeighbors(int pointIndex, const std::vector<std::pair<float, float>> &points, float maxDistance) {
    std::vector<int> neighbors;
    for (int i = 0; i < points.size(); i++)
        if (PointUtils::getDistance(points[pointIndex], points[i]) <= maxDistance)
            neighbors.push_back(i);
    return neighbors;
}

/**
 * The function findBisectorLine calculates the slope and y-intercept of the perpendicular bisector
 * line between two given points.
 * 
 * @param pointA The coordinates of the first point (x1, y1).
 * @param pointB The second point (x2, y2)
 * 
 * @return a std::pair<float, float> representing the equation of the bisector line. The first element
 * of the pair represents the slope of the line, and the second element represents the y-intercept of
 * the line.
 */
std::pair<float, float> findBisectorLine(const std::pair<float, float>& pointA, const std::pair<float, float>& pointB) {
    float slope = (pointB.second - pointA.second) / (pointB.first - pointA.first);
    float perpendicularSlope = -1 / slope;
    std::pair<float, float> pointAvg((pointA.first + pointB.first) / 2, (pointA.second + pointB.second) / 2);
    float yIntercept = pointAvg.second - perpendicularSlope * pointAvg.first;
    return std::make_pair(perpendicularSlope, yIntercept);
}

/**
 * The function findIntersectionPoint calculates the intersection point of two lines given their slope
 * and y-intercept.
 * 
 * @param line1 The line1 parameter is a pair of floats representing the equation of a line in the form
 * (m, b), where m is the slope of the line and b is the y-intercept.
 * @param line2 The line2 parameter represents a pair of float values that define a line in the form of
 * y = mx + b, where line2.first represents the slope (m) and line2.second represents the y-intercept
 * (b) of the line.
 * 
 * @return a std::pair<float, float> which represents the intersection point of two lines.
 */
std::pair<float, float> findIntersectionPoint(const std::pair<float, float>& line1, const std::pair<float, float>& line2) {
    float xIntersection = (line2.second - line1.second) / (line1.first - line2.first);
    float yIntersection = line1.first * xIntersection + line1.second;
    return std::make_pair(xIntersection, yIntersection);
}

/**
 * The function "findObstaclesCentroids" takes a vector of points and a maximum distance, and returns
 * the centroids of clusters of points that are within the maximum distance of each other.
 * 
 * @param points A vector of pairs representing the coordinates of points in a 2D space.
 * @param maxDistance The maxDistance parameter is the maximum distance allowed between two points for
 * them to be considered neighbors. If the distance between two points is greater than maxDistance,
 * they will not be considered neighbors.
 * 
 * @return The function `findObstaclesCentroids` returns a vector of pairs of floats, representing the
 * centroids of the found obstacles.
 */
std::vector<std::pair<float, float>> findObstaclesCentroids(const std::vector<std::pair<float, float>> &points, float maxDistance) {
    
    std::vector<int> labels(points.size(), UNLABELED);
    int clusterId = 0;

    for (int i = 0; i < points.size(); i++) {

        if (labels[i] != UNLABELED)
            continue;

        // Find neighbors
        std::vector<int> neighbors = findNeighbors(i, points, maxDistance);

        // Find neighbors of neighbors
        for (int j = 0; j < neighbors.size(); j++) {
            std::vector<int> curNeighbors = findNeighbors(neighbors[j], points, maxDistance);
            for (int k = 0; k < curNeighbors.size(); k++)
                if (std::count(neighbors.begin(), neighbors.end(), curNeighbors[k]) == 0)
                    neighbors.push_back(curNeighbors[k]);
        }

        // Check if any neighbor has a label assigned
        int labelFound = -1;
        for (int j = 0; j < neighbors.size(); j++)           
            if (labels[neighbors[j]] != UNLABELED) {
                labelFound = labels[neighbors[j]];
                break;
            }
        
        // If no label was found use the new cluster id
        if (labelFound == -1)
            labelFound = clusterId;

        for (int j = 0; j < neighbors.size(); j++)
            labels[neighbors[j]] = clusterId;

        // If the new cluster id has been used increment it
        if (labelFound == clusterId)
            clusterId++;
    }

    // Find the centroids
    std::vector<std::pair<float, float>> centroids;
    for (int i = 0; i < clusterId; i++) {
        int count = 0;
        std::pair<float, float> centroid(0,0);
        std::vector<std::pair<float, float>> tmpCluster;

        for (int j = 0; j < points.size(); j++) {
            if (labels[j] == i){
                tmpCluster.push_back(points[j]);
                centroid.first += points[j].first;
                centroid.second += points[j].second;
                count++;
            }
        }

        if (count == 0)
            continue;

        // Check cluster
        bool keepCluster = true;
        std::pair<float, float> pointIntersection;
        if (tmpCluster.size() < 5)
            keepCluster = false;
        else {
            // Pick the first, last and middle point
            std::pair<float, float> pointA = tmpCluster[0], pointB = tmpCluster[tmpCluster.size() - 1], pointMid = tmpCluster[tmpCluster.size() / 2];

            // Find the bisectors between (pointA, pointMid) and (pointB, pointMid)
            std::pair<float, float> bisectorA = findBisectorLine(pointA, pointMid);
            std::pair<float, float> bisectorB = findBisectorLine(pointB, pointMid);

            // Find the intersection point
            pointIntersection = findIntersectionPoint(bisectorA, bisectorB);

            float targetDist = std::max(PointUtils::getDistance(pointA, pointIntersection), PointUtils::getDistance(pointB, pointIntersection));

            // Discard the cluster if the targetDist is too far (i.e. the point don't have a circular distribution)
            if (targetDist > MAX_OBSTACLE_RADIUS)
                keepCluster = false;
            else {
                // Discard the cluster if at least one point has distance from pointIntersection outside of a certain tolerance
                for (int j = 0; j < tmpCluster.size(); j++) {
                    float curDist = PointUtils::getDistance(tmpCluster[j], pointIntersection);
                    if (!((curDist >= targetDist * (1.0 - DISTANCE_TOLERANCE)) && (curDist <= targetDist * (1.0 + DISTANCE_TOLERANCE)))) {
                        keepCluster = false;
                        break;
                    }
                }
            }
        }

        if (keepCluster) {
            centroids.push_back(pointIntersection);
        }
    }

    return centroids;
}

/**
 * The function finds the maximum minimum distance between any two points in a given vector of points.
 * 
 * @param points The "points" parameter is a vector of pairs of floats. Each pair represents the
 * coordinates of a point in a 2D space.
 * 
 * @return the maximum distance between any two points in the given vector of points.
 */
float findMaxMingetDistance(const std::vector<std::pair<float, float>> &points) {
    float maxDist = std::numeric_limits<float>::min();
    for (int i = 0; i < points.size(); i++) {
        float minDist = std::numeric_limits<float>::max();
        for (int j = 0; j < points.size(); j++) {
            if (j != i && PointUtils::getDistance(points[i], points[j]) < minDist)
                minDist = PointUtils::getDistance(points[i], points[j]); 
        }
        if (minDist > maxDist)
            maxDist = minDist;
    }
    return maxDist;
}

/**
 * The function "findObstacles" takes a vector of points, finds the maximum distance between any two
 * points, and then returns the centroids of the obstacles within that maximum distance.
 * 
 * @param points The "points" parameter is a vector of pairs of floats. Each pair represents the
 * coordinates of a point in a two-dimensional space.
 * 
 * @return a vector of pairs of floats.
 */
std::vector<std::pair<float, float>> ObstaclesFinder::findObstacles(const std::vector<std::pair<float, float>> &points) {
    float maxDistance = findMaxMingetDistance(points);
    return findObstaclesCentroids(points, maxDistance);
}