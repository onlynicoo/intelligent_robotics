/*
	Group 23: Nicola Lorenzon 2087643, Daniele Moschetta 2087640

    WorldValues provides to the nodes shared access to all the known values about the
    simulation world, such as: object sizes, pick positions and waypoints.
*/

#include "WorldValues.h"

// Tiago
float WorldValues::TIAGO_X = 6.494045;
float WorldValues::TIAGO_Y = -1.330839;
float WorldValues::TIAGO_TABLE_DISTANCE = 0.60;
float WorldValues::TIAGO_GRIPPER_LENGTH = 0.24;

// Pick table
float WorldValues::PICK_TABLE_BASE_X = 0.92;
float WorldValues::PICK_TABLE_BASE_Y = 0.92;
float WorldValues::PICK_TABLE_HEIGHT = 0.75;
float WorldValues::PICK_TABLE_X = 1.245143 + WorldValues::TIAGO_X;
float WorldValues::PICK_TABLE_Y = -1.613171 + WorldValues::TIAGO_Y;

std::vector<float> WorldValues::PICK_TABLE_BLUE_POSE {1.518865f + WorldValues::TIAGO_X, -1.281727f + WorldValues::TIAGO_Y + WorldValues::TIAGO_TABLE_DISTANCE, 270};
std::vector<float> WorldValues::PICK_TABLE_GREEN_POSE {1.186585f + WorldValues::TIAGO_X, -1.967165f + WorldValues::TIAGO_Y - WorldValues::TIAGO_TABLE_DISTANCE, 90};
std::vector<float> WorldValues::PICK_TABLE_RED_POSE {0.939529f + WorldValues::TIAGO_X, -1.284892f + WorldValues::TIAGO_Y + WorldValues::TIAGO_TABLE_DISTANCE, 270};

// Place table
float WorldValues::PLACE_TABLE_HEIGHT = 0.69;
float WorldValues::PLACE_TABLE_RADIUS = 0.21;

float WorldValues::PLACE_TABLE_BLUE_X = 6.007146 + WorldValues::TIAGO_X;
float WorldValues::PLACE_TABLE_BLUE_Y = 1.015966 + WorldValues::TIAGO_Y;
float WorldValues::PLACE_TABLE_GREEN_X = 5.007404 + WorldValues::TIAGO_X;
float WorldValues::PLACE_TABLE_GREEN_Y = 1.015966 + WorldValues::TIAGO_Y;
float WorldValues::PLACE_TABLE_RED_X = 4.007396 + WorldValues::TIAGO_X;
float WorldValues::PLACE_TABLE_RED_Y = 1.015966 + WorldValues::TIAGO_Y;

float WorldValues::WALL_X = 3.0 + WorldValues::TIAGO_X;

std::vector<float> WorldValues::PLACE_TABLE_BLUE_POSE {WorldValues::PLACE_TABLE_BLUE_X, WorldValues::PLACE_TABLE_BLUE_Y + WorldValues::PLACE_TABLE_RADIUS + WorldValues::TIAGO_TABLE_DISTANCE, 270};
std::vector<float> WorldValues::PLACE_TABLE_GREEN_POSE {WorldValues::PLACE_TABLE_GREEN_X, WorldValues::PLACE_TABLE_GREEN_Y + WorldValues::PLACE_TABLE_RADIUS + WorldValues::TIAGO_TABLE_DISTANCE, 270};
std::vector<float> WorldValues::PLACE_TABLE_RED_POSE {WorldValues::PLACE_TABLE_RED_X, WorldValues::PLACE_TABLE_RED_Y + WorldValues::PLACE_TABLE_RADIUS + WorldValues::TIAGO_TABLE_DISTANCE, 270};

// Objects
float WorldValues::BLUE_HEXAGON_RADIUS = 0.025;
float WorldValues::BLUE_HEXAGON_HEIGHT = 0.10;
float WorldValues::GREEN_TRIANGLE_BASE_X = 0.05;
float WorldValues::GREEN_TRIANGLE_BASE_Y = 0.065;
float WorldValues::GREEN_TRIANGLE_HEIGHT = 0.02;
float WorldValues::RED_CUBE_SIZE = 0.05;
float WorldValues::GOLD_OBS_RADIUS = 0.05;
float WorldValues::GOLD_OBS_HEIGHT = 0.20;

// Move waypoints
std::vector<float> WorldValues::PICK_WAYPOINT_POSE_1 {8.50, 0.00, -90};
std::vector<float> WorldValues::PICK_WAYPOINT_POSE_2 {2.376832f + WorldValues::TIAGO_X, -2.952710f + WorldValues::TIAGO_Y + WorldValues::TIAGO_TABLE_DISTANCE, 180};

std::vector<float> WorldValues::PLACE_WAYPOINT_POSE_1 {8.5, 1.956849f + WorldValues::TIAGO_Y, 0};
std::vector<float> WorldValues::PLACE_WAYPOINT_POSE_2 {2.376832f + WorldValues::TIAGO_X, -2.952710f + WorldValues::TIAGO_Y + WorldValues::TIAGO_TABLE_DISTANCE, 90};

std::vector<float> WorldValues::PLACE_DETECT_TABLES_POSE {4.998056f + WorldValues::TIAGO_X, 2.952710f + WorldValues::TIAGO_Y, 270};