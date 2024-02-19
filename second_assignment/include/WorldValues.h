/*
	Group 23: Nicola Lorenzon 2087643, Daniele Moschetta 2087640

    WorldValues provides to the nodes shared access to all the known values about the
    simulation world, such as: object sizes, pick positions and waypoints.
*/


#pragma once
#include <vector>

class WorldValues {
    public:

        // Tiago position
        static float TIAGO_X;
        static float TIAGO_Y;
        static float TIAGO_TABLE_DISTANCE;
        static float TIAGO_GRIPPER_LENGTH;

        // Pick table
        static float PICK_TABLE_BASE_X;
        static float PICK_TABLE_BASE_Y;
        static float PICK_TABLE_HEIGHT;
        static float PICK_TABLE_X;
        static float PICK_TABLE_Y;

        static std::vector<float> PICK_TABLE_BLUE_POSE;
        static std::vector<float> PICK_TABLE_GREEN_POSE;
        static std::vector<float> PICK_TABLE_RED_POSE;

        // Pose table
        static float PLACE_TABLE_HEIGHT;
        static float PLACE_TABLE_RADIUS;

        static float PLACE_TABLE_BLUE_X;
        static float PLACE_TABLE_BLUE_Y;
        static float PLACE_TABLE_GREEN_X;
        static float PLACE_TABLE_GREEN_Y;
        static float PLACE_TABLE_RED_X;
        static float PLACE_TABLE_RED_Y;

        static float WALL_X;

        static std::vector<float> PLACE_TABLE_BLUE_POSE;
        static std::vector<float> PLACE_TABLE_GREEN_POSE;
        static std::vector<float> PLACE_TABLE_RED_POSE;

        // Objects
        static float BLUE_HEXAGON_RADIUS;
        static float BLUE_HEXAGON_HEIGHT;
        static float GREEN_TRIANGLE_BASE_X;
        static float GREEN_TRIANGLE_BASE_Y;
        static float GREEN_TRIANGLE_HEIGHT;
        static float RED_CUBE_SIZE;
        static float GOLD_OBS_RADIUS;
        static float GOLD_OBS_HEIGHT;

        // Move waypoints
        static std::vector<float> PICK_WAYPOINT_POSE_1;
        static std::vector<float> PICK_WAYPOINT_POSE_2;

        static std::vector<float> PLACE_WAYPOINT_POSE_1;
        static std::vector<float> PLACE_WAYPOINT_POSE_2;

        static std::vector<float> PLACE_DETECT_TABLES_POSE;
};