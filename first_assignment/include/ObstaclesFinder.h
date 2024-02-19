// Group 23: Nicola Lorenzon 2087643, Daniele Moschetta 2087640

#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <random>
#include "PointUtils.h"

class ObstaclesFinder {
    private:
        static const int UNLABELED;
        static const float MAX_OBSTACLE_RADIUS;
        static const float DISTANCE_TOLERANCE;
    public:
        static std::vector<std::pair<float, float>> findObstacles(const std::vector<std::pair<float, float>> &points);
};