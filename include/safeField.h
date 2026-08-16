#pragma once

#include <Eigen/Dense>
#include <vector>

#include "common.h"

using Eigen::Vector2d;

// Elliptical safe corridor
struct epllipseSafeField {
    double a;
    double b;
    Vector2d center;
    double theta;
};

/**
 * @brief generate and optimize a sequence of elliptical safe corridors
 */
std::vector<epllipseSafeField> getCorridorAndOptimization(
    const std::vector<Vector2d>& bypassPath,
    const std::vector<std::vector<Obstacle>>& obstacles);
