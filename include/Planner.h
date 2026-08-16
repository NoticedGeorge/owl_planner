#pragma once

#include <vector>
#include <Eigen/Dense>
#include "safeField.h"
#include "Bypass.h"
#include "PolynomialTrajectory.h"

using Eigen::Vector2d;

class Planner {
public:
    Planner(double safe_r = 0.2, double detection_margin = 0.45);

    // Main interface: given a reference path and obstacle map, output a smooth feasible path
    std::vector<Vector2d> plan(
        const std::vector<Vector2d>& path,
        std::vector<std::vector<Obstacle>>& obstacles,
        const Vector2d& start,
        const Vector2d& goal,
        const Vector2d& velocity);

private:
    // =============================
    // Core modules
    // =============================
    std::vector<Vector2d> generateProgressiveAPF(
        Vector2d curr_pos,
        const std::vector<Obstacle>& group,
        const std::vector<Vector2d>& bypass_path,
        Vector2d goal,
        double max_length);

    std::pair<double, double> computeSideMetric(
        const std::vector<Obstacle>& group,
        const std::vector<std::vector<Obstacle>>& groups,
        const Vector2d& start,
        const Vector2d& goal);

    std::vector<Vector2d> buildControlPoints(
        const std::vector<Vector2d>& bypass,
        const std::vector<Vector2d>& final_path,
        const Vector2d& start,
        const Vector2d& goal);

    void spliceTrajectory(
        std::vector<Vector2d>& final_path,
        const std::vector<Vector2d>& smooth_traj);

    double pointToSegmentDistance(
        const Eigen::Vector2d& p,
        const Eigen::Vector2d& a,
        const Eigen::Vector2d& b);

    std::vector<Vector2d> getFinalSmoothPath(
        const std::vector<Vector2d>& raw_path);

private:
    double safe_r_;
    double detection_margin_;  // detection margin for early bypass (fixes close-range bypass failure)
};
