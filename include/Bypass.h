#pragma once

#include <vector>
#include <Eigen/Dense>
#include "safeField.h"

class DetourPlanner {
public:
    double safe_r = 0.2;

    std::vector<Eigen::Vector2d> bypassCluster(
        const Eigen::Vector2d& hit_point,
        const std::vector<Obstacle>& group,
        const Eigen::Vector2d& start,
        const Eigen::Vector2d& goal,
        const Eigen::Vector2d& safe_score,
        double step = 0.05);

private:
    // ===== Decision =====
    std::vector<Eigen::Vector2d> sampleLine(
        const Eigen::Vector2d& a, const Eigen::Vector2d& b, double step);

    std::vector<Eigen::Vector2d> extractClusterContourStep(
        const std::vector<Obstacle>& cluster,
        double step = 0.05,
        int num_points_per_circle = 16);

    Eigen::Vector2d computeScore(
        std::vector<Eigen::Vector2d>& l_path,
        std::vector<Eigen::Vector2d>& r_path,
        Eigen::Vector2d safe_score,
        Eigen::Vector2d velocity,
        Eigen::Vector2d start,
        Eigen::Vector2d goal);

    // ===== Path organization =====
    void organizePath(
        std::vector<Eigen::Vector2d>& path,
        const Eigen::Vector2d& start,
        const Eigen::Vector2d& goal);

    int findClosestIndex(
        const std::vector<Eigen::Vector2d>& contour,
        const Eigen::Vector2d& p);

    bool isBackToLine(
        const Eigen::Vector2d& pt,
        const Eigen::Vector2d& start,
        const Eigen::Vector2d& goal,
        double threshold = 0.08);
};
