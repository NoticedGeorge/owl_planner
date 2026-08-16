#pragma once

#include <vector>
#include <unordered_map>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "safeField.h"

using Eigen::Vector2d;
using Eigen::Matrix2d;

class ObstacleProcessor {
public:
    explicit ObstacleProcessor(double safe_r = 0.2);

    // Main interface: raw obstacles -> cluster and simplify
    std::vector<Obstacle> process(
        const std::vector<Obstacle>& raw_obs,
        double cluster_margin = 0.3);

    // =============================
    // Internal structures
    // =============================
    struct DSU {
        std::vector<int> parent;
        DSU(int n);
        int find(int i);
        void unite(int i, int j);
    };

    // =============================
    // Core pipeline
    // =============================
    std::vector<std::vector<Obstacle>> clusterObstacles(
        const std::vector<Obstacle>& obstacles,
        double margin);

    std::vector<Obstacle> simplifyClusters(
        const std::vector<std::vector<Obstacle>>& clusters);

    std::vector<Obstacle> getSimplifiedObstaclesAuto(
        const std::vector<Obstacle>& raw_obs,
        double cluster_margin = 0.3);

private:
    double safe_r_;
};
