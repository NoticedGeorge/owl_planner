#include "../include/ObstacleProcessor.h"
#include <numeric>
#include <algorithm>
#include <cmath>

using namespace std;

// =============================
// DSU
// =============================
ObstacleProcessor::DSU::DSU(int n) {
    parent.resize(n);
    iota(parent.begin(), parent.end(), 0);
}

int ObstacleProcessor::DSU::find(int i) {
    if (parent[i] == i) return i;
    return parent[i] = find(parent[i]);
}

void ObstacleProcessor::DSU::unite(int i, int j) {
    int ri = find(i);
    int rj = find(j);
    if (ri != rj) parent[ri] = rj;
}

// =============================
// constructor
// =============================
ObstacleProcessor::ObstacleProcessor(double safe_r)
    : safe_r_(safe_r) {}

// =============================
// main entry
// =============================
vector<Obstacle> ObstacleProcessor::process(
    const vector<Obstacle>& raw_obs,
    double cluster_margin)
{
    auto clusters = clusterObstacles(raw_obs, cluster_margin);
    return simplifyClusters(clusters);
}

// =============================
// clustering
// =============================
vector<vector<Obstacle>> ObstacleProcessor::clusterObstacles(
    const vector<Obstacle>& obstacles,
    double margin)
{
    int n = obstacles.size();
    DSU dsu(n);

    for (int i = 0; i < n; ++i) {
        if (!obstacles[i].active) continue;

        for (int j = i + 1; j < n; ++j) {
            if (!obstacles[j].active) continue;

            double r = obstacles[i].radius + obstacles[j].radius + margin;

            if ((obstacles[i].center - obstacles[j].center).squaredNorm() < r * r) {
                dsu.unite(i, j);
            }
        }
    }

    unordered_map<int, vector<Obstacle>> clusters;

    for (int i = 0; i < n; ++i) {
        if (obstacles[i].active)
            clusters[dsu.find(i)].push_back(obstacles[i]);
    }

    vector<vector<Obstacle>> result;
    for (auto& p : clusters)
        result.push_back(move(p.second));

    return result;
}

// =============================
// simplified modeling (core)
// =============================
vector<Obstacle> ObstacleProcessor::simplifyClusters(
    const vector<vector<Obstacle>>& clusters)
{
    vector<Obstacle> final_obs;

    for (const auto& cluster : clusters) {
        if (cluster.empty()) continue;

        // ===== 1. centroid =====
        Vector2d centroid = Vector2d::Zero();
        for (const auto& o : cluster)
            centroid += o.center;
        centroid /= (double)cluster.size();

        // ===== 2. dynamic fusion =====
        Vector2d avg_vel = Vector2d::Zero();
        double max_gain = 0.0;

        for (const auto& o : cluster) {
            avg_vel += o.velocity;
            max_gain = max(max_gain, o.velocity_gain);
        }
        avg_vel /= (double)cluster.size();

        // ===== 3. PCA =====
        Matrix2d cov = Matrix2d::Zero();
        for (const auto& o : cluster) {
            Vector2d d = o.center - centroid;
            cov += d * d.transpose();
        }

        Eigen::SelfAdjointEigenSolver<Matrix2d> solver(cov);
        Vector2d main_axis = solver.eigenvectors().col(1).normalized();
        Vector2d ortho_axis(-main_axis.y(), main_axis.x());

        // ===== 4. projection =====
        double min_p = 1e9, max_p = -1e9;
        double min_o = 1e9, max_o = -1e9;

        for (const auto& o : cluster) {
            double p = (o.center - centroid).dot(main_axis);
            double q = (o.center - centroid).dot(ortho_axis);

            min_p = min(min_p, p - o.radius);
            max_p = max(max_p, p + o.radius);

            min_o = min(min_o, q - o.radius);
            max_o = max(max_o, q + o.radius);
        }

        double auto_r = (max_o - min_o) / 2.0;
        auto_r = max(auto_r, 0.3);

        double length = max_p - min_p;

        // ===== 5. modeling =====
        if (length <= 2.0 * auto_r) {
            final_obs.push_back({
                centroid,
                auto_r,
                true,
                avg_vel,
                max_gain
            });
        } else {
            double step = auto_r * 0.8;
            double curr = min_p + auto_r;

            while (curr < max_p - auto_r) {
                final_obs.push_back({
                    centroid + curr * main_axis,
                    auto_r,
                    true,
                    avg_vel,
                    max_gain
                });
                curr += step;
            }

            final_obs.push_back({
                centroid + (max_p - auto_r) * main_axis,
                auto_r,
                true,
                avg_vel,
                max_gain
            });
        }
    }

    return final_obs;
}

std::vector<Obstacle> ObstacleProcessor::getSimplifiedObstaclesAuto(
    const std::vector<Obstacle>& raw_obs,
    double cluster_margin)
{
    if (raw_obs.empty()) return {};

    // ===== step 1: DSU clustering =====
    int n = raw_obs.size();
    DSU dsu(n);

    for (int i = 0; i < n; ++i) {
        if (!raw_obs[i].active) continue;

        for (int j = i + 1; j < n; ++j) {
            if (!raw_obs[j].active) continue;

            double r_sum = raw_obs[i].radius + raw_obs[j].radius + cluster_margin;

            if ((raw_obs[i].center - raw_obs[j].center).squaredNorm() < r_sum * r_sum) {
                dsu.unite(i, j);
            }
        }
    }

    std::unordered_map<int, std::vector<Obstacle>> clusters;

    for (int i = 0; i < n; ++i) {
        if (raw_obs[i].active) {
            clusters[dsu.find(i)].push_back(raw_obs[i]);
        }
    }

    // ===== step 2: automated modeling =====
    std::vector<Obstacle> final_obs;

    for (auto& pair : clusters) {
        const auto& cluster = pair.second;
        if (cluster.empty()) continue;

        // ===== 1. geometric center =====
        Vector2d centroid = Vector2d::Zero();
        for (const auto& o : cluster)
            centroid += o.center;
        centroid /= (double)cluster.size();

        // ===== 2. dynamic info fusion (new) =====
        Vector2d avg_velocity = Vector2d::Zero();
        double max_gain = 0.0;

        for (const auto& o : cluster) {
            avg_velocity += o.velocity;
            max_gain = std::max(max_gain, o.velocity_gain);
        }
        avg_velocity /= (double)cluster.size();

        // ===== 3. PCA principal direction =====
        Matrix2d cov = Matrix2d::Zero();
        for (const auto& o : cluster) {
            Vector2d diff = o.center - centroid;
            cov += diff * diff.transpose();
        }

        Eigen::SelfAdjointEigenSolver<Matrix2d> solver(cov);
        Vector2d main_axis = solver.eigenvectors().col(1).normalized();
        Vector2d ortho_axis(-main_axis.y(), main_axis.x());

        // ===== 4. projection range =====
        double min_p = 1e9, max_p = -1e9;
        double min_o = 1e9, max_o = -1e9;

        for (const auto& o : cluster) {
            double p = (o.center - centroid).dot(main_axis);
            double o_val = (o.center - centroid).dot(ortho_axis);

            min_p = std::min(min_p, p - o.radius);
            max_p = std::max(max_p, p + o.radius);

            min_o = std::min(min_o, o_val - o.radius);
            max_o = std::max(max_o, o_val + o.radius);
        }

        // ===== 5. automatic radius =====
        double auto_r = (max_o - min_o) / 2.0;
        auto_r = std::max(auto_r, 0.3);  // minimum protection

        double length = max_p - min_p;

        // ===== 6. modeling =====
        if (length <= 2.0 * auto_r) {
            // single circle
            final_obs.push_back({
                centroid,
                auto_r,
                true,
                avg_velocity,
                max_gain
            });
        } else {
            // multiple circles (elongated)
            double step = auto_r * 0.8;
            double curr_p = min_p + auto_r;

            while (curr_p < max_p - auto_r) {
                final_obs.push_back({
                    centroid + curr_p * main_axis,
                    auto_r,
                    true,
                    avg_velocity,
                    max_gain
                });
                curr_p += step;
            }

            // pad the tail with points
            final_obs.push_back({
                centroid + (max_p - auto_r) * main_axis,
                auto_r,
                true,
                avg_velocity,
                max_gain
            });
        }
    }

    return final_obs;
}
//
// Created by georg on 2026/4/18.
//
