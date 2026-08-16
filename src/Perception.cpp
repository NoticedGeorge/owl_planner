#include "Perception.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace {

// grid downsampling: keep one obstacle per 0.3 m cell (limits cluster size), keep original radius
std::vector<Obstacle> downsampleObstacles(
    const std::vector<Obstacle>& obstacles,
    double grid_size = 0.3)
{
    std::unordered_map<std::int64_t, Obstacle> grid_map;

    for (const auto& obs : obstacles)
    {
        std::int64_t ix = static_cast<std::int64_t>(std::floor(obs.center.x() / grid_size));
        std::int64_t iy = static_cast<std::int64_t>(std::floor(obs.center.y() / grid_size));
        std::int64_t key = (ix << 32) | (iy & 0xFFFFFFFF);

        if (grid_map.find(key) == grid_map.end())
            grid_map[key] = obs;
    }

    std::vector<Obstacle> result;
    result.reserve(grid_map.size());

    for (auto& kv : grid_map)
        result.push_back(kv.second);
    return result;
}

} // namespace

PerceptionNode::PerceptionNode(double safe_r)
    : processor_(safe_r) {}

PerceptionFrame PerceptionNode::process(const ObstacleMap& circle_detections)
{
    PerceptionFrame frame;
    frame.stamp = Clock::now();
    frame.seq = ++seq_;

    // 1. flatten into raw detections
    std::vector<Obstacle> raw;
    for (const auto& group : circle_detections)
        for (const auto& o : group)
            raw.push_back(o);

    if (raw.empty())
    {
        frame.data = circle_detections;
        return frame;
    }

    // 2. grid downsampling
    auto sampled = downsampleObstacles(raw, 0.3);

    // 3. constant-velocity prediction + inflation: avoid where obstacles will be
    const double prediction_s = 1.2;
    for (auto& o : sampled)
    {
        o.center += o.velocity * prediction_s;
        o.radius += 0.25;
    }

    // 4. clustering
    auto clusters = processor_.clusterObstacles(sampled, 0.4);

    // 5. each cluster -> simplified modeling (multi-circle approximation)
    ObstacleMap out;
    out.reserve(clusters.size());
    for (const auto& cluster : clusters)
    {
        if (cluster.empty()) continue;
        out.push_back(processor_.getSimplifiedObstaclesAuto(cluster));
    }

    frame.data = std::move(out);
    return frame;
}
