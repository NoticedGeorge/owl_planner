#pragma once

#include <cstdint>

#include "common.h"
#include "ObstacleProcessor.h"

// Perception node: circular detection -> grid downsampling -> clustering -> simplified modeling
// Every output frame carries an acquisition timestamp and a monotonic sequence number
class PerceptionNode
{
public:
    explicit PerceptionNode(double safe_r = 0.2);

    PerceptionFrame process(const ObstacleMap& circle_detections);

    std::uint64_t seq() const { return seq_; }

private:
    ObstacleProcessor processor_;
    std::uint64_t seq_ = 0;
};
