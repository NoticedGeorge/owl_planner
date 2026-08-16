#pragma once

#include <Eigen/Dense>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------------------------------------------------------------------------
// Clock and timestamps (perception / planning / execution share one clock)
// ---------------------------------------------------------------------------
using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

inline double msBetween(const TimePoint& a, const TimePoint& b)
{
    return std::chrono::duration<double, std::milli>(b - a).count();
}

inline double msSince(const TimePoint& t)
{
    return msBetween(t, Clock::now());
}

// ---------------------------------------------------------------------------
// Obstacle (2D circular approximation)
// ---------------------------------------------------------------------------
struct Obstacle
{
    Eigen::Vector2d center;
    double radius;
    bool active = true;
    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
    double velocity_gain = 0.0;
};

using ObstacleGroup = std::vector<Obstacle>;
using ObstacleMap = std::vector<ObstacleGroup>;

// ---------------------------------------------------------------------------
// Timestamped data frame
// ---------------------------------------------------------------------------
template <typename T>
struct TimeStamped
{
    T data;
    TimePoint stamp;            // when the data was produced (acquired / completed)
    std::uint64_t seq = 0;      // monotonic sequence number for deduplication
};

using PerceptionFrame = TimeStamped<ObstacleMap>;

// Robot state
struct RobotState
{
    Eigen::Vector2d pos;
    Eigen::Vector2d vel;
    double speed = 0.0;
};

// Sliding-window plan request (submitted by the executor to the pipeline)
struct PlanRequest
{
    std::vector<Eigen::Vector2d> ref_path;  // reference path within the window
    Eigen::Vector2d start;                  // planning start (robot position)
    Eigen::Vector2d goal;                   // window end (anchor or global goal)
    int anchor_idx = -1;                    // anchor index in the remaining path
    std::string reason;                     // trigger reason (logging / debugging)
};

// Plan result frame: carries the timestamps and timing of the perception frame it was based on
struct PlanFrame
{
    TimeStamped<std::vector<Eigen::Vector2d>> frame;
    int anchor_idx = -1;
    std::uint64_t perception_seq = 0;
    TimePoint perception_stamp;
    double plan_ms = 0.0;           // planning computation time
    double perception_age_ms = 0.0; // perception freshness at plan start
    std::string reason;
};

// Pipeline metrics (GUI / terminal display)
struct PipelineMetrics
{
    double perception_ms = 0.0;     // avg perception processing time
    double perception_hz = 0.0;     // actual perception rate
    double plan_ms = 0.0;           // last plan time
    double perception_age_ms = 0.0; // perception data age at planning time
    double plan_age_ms = 0.0;       // current plan age
    double end_to_end_ms = 0.0;     // end-to-end latency: perception acquisition -> executor consumption
    std::uint64_t replans = 0;
    std::uint64_t perception_seq = 0;
    std::uint64_t plan_seq = 0;
    bool replan_pending = false;
};
