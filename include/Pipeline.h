#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

#include "common.h"
#include "Perception.h"
#include "Planner.h"
#include "World.h"

// Planning pipeline:
//   - perception thread: samples world snapshots at a fixed rate, emits timestamped frames
//   - planning thread: event-driven (sliding-window requests), plans from the latest perception frame,
//     emits plan frames with timestamps / perception association
// All shared state is managed with timestamps + sequence numbers for end-to-end latency measurement.
class PlanningPipeline
{
public:
    PlanningPipeline(World& world,
                     double safe_r = 0.2,
                     double perception_hz = 30.0);
    ~PlanningPipeline();

    PlanningPipeline(const PlanningPipeline&) = delete;
    PlanningPipeline& operator=(const PlanningPipeline&) = delete;

    void start();
    void stop();

    // ---- Executor -> pipeline ----
    void updateRobot(const RobotState& robot);
    void submitRequest(PlanRequest req);

    // ---- Pipeline -> executor ----
    bool latestPlan(PlanFrame& out) const;
    bool perceptionReady() const;
    bool replanInFlight() const;
    double planAgeMs() const;
    void reportEndToEnd(double ms);

    PipelineMetrics metrics() const;

private:
    void perceptionLoop();
    void plannerLoop();

    World& world_;
    double perception_hz_;
    double safe_r_;

    std::unique_ptr<PerceptionNode> perception_;
    std::unique_ptr<Planner> planner_;

    std::thread perception_thread_;
    std::thread planner_thread_;
    std::atomic<bool> running_{true};

    // Robot state
    mutable std::mutex robot_mutex_;
    RobotState robot_;

    // Latest perception frame
    mutable std::mutex perception_mutex_;
    PerceptionFrame latest_perception_;
    std::atomic<double> perception_ms_{0.0};

    // Plan requests (event queue, keeps only the latest)
    mutable std::mutex request_mutex_;
    std::condition_variable request_cv_;
    std::optional<PlanRequest> pending_request_;
    std::atomic<bool> replan_pending_{false};

    // Latest plan frame
    mutable std::mutex plan_mutex_;
    PlanFrame latest_plan_;
    std::atomic<std::uint64_t> plan_seq_{0};
    std::atomic<std::uint64_t> replans_{0};

    // Metrics
    mutable std::mutex metrics_mutex_;
    PipelineMetrics metrics_;
};
