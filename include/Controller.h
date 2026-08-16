#pragma once

#include <limits>
#include <vector>

#include "common.h"

// Executor (pure-pursuit controller)
//   - maintains the remaining path, requests replans via a sliding window
//   - consumes the latest plan frame and splices it into the remaining path
//   - judges plan freshness from perception/plan timestamps
class Controller
{
public:
    Controller(const Eigen::Vector2d& goal,
               double window_len = 14.0,
               double lookahead_dist = 2.5,
               double robot_speed = 1.5,
               double safe_r = 0.2,
               double step_spacing = 0.05,
               double plan_expiry_ms = 250.0);

    // User waypoint: switch to a new goal and rebuild the reference path from the current robot position
    void setGoal(const Eigen::Vector2d& goal, const Eigen::Vector2d& robot_pos);

    void init(const Eigen::Vector2d& start);
    // initialize with a custom initial reference path (may include entry waypoints)
    void initWithPath(const std::vector<Eigen::Vector2d>& path);

    // decide whether replanning is needed; returns a request with a reason, or an empty reason
    PlanRequest update(const RobotState& robot,
                       const ObstacleMap& obstacles,
                       double plan_age_ms,
                       bool replan_in_flight);

    // splice a new plan result back into the remaining path (sliding-window replacement)
    void applyPlan(const PlanFrame& plan);

    // advance one frame along the remaining path, return the updated robot state
    RobotState follow(const RobotState& robot, double dt,
                      const ObstacleMap& obstacles);

    void setCollided(bool c) { collision_ = c; }
    bool reached() const { return reached_; }
    bool collided() const { return collision_; }
    double distToPath() const { return dist_to_path_; }
    std::uint64_t dodgeCount() const { return dodge_count_; }

    const std::vector<Eigen::Vector2d>& remainingPath() const { return remaining_; }
    const std::vector<Eigen::Vector2d>& executedPath() const { return executed_; }

    PlanRequest makeInitialRequest(const RobotState& robot) const;

private:
    void resetState();
    bool isBlocked(const ObstacleMap& obstacles) const;
    int closestIndex(const Eigen::Vector2d& p) const;
    PlanRequest buildWindowRequest(const RobotState& robot,
                                   const std::string& reason);

    Eigen::Vector2d goal_;
    double window_len_;
    double lookahead_dist_;
    double robot_speed_;
    double safe_r_;
    double step_spacing_;
    double plan_expiry_ms_;

    std::vector<Eigen::Vector2d> remaining_;
    std::vector<Eigen::Vector2d> executed_;

    bool reached_ = false;
    bool collision_ = false;
    double dist_to_path_ = 0.0;
    int last_anchor_ = -1;
    double last_replan_time_ = -std::numeric_limits<double>::infinity();
    std::uint64_t dodge_count_ = 0;
};
