#include "Controller.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace {

// projection of point p onto segment ab, and the squared distance
double pointToSegmentProjection(const Eigen::Vector2d& p,
                                const Eigen::Vector2d& a,
                                const Eigen::Vector2d& b,
                                Eigen::Vector2d& proj)
{
    Eigen::Vector2d ab = b - a;
    double t = (p - a).dot(ab) / std::max(ab.squaredNorm(), 1e-12);
    t = std::max(0.0, std::min(1.0, t));
    proj = a + t * ab;
    return (p - proj).squaredNorm();
}

double pointToSegmentDistance(const Eigen::Vector2d& p,
                              const Eigen::Vector2d& a,
                              const Eigen::Vector2d& b)
{
    Eigen::Vector2d proj;
    return std::sqrt(pointToSegmentProjection(p, a, b, proj));
}

// uniformly spaced straight-line reference path
std::vector<Eigen::Vector2d> sampleLinePath(const Eigen::Vector2d& start,
                                            const Eigen::Vector2d& goal,
                                            double step)
{
    std::vector<Eigen::Vector2d> path;

    Eigen::Vector2d dir = goal - start;
    double length = dir.norm();
    if (length < 1e-6)
    {
        path.push_back(start);
        return path;
    }

    dir /= length;
    int num_steps = static_cast<int>(length / step);
    path.reserve(num_steps + 2);

    for (int i = 0; i <= num_steps; ++i)
        path.push_back(start + dir * step * i);

    if ((path.back() - goal).norm() > 1e-6)
        path.push_back(goal);

    return path;
}

double nowSeconds()
{
    return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

} // namespace

Controller::Controller(const Eigen::Vector2d& goal,
                       double window_len,
                       double lookahead_dist,
                       double robot_speed,
                       double safe_r,
                       double step_spacing,
                       double plan_expiry_ms)
    : goal_(goal),
      window_len_(window_len),
      lookahead_dist_(lookahead_dist),
      robot_speed_(robot_speed),
      safe_r_(safe_r),
      step_spacing_(step_spacing),
      plan_expiry_ms_(plan_expiry_ms)
{
}

void Controller::setGoal(const Eigen::Vector2d& goal,
                         const Eigen::Vector2d& robot_pos)
{
    goal_ = goal;
    remaining_ = sampleLinePath(robot_pos, goal_, step_spacing_);
    resetState();
    // allow an immediate replan so the planner smooths the new goal path
    last_replan_time_ = -std::numeric_limits<double>::infinity();
}

void Controller::init(const Eigen::Vector2d& start)
{
    remaining_ = sampleLinePath(start, goal_, step_spacing_);
    resetState();
}

void Controller::initWithPath(const std::vector<Eigen::Vector2d>& path)
{
    remaining_ = path;
    resetState();
}

void Controller::resetState()
{
    executed_.clear();
    reached_ = false;
    collision_ = false;
    dist_to_path_ = 0.0;
    last_anchor_ = -1;
    last_replan_time_ = -std::numeric_limits<double>::infinity();
}

int Controller::closestIndex(const Eigen::Vector2d& p) const
{
    int best = 0;
    double best_dist = std::numeric_limits<double>::max();

    for (int i = 0; i < static_cast<int>(remaining_.size()); ++i)
    {
        double d = (remaining_[i] - p).squaredNorm();
        if (d < best_dist)
        {
            best_dist = d;
            best = i;
        }
    }
    return best;
}

// whether the path within the next 5 m conflicts with obstacles
bool Controller::isBlocked(const ObstacleMap& obstacles) const
{
    double accumulated = 0.0;

    for (int i = 0; i + 1 < static_cast<int>(remaining_.size()); ++i)
    {
        const Eigen::Vector2d& p1 = remaining_[i];
        const Eigen::Vector2d& p2 = remaining_[i + 1];

        accumulated += (p2 - p1).norm();
        if (accumulated > 5.0) break;

        for (const auto& group : obstacles)
        {
            for (const auto& obs : group)
            {
                if (!obs.active) continue;
                double d = pointToSegmentDistance(obs.center, p1, p2);
                if (d < obs.radius + safe_r_)
                    return true;
            }
        }
    }
    return false;
}

PlanRequest Controller::buildWindowRequest(const RobotState& robot,
                                           const std::string& reason)
{
    PlanRequest req;
    req.reason = reason;

    if (remaining_.empty())
        return req;

    int idx0 = std::min(closestIndex(robot.pos),
                        static_cast<int>(remaining_.size()) - 1);

    // accumulate arc length along the path to determine the window anchor
    double acc = 0.0;
    int anchor = idx0;
    for (int i = idx0; i + 1 < static_cast<int>(remaining_.size()) &&
                        acc < window_len_; ++i)
    {
        acc += (remaining_[i + 1] - remaining_[i]).norm();
        anchor = i + 1;
    }

    // window reference path: robot position + remaining path (up to the anchor)
    req.ref_path.clear();
    req.ref_path.push_back(robot.pos);
    for (int i = idx0 + 1; i <= anchor; ++i)
        req.ref_path.push_back(remaining_[i]);

    // deduplicate
    std::vector<Eigen::Vector2d> filtered;
    for (const auto& p : req.ref_path)
    {
        if (filtered.empty() ||
            (p - filtered.back()).norm() > 1e-3)
            filtered.push_back(p);
    }
    req.ref_path = std::move(filtered);

    req.start = robot.pos;
    req.goal = remaining_[anchor];
    req.anchor_idx = anchor;
    last_anchor_ = anchor;

    return req;
}

PlanRequest Controller::makeInitialRequest(const RobotState& robot) const
{
    PlanRequest req;
    req.reason = "initial";
    req.ref_path = remaining_;
    req.start = robot.pos;
    req.goal = goal_;
    req.anchor_idx = static_cast<int>(remaining_.size()) - 1;
    return req;
}

PlanRequest Controller::update(const RobotState& robot,
                               const ObstacleMap& obstacles,
                               double plan_age_ms,
                               bool replan_in_flight)
{
    PlanRequest none;
    if (reached_ || collision_ || remaining_.size() < 3)
        return none;

    std::string reason;
    if (isBlocked(obstacles))
    {
        reason = "blocked";
    }
    else if (plan_age_ms > plan_expiry_ms_)
    {
        reason = "stale";
    }
    else if (dist_to_path_ > 0.6)
    {
        reason = "off-path";
    }
    else
    {
        return none;
    }

    // emergencies (collision) may interrupt an in-flight plan; stale refresh is skipped if a plan is in flight
    if (replan_in_flight && reason != "blocked")
        return none;

    double now = nowSeconds();
    if (now - last_replan_time_ < 0.12)
        return none;
    last_replan_time_ = now;

    return buildWindowRequest(robot, reason);
}

void Controller::applyPlan(const PlanFrame& plan)
{
    const auto& new_window = plan.frame.data;
    if (new_window.size() < 2 || remaining_.empty())
        return;

    int anchor = plan.anchor_idx;
    if (anchor < 0 || anchor >= static_cast<int>(remaining_.size()))
    {
        remaining_ = new_window;
        return;
    }

    // window replacement: new path + old path tail beyond the anchor
    std::vector<Eigen::Vector2d> tail(remaining_.begin() + anchor + 1,
                                      remaining_.end());
    remaining_ = new_window;
    remaining_.insert(remaining_.end(), tail.begin(), tail.end());
}

RobotState Controller::follow(const RobotState& robot, double dt,
                              const ObstacleMap& obstacles)
{
    RobotState out = robot;

    if (reached_ || collision_)
        return out;

    if (remaining_.size() < 2)
    {
        out.pos = goal_;
        reached_ = true;
        return out;
    }

    // 1. nearest path point (segment projection)
    int closest_idx = 0;
    Eigen::Vector2d best_proj;
    double min_dist2 = std::numeric_limits<double>::max();

    for (int i = 0; i + 1 < static_cast<int>(remaining_.size()); ++i)
    {
        Eigen::Vector2d proj;
        double d2 = pointToSegmentProjection(robot.pos,
                                             remaining_[i],
                                             remaining_[i + 1],
                                             proj);
        if (d2 < min_dist2)
        {
            min_dist2 = d2;
            closest_idx = i;
            best_proj = proj;
        }
    }
    dist_to_path_ = std::sqrt(min_dist2);

    // 2. lookahead target (by arc length)
    int target_idx = closest_idx + 1;
    double arc = 0.0;
    for (int i = closest_idx;
         i + 1 < static_cast<int>(remaining_.size()) && arc < lookahead_dist_;
         ++i)
    {
        arc += (remaining_[i + 1] - remaining_[i]).norm();
        target_idx = i + 1;
    }

    Eigen::Vector2d target = remaining_[target_idx];

    // keep the target ahead of the robot
    Eigen::Vector2d to_goal = (goal_ - robot.pos).normalized();
    Eigen::Vector2d to_target = (target - robot.pos).normalized();
    if (to_goal.dot(to_target) < 0)
        target = goal_;

    // 4. heading (may be corrected by dynamic dodging)
    Eigen::Vector2d dir = target - robot.pos;
    double dist = dir.norm();

    // 3. lateral error -> speed scaling (quadratic mapping)
    double speed_scale = 1.0;
    if (dist_to_path_ > 0.5)
    {
        speed_scale = 0.2;
    }
    else if (dist_to_path_ > 0.01)
    {
        double ratio = (dist_to_path_ - 0.01) / 0.49;
        speed_scale = 0.2 + 0.8 * (1.0 - ratio * ratio);
    }

    // 3.5 proximity slowdown: reduce speed near obstacles to avoid cutting corners
    double prox_scale = 1.0;
    for (const auto& group : obstacles)
    {
        for (const auto& obs : group)
        {
            double d = (robot.pos - obs.center).norm() - obs.radius;
            if (d < 3.5)
            {
                double s = (d - 0.4) / 3.1; // within 0.4 m -> 0.15x, beyond 3.5 m -> 1x
                prox_scale = std::min(prox_scale, std::max(0.15, std::min(1.0, s)));
            }
        }
    }
    speed_scale *= prox_scale;

    // 3.6 dynamic dodging (velocity-obstacle style): predict whether an obstacle will hit the robot,
    //     and dodge laterally to avoid being rear-ended
    Eigen::Vector2d dodge(0.0, 0.0);
    for (const auto& group : obstacles)
    {
        for (const auto& obs : group)
        {
            if (obs.velocity.norm() < 1e-3) continue;

            Eigen::Vector2d rel_v = out.vel - obs.velocity;
            Eigen::Vector2d rel_p = out.pos - obs.center;
            double rel_speed2 = rel_v.squaredNorm();
            if (rel_speed2 < 1e-6) continue;

            double dp = rel_p.dot(rel_v);
            if (dp >= 0.0) continue; // moving away

            double tca = -dp / rel_speed2;
            if (tca > 1.5) continue; // collision is far away

            Eigen::Vector2d closest = rel_p + rel_v * tca;
            double d_close = closest.norm();
            double threat = obs.radius + 0.8;

            if (d_close < threat)
            {
                Eigen::Vector2d n = closest.normalized();
                Eigen::Vector2d side(-n.y(), n.x());
                // steer to the side opposite the obstacle motion (pass behind it)
                if (side.dot(obs.velocity) > 0.0)
                    side = -side;
                double strength = (threat - d_close) / threat;
                dodge += side * strength;
            }
        }
    }

    if (dodge.norm() > 1e-3)
    {
        ++dodge_count_;
        dodge.normalize();
        dir = (dir.normalized() + dodge * 0.9).normalized();
        speed_scale = std::min(speed_scale, 0.55);
    }

    // 3.7 hard-brake fallback: stop completely when almost touching
    for (const auto& group : obstacles)
    {
        for (const auto& obs : group)
        {
            double d = (robot.pos - obs.center).norm() - obs.radius;
            if (d < 0.25)
            {
                speed_scale = 0.0;
                break;
            }
        }
        if (speed_scale == 0.0) break;
    }

    if (dist > 1e-6)
    {
        double step = std::min(robot_speed_ * std::max(dt, 1e-4), dist);
        step *= speed_scale;

        out.pos += dir.normalized() * step;
        out.vel = dir.normalized() * (step / std::max(dt, 1e-4));
        out.speed = step / std::max(dt, 1e-4);

        executed_.push_back(out.pos);
    }

    // 5. drop the traversed path prefix
    if (closest_idx > 0)
    {
        remaining_.erase(remaining_.begin(),
                         remaining_.begin() + closest_idx);
    }

    // 6. goal reached
    if ((out.pos - goal_).norm() < 0.2)
    {
        out.pos = goal_;
        reached_ = true;
        std::cout << "[controller] Goal reached." << std::endl;
    }

    return out;
}
