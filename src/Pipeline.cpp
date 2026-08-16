#include "Pipeline.h"

#include <algorithm>
#include <iostream>
#include <limits>

namespace {

// Plan safety validation: no path point may intrude into inflated obstacles
// Returns (passed, min clearance)
std::pair<bool, double> checkPlanSafety(
    const std::vector<Vector2d>& path,
    const ObstacleMap& obstacles,
    double safety_margin = 0.05)
{
    double min_clear = std::numeric_limits<double>::max();

    for (const auto& p : path)
    {
        for (const auto& group : obstacles)
        {
            for (const auto& obs : group)
            {
                double clear = (p - obs.center).norm() - obs.radius;
                min_clear = std::min(min_clear, clear);
            }
        }
    }

    return {min_clear >= -safety_margin, min_clear};
}

// Whether a point is inside any obstacle (or outside the world bounds)
bool pointBlocked(const Vector2d& p, const ObstacleMap& obstacles,
                  double margin, double xmin, double xmax,
                  double ymin, double ymax)
{
    if (p.x() < xmin || p.x() > xmax ||
        p.y() < ymin || p.y() > ymax)
        return true;

    for (const auto& group : obstacles)
        for (const auto& obs : group)
            if ((p - obs.center).norm() < obs.radius + margin)
                return true;

    return false;
}

// Geometric emergency bypass: fallback when the main planner keeps failing
// Offset perpendicular to the obstacle-robot direction, picking the side with more clearance
std::vector<Vector2d> emergencyBypass(
    const std::vector<Vector2d>& ref,
    const ObstacleMap& obstacles,
    double xmin, double xmax, double ymin, double ymax,
    double standoff = 1.2)
{
    if (ref.size() < 2)
        return ref;

    // 1. find the first blocked reference point
    int hit = -1;
    for (int i = 0; i < static_cast<int>(ref.size()); ++i)
    {
        if (pointBlocked(ref[i], obstacles, 0.0, xmin, xmax, ymin, ymax))
        {
            hit = i;
            break;
        }
    }
    if (hit < 0)
        return ref;

    // 2. take the nearest blocking obstacle
    const Obstacle* block = nullptr;
    double best_d = std::numeric_limits<double>::max();
    for (const auto& group : obstacles)
        for (const auto& obs : group)
        {
            double d = (obs.center - ref[hit]).norm();
            if (d < best_d)
            {
                best_d = d;
                block = &obs;
            }
        }
    if (!block)
        return ref;

    // 3. left/right candidates (away from the obstacle + away from world bounds)
    Eigen::Vector2d to_obs = (block->center - ref[0]).normalized();
    Eigen::Vector2d normal(-to_obs.y(), to_obs.x());
    const double offset = block->radius + standoff;

    Eigen::Vector2d left = block->center + normal * offset;
    Eigen::Vector2d right = block->center - normal * offset;

    bool left_ok = !pointBlocked(left, obstacles, 0.3, xmin, xmax, ymin, ymax);
    bool right_ok = !pointBlocked(right, obstacles, 0.3, xmin, xmax, ymin, ymax);

    if (!left_ok && !right_ok)
        return ref; // cannot bypass: keep the original path and wait for the next replan

    Eigen::Vector2d waypoint;
    if (left_ok && right_ok)
    {
        waypoint = ((left - ref[0]).norm() < (right - ref[0]).norm())
                       ? left : right;
    }
    else
    {
        waypoint = left_ok ? left : right;
    }

    // 4. return point: a path point past the obstacle (guaranteed outside it)
    int back = hit;
    while (back < static_cast<int>(ref.size()) &&
           (ref[back] - block->center).norm() < block->radius + 1.5)
        ++back;
    if (back >= static_cast<int>(ref.size()))
        back = static_cast<int>(ref.size()) - 1;

    // 5. sample the polyline: start -> transition -> candidate -> return point
    std::vector<Vector2d> out;
    const double step = 0.05;

    auto appendLine = [&](const Vector2d& a, const Vector2d& b)
    {
        double len = (b - a).norm();
        int n = std::max(1, static_cast<int>(std::ceil(len / step)));
        for (int i = 0; i <= n; ++i)
            out.push_back(a + (b - a) * (double(i) / n));
    };

    appendLine(ref[0], ref[hit]);
    appendLine(ref[hit], waypoint);
    appendLine(waypoint, ref[back]);
    appendLine(ref[back], ref.back());

    // deduplicate
    std::vector<Vector2d> filtered;
    for (const auto& p : out)
        if (filtered.empty() || (p - filtered.back()).norm() > 1e-3)
            filtered.push_back(p);

    return filtered;
}

} // namespace

PlanningPipeline::PlanningPipeline(World& world,
                                   double safe_r,
                                   double perception_hz)
    : world_(world),
      perception_hz_(perception_hz),
      safe_r_(safe_r),
      perception_(std::make_unique<PerceptionNode>(safe_r)),
      planner_(std::make_unique<Planner>(safe_r))
{
    latest_perception_.stamp = Clock::now();
    latest_plan_.frame.stamp = Clock::now();
}

void PlanningPipeline::setWorkBounds(double xmin, double xmax,
                                     double ymin, double ymax)
{
    work_xmin_ = xmin; work_xmax_ = xmax;
    work_ymin_ = ymin; work_ymax_ = ymax;
}

PlanningPipeline::~PlanningPipeline()
{
    stop();
}

void PlanningPipeline::start()
{
    if (perception_thread_.joinable() || planner_thread_.joinable())
        return;

    running_ = true;
    perception_thread_ = std::thread(&PlanningPipeline::perceptionLoop, this);
    planner_thread_ = std::thread(&PlanningPipeline::plannerLoop, this);
}

void PlanningPipeline::stop()
{
    if (!running_.exchange(false))
        return;

    {
        std::lock_guard<std::mutex> lock(request_mutex_);
        pending_request_.reset();
    }
    request_cv_.notify_all();

    if (perception_thread_.joinable())
        perception_thread_.join();
    if (planner_thread_.joinable())
        planner_thread_.join();
}

// ---------------------------------------------------------------------------
// Perception thread: fixed rate, emits timestamped perception frames
// ---------------------------------------------------------------------------
void PlanningPipeline::perceptionLoop()
{
    const double period_s = 1.0 / std::max(perception_hz_, 1.0);

    while (running_)
    {
        auto loop_start = Clock::now();

        // 1. take a world snapshot (locked, thread-safe)
        auto detections = world_.snapshotCircleObstacles(0.1);

        // 2. process (downsample -> cluster -> simplify)
        PerceptionFrame frame = perception_->process(detections);
        frame.stamp = loop_start; // perception acquisition timestamp (not processing completion)

        // 3. publish
        {
            std::lock_guard<std::mutex> lock(perception_mutex_);
            latest_perception_ = frame;
        }

        perception_ms_.store(msBetween(loop_start, Clock::now()));

        // 4. frame-rate control (estimate perception rate from the actual period)
        auto elapsed = msSince(loop_start);
        double sleep_ms = period_s * 1000.0 - elapsed;
        if (sleep_ms > 0.5)
            std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(sleep_ms));

        std::lock_guard<std::mutex> lock(metrics_mutex_);
        metrics_.perception_hz = 1000.0 / std::max(msBetween(loop_start, Clock::now()), 1e-6);
    }
}

// ---------------------------------------------------------------------------
// Planning thread: sliding window + latest perception frame
// ---------------------------------------------------------------------------
void PlanningPipeline::plannerLoop()
{
    while (running_)
    {
        PlanRequest req;
        {
            std::unique_lock<std::mutex> lock(request_mutex_);
            request_cv_.wait(lock, [&]()
            {
                return !running_ || pending_request_.has_value();
            });
            if (!running_) break;
            req = std::move(*pending_request_);
            pending_request_.reset();
        }

        // read the latest perception frame and robot state
        RobotState robot;
        PerceptionFrame pf;
        {
            std::lock_guard<std::mutex> lock(robot_mutex_);
            robot = robot_;
        }
        {
            std::lock_guard<std::mutex> lock(perception_mutex_);
            pf = latest_perception_;
        }

        if (req.ref_path.size() < 2)
        {
            replan_pending_.store(false);
            continue;
        }

        // run sliding-window planning
        auto t0 = Clock::now();
        double perception_age = msBetween(pf.stamp, t0);

        std::vector<Vector2d> window_path;
        if (!pf.data.empty())
        {
            window_path = planner_->plan(req.ref_path, pf.data,
                                         req.start, req.goal, robot.vel);
        }
        else
        {
            window_path = req.ref_path; // fall back to the reference path if perception is missing
        }

        // Safety check: on failure try the geometric bypass first; keep the previous plan if still unsafe
        auto [safe, min_clear] = checkPlanSafety(window_path, pf.data, 0.05);
        if (!safe)
        {
            auto bypass = emergencyBypass(req.ref_path, pf.data,
                                          work_xmin_, work_xmax_,
                                          work_ymin_, work_ymax_, 1.2);
            auto [bypass_safe, bypass_clear] =
                checkPlanSafety(bypass, pf.data, 0.05);

            static auto last_warn = Clock::now();
            if (bypass_safe)
            {
                window_path = std::move(bypass);
                safe = true;
                if (msSince(last_warn) > 500.0)
                {
                    std::cout << "[pipeline] emergency bypass (clearance "
                              << bypass_clear << " m)" << std::endl;
                    last_warn = Clock::now();
                }
            }
            else
            {
                replan_pending_.store(false);
                {
                    std::lock_guard<std::mutex> lock(metrics_mutex_);
                    metrics_.plan_ms = msBetween(t0, Clock::now());
                    metrics_.perception_age_ms = perception_age;
                }
                if (msSince(last_warn) > 500.0)
                {
                    std::cout << "[pipeline] plan rejected (min clearance "
                              << min_clear << " m), keeping previous plan"
                              << std::endl;
                    last_warn = Clock::now();
                }
                continue;
            }
        }

        auto t1 = Clock::now();
        double plan_ms = msBetween(t0, t1);

        // publish the plan frame (with perception association info)
        PlanFrame frame;
        frame.frame.data = std::move(window_path);
        frame.frame.stamp = t1;
        frame.frame.seq = ++plan_seq_;
        frame.anchor_idx = req.anchor_idx;
        frame.perception_seq = pf.seq;
        frame.perception_stamp = pf.stamp;
        frame.plan_ms = plan_ms;
        frame.perception_age_ms = perception_age;
        frame.reason = req.reason;

        {
            std::lock_guard<std::mutex> lock(plan_mutex_);
            latest_plan_ = frame;
        }

        replans_.fetch_add(1);

        {
            std::lock_guard<std::mutex> lock(metrics_mutex_);
            metrics_.plan_ms = plan_ms;
            metrics_.perception_age_ms = perception_age;
            metrics_.replans = replans_.load();
            metrics_.plan_seq = plan_seq_.load();
            metrics_.perception_seq = pf.seq;
            metrics_.replan_pending = false;
        }
        replan_pending_.store(false);
    }
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------
void PlanningPipeline::updateRobot(const RobotState& robot)
{
    std::lock_guard<std::mutex> lock(robot_mutex_);
    robot_ = robot;
}

void PlanningPipeline::submitRequest(PlanRequest req)
{
    {
        std::lock_guard<std::mutex> lock(request_mutex_);
        pending_request_ = std::move(req);
    }
    replan_pending_.store(true);
    request_cv_.notify_all();
}

bool PlanningPipeline::latestPlan(PlanFrame& out) const
{
    std::lock_guard<std::mutex> lock(plan_mutex_);
    if (plan_seq_.load() == 0)
        return false;
    out = latest_plan_;
    return true;
}

bool PlanningPipeline::perceptionReady() const
{
    std::lock_guard<std::mutex> lock(perception_mutex_);
    return latest_perception_.seq > 0;
}

bool PlanningPipeline::replanInFlight() const
{
    return replan_pending_.load();
}

double PlanningPipeline::planAgeMs() const
{
    std::lock_guard<std::mutex> lock(plan_mutex_);
    if (plan_seq_.load() == 0)
        return 1e9;
    return msSince(latest_plan_.frame.stamp);
}

void PlanningPipeline::reportEndToEnd(double ms)
{
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    metrics_.end_to_end_ms = ms;
}

PipelineMetrics PlanningPipeline::metrics() const
{
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    PipelineMetrics m = metrics_;

    m.perception_ms = perception_ms_.load();
    m.plan_age_ms = planAgeMs();
    m.replan_pending = replan_pending_.load();

    {
        std::lock_guard<std::mutex> lock2(plan_mutex_);
        m.plan_seq = plan_seq_.load();
    }
    return m;
}
