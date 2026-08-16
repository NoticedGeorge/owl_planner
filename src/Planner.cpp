//
// Created by georg on 2026/4/18.
//
#include "../include/Planner.h"
#include <limits>

using namespace std;

Planner::Planner(double safe_r, double detection_margin)
    : safe_r_(safe_r),
      detection_margin_(detection_margin) {}

double Planner::pointToSegmentDistance(
    const Eigen::Vector2d& p,
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b)
{
    Eigen::Vector2d ab = b - a;
    Eigen::Vector2d ap = p - a;

    double t = ap.dot(ab) / ab.squaredNorm();
    t = std::max(0.0, std::min(1.0, t));

    Eigen::Vector2d proj = a + t * ab;
    return (p - proj).norm();
}

std::vector<Vector2d> extractKeyPoints(const std::vector<Vector2d>& raw_path, double dist_threshold = 0.5) {
    if (raw_path.size() < 3) return raw_path;
    std::vector<Vector2d> key_points;
    key_points.push_back(raw_path.front());

    double current_dist_sum = 0.0;
    for (size_t i = 1; i < raw_path.size() - 1; ++i) {
        current_dist_sum += (raw_path[i] - raw_path[i-1]).norm();
        if (current_dist_sum >= dist_threshold) {
            key_points.push_back(raw_path[i]);
            current_dist_sum = 0.0;
        }
    }
    key_points.push_back(raw_path.back());
    return key_points;
}

std::vector<Vector2d> generateKinematicSmoothPath(const std::vector<Vector2d>& path,
                                                   double min_turn_radius_ratio = 0.45,
                                                   int segments = 20) {
    // 1. extract feature points first, turning the dense path into polyline control points
    std::vector<Vector2d> key_points = extractKeyPoints(path);

    if (key_points.size() < 3) return path;

    std::vector<Vector2d> refined;
    refined.push_back(key_points.front());

    for (size_t i = 1; i < key_points.size() - 1; ++i) {
        Vector2d v1 = (key_points[i] - key_points[i-1]).normalized();
        Vector2d v2 = (key_points[i+1] - key_points[i]).normalized();

        // 2. key adjustment: use a larger smoothing distance so the turn starts earlier
        // min_turn_radius_ratio = 0.45 means the chamfer takes 90% of each segment
        double d1 = (key_points[i]-key_points[i-1]).norm();
        double d2 = (key_points[i+1]-key_points[i]).norm();
        double l = std::min(d1, d2) * min_turn_radius_ratio;

        // 3. generate Bezier control points A and B
        Vector2d A = key_points[i] - v1 * l;
        Vector2d B = key_points[i] + v2 * l;

        refined.push_back(A);
        // 4. generate the curve
        for (int j = 1; j < segments; ++j) {
            double t = (double)j / segments;
            refined.push_back(std::pow(1-t, 2)*A + 2*(1-t)*t*key_points[i] + std::pow(t, 2)*B);
        }
        refined.push_back(B);
    }

    refined.push_back(key_points.back());
    return refined;
}

// 5. previous uniform-resampling logic (unchanged)
std::vector<Vector2d> resamplePath(const std::vector<Vector2d>& smooth_path, double step) {
    if (smooth_path.size() < 2) return smooth_path;

    std::vector<double> lengths;
    lengths.push_back(0.0);
    double total_length = 0.0;
    for (size_t i = 1; i < smooth_path.size(); ++i) {
        total_length += (smooth_path[i] - smooth_path[i-1]).norm();
        lengths.push_back(total_length);
    }

    std::vector<Vector2d> resampled_path;
    resampled_path.push_back(smooth_path.front());

    double current_dist = step;
    size_t last_idx = 0;

    while (current_dist < total_length) {
        while (last_idx < lengths.size() - 2 && lengths[last_idx + 1] < current_dist) {
            last_idx++;
        }
        double segment_len = lengths[last_idx + 1] - lengths[last_idx];
        double t = (current_dist - lengths[last_idx]) / segment_len;
        Vector2d p = (1.0 - t) * smooth_path[last_idx] + t * smooth_path[last_idx + 1];
        resampled_path.push_back(p);
        current_dist += step;
    }

    if ((resampled_path.back() - smooth_path.back()).norm() > 1e-3) {
        resampled_path.push_back(smooth_path.back());
    }
    return resampled_path;
}

// final kinematic smoothing entry point
std::vector<Vector2d> Planner::getFinalSmoothPath(const std::vector<Vector2d>& raw_path) {

    // 1. generate a geometrically smooth path with large chamfers
    // ratio = 0.45 satisfies the "smooth even more" requirement
    auto kinematic_path = generateKinematicSmoothPath( raw_path, 0.45, 20);

    // 2. resample the smoothed path so the control step (speed) is a constant 0.05
    return resamplePath(kinematic_path, 0.05);
}

void Planner::spliceTrajectory(
    std::vector<Vector2d>& final_path,
    const std::vector<Vector2d>& smooth_traj) {
    if (final_path.empty() || smooth_traj.empty())
        return;

    Vector2d start_pt = smooth_traj.front();

    // ========= 1. find the nearest point =========
    int nearest_idx = -1;
    double min_dist = 1e9;

    for (size_t i = 0; i < final_path.size(); i++)
    {
        double d = (final_path[i] - start_pt).norm();
        if (d < min_dist)
        {
            min_dist = d;
            nearest_idx = i;
        }
    }

    if (nearest_idx < 0) return;

    // ========= 2. truncate final_path =========
    final_path.erase(final_path.begin() + nearest_idx,
                     final_path.end());

    // ========= 3. insert smooth_traj =========
    final_path.insert(final_path.end(),
                      smooth_traj.begin(),
                      smooth_traj.end());
}

std::vector<Vector2d> Planner::buildControlPoints(
    const std::vector<Vector2d>& bypass,
    const std::vector<Vector2d>& final_path,
    const Vector2d& start,
    const Vector2d& goal){
    std::vector<Vector2d> control_pts;

    // ========= 1. take the final_path tail first =========
    double length = 1.0;
    if (!final_path.empty())
    {
        double total_len = 0.0;
        for (size_t i = 1; i < final_path.size(); i++)
        {
            total_len += (final_path[i] - final_path[i - 1]).norm();
        }

        if (total_len <= length)
        {
            control_pts.insert(control_pts.end(),
                               final_path.begin(),
                               final_path.end());
        }
        else
        {
            double acc = 0.0;
            std::vector<Vector2d> temp;

            for (int i = final_path.size() - 1; i > 0; i--)
            {
                double d = (final_path[i] - final_path[i - 1]).norm();
                acc += d;

                temp.push_back(final_path[i]);

                if (acc >= length)
                    break;
            }

            std::reverse(temp.begin(), temp.end());

            control_pts.insert(control_pts.end(),
                               temp.begin(),
                               temp.end());
        }
    }

    // ========= 2. insert the bypass (middle segment) =========
    for (const auto& p : bypass)
    {
        control_pts.push_back(p);
    }

    // ========= 3. rejoin the main path =========
    Vector2d last = control_pts.back();
    Vector2d main_dir = (goal - start).normalized();

    double proj_len = (last - start).dot(main_dir);
    Vector2d proj = start + proj_len * main_dir;

    double dist_to_goal = (goal - proj).norm();

    std::vector<Vector2d> extension_pts;

    double step = 0.05;

    if (dist_to_goal <= 1.0)
    {
        int steps = std::max(1, int(dist_to_goal / step));

        for (int i = 1; i <= steps; i++)
        {
            double t = double(i) / steps;
            Vector2d p = proj * (1 - t) + goal * t;
            extension_pts.push_back(p);
        }
    }
    else
    {
        int steps = int(length / step);

        for (int i = 1; i <= steps; i++)
        {
            Vector2d p = proj + main_dir * (i * step);
            extension_pts.push_back(p);
        }
    }

    control_pts.insert(control_pts.end(),
                       extension_pts.begin(),
                       extension_pts.end());

    // ========= 4. deduplicate (important) =========
    std::vector<Vector2d> filtered;
    for (const auto& p : control_pts)
    {
        if (filtered.empty() ||
            (p - filtered.back()).norm() > 1e-3)
        {
            filtered.push_back(p);
        }
    }

    return filtered;
}

std::pair<double, double> Planner::computeSideMetric(
    const std::vector<Obstacle>& group,
    const std::vector<std::vector<Obstacle>>& groups,
    const Vector2d& start,
    const Vector2d& goal) {
    Vector2d dir = (goal - start).normalized();
    Vector2d normal(-dir.y(), dir.x());

    double left_count = 0.0;
    double right_count = 0.0;

    double left_penalty = 0.0;
    double right_penalty = 0.0;

    for (const auto& g : groups)
    {
        for (const auto& obs : g)
        {
            // ==== exclude the current group ====
            bool in_current_group = false;
            for (const auto& og : group)
            {
                if (&obs == &og)
                {
                    in_current_group = true;
                    break;
                }
            }
            if (in_current_group) continue;

            for (const auto& gobs : group)
            {
                double d = (obs.center - gobs.center).norm();
                // ==== far-field filter (for density) ====
                if (d < 5.0 + obs.radius + gobs.radius)
                {
                    Vector2d v = obs.center - start;
                    double side = v.dot(normal);

                    if (side > 0)
                        left_count += 1.0;
                    else
                        right_count += 1.0;

                    double safe_dist = obs.radius + gobs.radius + safe_r_;

                    if (d <= safe_dist)
                    {
                        double p = (safe_dist - d) / safe_r_;

                        if (side > 0)
                            left_penalty += p;
                        else
                            right_penalty += p;
                    }
                }

            }

        }
    }

    // ==== density normalization ====
    double total_d = left_count + right_count + 1e-6;
    double total_p = left_penalty + right_penalty + 1e-6;

    double density_left  = left_count  / total_d;
    double density_right = right_count / total_d;
    double penalty_left  = left_penalty / total_p;
    double penalty_right = right_penalty / total_p;

    double left_value = 0.3 * density_left + 0.7 * penalty_left;
    double right_value = 0.3 * density_right + 0.7 * penalty_right;

    pair<double, double> results(left_value, right_value);

    return results;
}

std::vector<Vector2d> Planner::generateProgressiveAPF(
    Vector2d curr_pos,
    const std::vector<Obstacle>& group,
    const std::vector<Vector2d>& bypass_path, // pre-generated bypass path
    Vector2d goal,
    double max_length)
{
    std::vector<Vector2d> apf_path;
    Vector2d curr = curr_pos;
    double step_size = 0.03;
    double accumulated_dist = 0.0;

    if (bypass_path.size() < 2) return {curr};

    // --- key step: lock the bypass polarity ---
    // use the direction of the bypass path's first segment as the reference
    // make sure the path has at least two points
    Vector2d bypass_dir;
    if (bypass_path.size() >= 2) {
        int mid_idx = bypass_path.size() / 2;
        // use the vector from start to midpoint
        bypass_dir = (bypass_path[mid_idx] - bypass_path[0]).normalized();
    }
    else bypass_dir = (goal - curr_pos).normalized();

    while (accumulated_dist < max_length) {
        apf_path.push_back(curr);
        Vector2d force_rep_total(0, 0);
        for (const auto& obs : group) {
            Vector2d delta = curr - obs.center;
            double d = delta.norm() - obs.radius;

            Vector2d n = delta.normalized();

            // compute the two candidate tangents
            Vector2d t_ccw(-n.y(), n.x());
            Vector2d t_cw(n.y(), -n.x());

            // --- core fix: pick the tangent closest to the bypass direction ---
            // this keeps the APF glide direction aligned with the geometric bypass
            Vector2d t = (t_ccw.dot(bypass_dir) > t_cw.dot(bypass_dir)) ? t_ccw : t_cw;

            Vector2d f_single(0, 0);
            if (d < 0) {
                f_single = n * (1.5 / ( -d + 0.05)); // radial push
            }
            else if (d >= 0 && d < safe_r_) {
                // 2. fast-response zone (0.2 m span)

                // complete the turn at 60% of the zone: 0.2 * 0.6 = 0.12 m
                double transition_limit = safe_r_;

                if (d < transition_limit) {
                    // fast smooth turn segment (0 to 0.12 m)
                    double s = d / transition_limit;
                    double alpha = 0.5 * (1.0 - std::cos(M_PI * s)); // cosine smoothing
                    Vector2d dir = ((1.0 - alpha) * n + alpha * t).normalized();
                    f_single = dir * 2.5; // slightly stronger push for a quick start
                }
            }
            force_rep_total += f_single;
        }

        // combine with attraction (keep its gain small so the tangential guide dominates)
        Vector2d force_total = 2.0 * force_rep_total;

        Vector2d next_pos = curr + force_total.normalized() * step_size;
        accumulated_dist += (next_pos - curr).norm();
        curr = next_pos;

        double min_dist = std::numeric_limits<double>::max();
        for (const auto& obs : group) {
            double d = (curr - obs.center).norm() - obs.radius;
            if (d < min_dist) min_dist = d;
        }
        // std::cout << min_dist << std::endl;

        // exit: when the distance to the nearest obstacle exceeds the safe distance and the force is small enough
        if (min_dist > safe_r_ - 0.02) {
            break;
        }
    }
    return apf_path;
}

vector<Vector2d> Planner::plan(
    const vector<Vector2d>& path,
    vector<vector<Obstacle>>& obstacles,
    const Vector2d& start,
    const Vector2d& goal,
    const Vector2d& velocity)
{
    std::vector<Vector2d> final_path;
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        Vector2d p1 = path[i];
        Vector2d p2 = path[i + 1];

        final_path.push_back(p1);

        bool bypassed = false;

        for (size_t j = 0; j < obstacles.size(); ++j)
        {
            auto& group = obstacles[j];
            if (group.empty()) continue;
            if (!group[0].active) continue;

            double dist = 0.0;

            // ===== min distance =====
            for (size_t k = 0; k < group.size(); ++k) {
                double d = pointToSegmentDistance(group[k].center, p1, p2);
                if (k == 0 || d < dist) dist = d;
            }

            if (dist < group[0].radius + safe_r_ + detection_margin_)
            {
                auto [rho_left, rho_right] = computeSideMetric(obstacles[j], obstacles, p1, p2);
                Vector2d safe_score (rho_left, rho_right);
                bool all_obtuse = true;

                // ===== special handling at the start =====
                if ((p1 - start).norm() < 0.2)
                {
                    for (const auto& obs : group)
                    {
                        Vector2d dir = (obs.center - p1).normalized();
                        if (dir.dot((goal - p1).normalized()) < 0)
                        {
                            all_obtuse = false;
                            break;
                        }
                    }

                    if (!all_obtuse)
                    {
                        // obstacle behind -> ignore
                        for (auto& obs : group)
                            obs.active = false;
                    }
                    else
                    {
                        // ===== APF + geometric fusion =====
                        DetourPlanner detplanner;
                        std::vector<Eigen::Vector2d> bypass =
                            detplanner.bypassCluster(p1, group, start, goal, safe_score);
                        auto apf_segment = generateProgressiveAPF(p1, group, bypass, goal, 1.0);

                        if (!apf_segment.empty() && !bypass.empty())
                        {
                            int best_idx = 0;
                            double min_d = 1e6;

                            for (size_t b = 0; b < bypass.size(); ++b)
                            {
                                double d = (bypass[b] - apf_segment.back()).norm();
                                if (d < min_d)
                                {
                                    min_d = d;
                                    best_idx = b;
                                }
                            }

                            final_path.insert(final_path.end(),
                                              apf_segment.begin(), apf_segment.end());

                            vector<Vector2d> target_segment(bypass.begin() + best_idx, bypass.end());
                            if (best_idx < static_cast<int>(bypass.size())) {
                                std::vector<Vector2d> smooth_traj;
                                vector<Vector2d> control_path = buildControlPoints(target_segment, final_path, start, goal);
                                std::vector<epllipseSafeField> field = getCorridorAndOptimization(control_path, obstacles);
                                // fields.insert(fields.end(), field.begin(), field.end());
                                smooth_traj = PolyTrajectoryOptimizer::optimizePiecewiseTrajectory(control_path, field, velocity);
                                spliceTrajectory(final_path, smooth_traj);
                                // final_path.insert(final_path.end(), bypass.begin() + bypass_start_idx, bypass.end());
                            }
                        }
                    }
                }
                else
                {
                    // ===== normal bypass =====
                    DetourPlanner detplanner;
                    std::vector<Eigen::Vector2d> bypass =
                        detplanner.bypassCluster(p1, group, start, goal, safe_score);
                    vector<Vector2d> target_segment(bypass.begin(), bypass.end());
                    std::vector<Vector2d> smooth_traj;
                    vector<Vector2d> control_path = buildControlPoints(target_segment, final_path, start, goal);
                    std::vector<epllipseSafeField> field = getCorridorAndOptimization(control_path, obstacles);
                    // fields.insert(fields.end(), field.begin(), field.end());
                    smooth_traj = PolyTrajectoryOptimizer::optimizePiecewiseTrajectory(control_path, field, velocity);
                    spliceTrajectory(final_path, smooth_traj);
                    // final_path.insert(final_path.end(), bypass.begin(), bypass.end());
                }

                // ===== mark as handled =====
                for (size_t idx = i + 1; idx < path.size(); ++idx)
                {
                    double d = (path[idx] - final_path.back()).norm();
                    if (d < 0.1)
                    {
                        for (auto& obs : group)
                            obs.active = false;
                        break;
                    }
                }

                bypassed = true;
                break;
            }
        }

        // ===== jump mechanism =====
        if (bypassed)
        {
            double min_dist = std::numeric_limits<double>::max();

            while (i + 1 < path.size())
            {
                double d = (path[i + 1] - final_path.back()).norm();

                if (d < 0.1)
                {
                    if (d > min_dist)
                        break;
                }

                min_dist = d;
                ++i;
            }
        }
    }

    final_path.push_back(path.back());

    return getFinalSmoothPath(final_path);
    // return final_path;
}
