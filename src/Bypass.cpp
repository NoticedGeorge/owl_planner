//
// Created by georg on 2026/4/18.
//
#include "../include/Bypass.h"

namespace {

double distance(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    return (a - b).norm();
}

double cross(const Eigen::Vector2d& O, const Eigen::Vector2d& A, const Eigen::Vector2d& B) {
    return (A.x() - O.x()) * (B.y() - O.y()) - (A.y() - O.y()) * (B.x() - O.x());
}

} // namespace

std::vector<Eigen::Vector2d> DetourPlanner::sampleLine(const Eigen::Vector2d& a, const Eigen::Vector2d& b, double step) {
    std::vector<Eigen::Vector2d> sampled;
    double len = distance(a, b);
    int n = std::max(1, int(std::ceil(len / step)));
    for (int i = 0; i <= n; ++i) {
        double t = double(i) / n;
        sampled.push_back(a + t * (b - a));
    }
    return sampled;
}


std::vector<Eigen::Vector2d> DetourPlanner::extractClusterContourStep(
    const std::vector<Obstacle>& cluster,
    double step,
    int num_points_per_circle) // safety expansion distance
{
    double offset = safe_r;
    std::vector<Eigen::Vector2d> points;

    // 1. discretize each circle and expand the radius by offset
    for (const auto& c : cluster) {
        double r = c.radius + offset;
        for (int i = 0; i < num_points_per_circle; ++i) {
            double theta = 2.0 * M_PI * i / num_points_per_circle;
            points.push_back(Eigen::Vector2d(
                c.center.x() + r * std::cos(theta),
                c.center.y() + r * std::sin(theta)
            ));
        }
    }

    // 2. Graham Scan to build the convex hull
    std::sort(points.begin(), points.end(), [](const Eigen::Vector2d& a, const Eigen::Vector2d& b){
        return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
    });

    std::vector<Eigen::Vector2d> hull;

    for (const auto& p : points) {
        while (hull.size() >= 2 && cross(hull[hull.size()-2], hull[hull.size()-1], p) <= 0)
            hull.pop_back();
        hull.push_back(p);
    }

    size_t t = hull.size() + 1;
    for (auto it = points.rbegin(); it != points.rend(); ++it) {
        while (hull.size() >= t && cross(hull[hull.size()-2], hull[hull.size()-1], *it) <= 0)
            hull.pop_back();
        hull.push_back(*it);
    }

    hull.pop_back(); // drop duplicate points

    // 3. sample along the hull edge by step
    std::vector<Eigen::Vector2d> contour;
    for (size_t i = 0; i < hull.size(); ++i) {
        const Eigen::Vector2d& a = hull[i];
        const Eigen::Vector2d& b = hull[(i + 1) % hull.size()];
        std::vector<Eigen::Vector2d> seg = sampleLine(a, b, step);
        contour.insert(contour.end(), seg.begin(), seg.end());
    }

    return contour;
}

Eigen::Vector2d DetourPlanner::computeScore(std::vector<Vector2d>& l_path, std::vector<Vector2d>& r_path, Eigen::Vector2d safe_score, Eigen::Vector2d velocity, Eigen::Vector2d start, Eigen::Vector2d goal) {
    double l_v_gain = 0, r_v_gain = 0;
    Eigen::Vector2d dir = (goal - start).normalized();
    Eigen::Vector2d normal_left(-dir.y(), dir.x());
    if (velocity.norm() > 1e-6) {
        bool ifLeft = false; //left is false, right is true
        double cross = normal_left.dot(velocity);

        if (cross > 1e-6)
            ifLeft = true;   // left
        else if (cross < -1e-6)
            ifLeft = false;  // right

        if (ifLeft == true) r_v_gain = -0.2;
        else l_v_gain = -0.2;
    }

    double l_p_gain = 0, r_p_gain = 0;
    double total = l_path.size() + r_path.size();
    l_p_gain = l_path.size() / total;
    r_p_gain = r_path.size() / total;

    double final_gain_l = l_v_gain + 0.5 * l_p_gain + 0.5 * safe_score[0];
    double final_gain_r = r_v_gain + 0.5 * r_p_gain + 0.5 * safe_score[1];

    return Eigen::Vector2d(final_gain_l, final_gain_r);
}

bool DetourPlanner::isBackToLine(
    const Eigen::Vector2d& pt,
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& goal,
    double threshold) // distance threshold 0.08 m
{
    Eigen::Vector2d line_vec = goal - start;
    double line_len = line_vec.norm();

    // if start and end coincide, just measure the distance to the goal
    if (line_len < 1e-6) {
        return (goal - pt).norm() < threshold;
    }

    // perpendicular distance from the point to the line (cross-track distance)
    // 2D cross-product: |x1*y2 - y1*x2| / L
    Eigen::Vector2d pt_vec = pt - start;
    double cross_product = pt_vec.x() * line_vec.y() - pt_vec.y() * line_vec.x();
    double distance = std::abs(cross_product) / line_len;

    return distance < threshold;
}

void DetourPlanner::organizePath(std::vector<Vector2d>& path, const Eigen::Vector2d& start, const Eigen::Vector2d& goal) {
    if (path.empty()) return;

    // 1. make sure start is considered
    // if start is not in path, add it first; the sort will place it at index 0
    // path.push_back(start);

    Eigen::Vector2d currentPos = start;
    const size_t n = path.size();

    for (size_t i = 0; i < n; ++i) {
        size_t bestIdx = i;
        double minScore = std::numeric_limits<double>::max();

        // 2. the search range is [i, n-1]
        // via swapping, points before i are already-sorted "fixed" points
        // points after i are the candidate pool; swapping never loses a point
        for (size_t j = i; j < n; ++j) {
            // use squaredNorm to avoid sqrt CPU cost
            double distToCurrentSq = (path[j] - currentPos).squaredNorm();

            // tip: keep a small heuristic weight to bias the path toward the goal
            // for pure nearest-neighbor, just use distToCurrentSq
            double score = distToCurrentSq + 0.001 * (path[j] - goal).squaredNorm();

            if (score < minScore) {
                minScore = score;
                bestIdx = j;
            }
        }

        // 3. swap in place: move the chosen nearest point to position i
        // so data before path[i] is untouched, and path[bestIdx] receives the displaced old data
        if (bestIdx != i) {
            std::swap(path[i], path[bestIdx]);
        }

        // update the current reference point to the selected point
        currentPos = path[i];
    }
}

std::vector<Eigen::Vector2d> DetourPlanner::bypassCluster(
    const Eigen::Vector2d& hit_point,
    const std::vector<Obstacle>& group,
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& goal,
    const Eigen::Vector2d& safe_score,
    double step) {
    // extract the whole obstacle-cluster contour with step length
    Eigen::Vector2d velocity = group[0].velocity;
    auto contour = extractClusterContourStep(group, step);

    // split the contour into left/right paths (perpendicular to the start-goal line)
    Eigen::Vector2d dir = (goal - start).normalized();
    Eigen::Vector2d perp(-dir.y(), dir.x()); // perpendicular direction

    std::vector<Eigen::Vector2d> left_path, right_path;
    for (auto& pt : contour) {
        Eigen::Vector2d rel = pt - hit_point;
        if (rel.dot(perp) >= 0) left_path.push_back(pt);
        else right_path.push_back(pt);
    }
    organizePath(left_path, start, goal);
    organizePath(right_path, start, goal);

    // ==== scan both paths to find where they return to the straight line ====
    auto findReturnPath = [&](const std::vector<Eigen::Vector2d>& path) {
        std::vector<Eigen::Vector2d> ret;
        for (size_t i = 0; i < path.size(); ++i) {
            ret.push_back(path[i]);
            if (i > 20 && isBackToLine(path[i], start, goal)) {
                // std::cout << "yes" << std::endl;
                break;
            }
        }
        return ret;
    };
    //length score
    std::vector<Eigen::Vector2d> left_result = findReturnPath(left_path);
    std::vector<Eigen::Vector2d> right_result = findReturnPath(right_path);

    Eigen::Vector2d score = computeScore(left_result, right_result, safe_score, velocity, start, goal);

    // ==== decision: return the shorter path or a random one ====
    if (left_result.empty()) return right_result;
    if (right_result.empty()) return left_result;
    return (score[0] < score[1]) ? left_result : right_result;
}
