//
// Created by hp on 2026/4/15.
#include "../include/safeField.h"
#include <cmath>
#include <algorithm>

bool isInsideEllipse(const Vector2d& pt, const epllipseSafeField& e)
{
    double cos_t = cos(e.theta);
    double sin_t = sin(e.theta);

    Vector2d d = pt - e.center;

    double x =  cos_t * d.x() + sin_t * d.y();
    double y = -sin_t * d.x() + cos_t * d.y();

    double val = (x * x) / (e.a * e.a) +
                 (y * y) / (e.b * e.b);

    return val < 1.0;
}

std::vector<epllipseSafeField> removeTinyEllipses(
    const std::vector<epllipseSafeField>& corridors,
    double min_axis = 0.25,        // min semi-axis
    double min_area = 0.08,        // min area
    double min_progress = 0.15     // min progress distance
)
{
    std::vector<epllipseSafeField> result;

    if (corridors.empty()) return result;

    result.push_back(corridors[0]);

    for (size_t i = 1; i < corridors.size(); i++)
    {
        const auto& e = corridors[i];

        // ========= 1. semi-axis filter =========
        if (e.a < min_axis || e.b < min_axis)
            continue;

        // ========= 2. area filter =========
        double area = M_PI * e.a * e.b;
        if (area < min_area)
            continue;

        // ========= 3. progress-distance filter (key) =========
        const auto& last = result.back();
        double progress = (e.center - last.center).norm();

        if (progress < min_progress)
            continue;

        result.push_back(e);
    }

    return result;
}

void refineEllipseWithObstacleRepulsion(
    epllipseSafeField& ellipse,
    const std::vector<std::vector<Obstacle>>& obstacles,
    double min_r)
{
    Vector2d center = ellipse.center;
    double a = ellipse.a;
    double b = ellipse.b;
    double theta = ellipse.theta;

    double cos_t = cos(theta);
    double sin_t = sin(theta);

    // =========================================================
    // 1. find the nearest obstacle
    // =========================================================
    Vector2d nearest_obs;
    double min_dist = 1e9;
    bool found = false;

    for (const auto& group : obstacles)
    {
        for (const auto& obs : group)
        {
            if (!obs.active) continue;

            double d = (center - obs.center).norm() - obs.radius;

            if (d < min_dist)
            {
                min_dist = d;
                nearest_obs = obs.center;
                found = true;
            }
        }
    }

    if (!found) return;

    // =========================================================
    // 2. direction: away from the nearest obstacle
    // =========================================================
    Vector2d dir = center - nearest_obs;
    if (dir.norm() < 1e-6) return;
    dir.normalize();

    // =========================================================
    // 3. collision check
    // =========================================================
    auto checkCollision = [&](const Vector2d& c, double aa, double bb)
    {
        for (const auto& group : obstacles)
        {
            for (const auto& obs : group)
            {
                if (!obs.active) continue;

                Vector2d d = c - obs.center;

                double x =  cos_t * d.x() + sin_t * d.y();
                double y = -sin_t * d.x() + cos_t * d.y();

                double ax = aa + obs.radius;
                double by = bb + obs.radius;

                double val = (x*x)/(ax*ax) + (y*y)/(by*by);

                if (val < 1.0)
                    return true;
            }
        }
        return false;
    };

    // =========================================================
    // 4. move along the direction (core: moving = inflating)
    // =========================================================
    Vector2d best_center = center;
    double best_a = a;
    double best_b = b;

    for (double step = 0.0; step <= 0.5; step += 0.1)
    {
        Vector2d new_center = center + step * dir;

        // key rule: how far you move -> how much you inflate (fully coupled)
        double new_a = a + step;
        double new_b = b + step;

        // limits
        new_a = std::max(min_r, new_a);
        new_b = std::max(min_r, new_b);

        // collision handling
        if (checkCollision(new_center, new_a, new_b))
        {
            // slight retreat (keeps the rule consistent)
            new_a = a + (step - 0.1);
            new_b = b + (step - 0.1);

            if (checkCollision(new_center, new_a, new_b))
                break;
        }

        best_center = new_center;
        best_a = new_a;
        best_b = new_b;
    }

    // =========================================================
    // 5. write back
    // =========================================================
    ellipse.center = best_center;
    ellipse.a = best_a;
    ellipse.b = best_b;
}

std::vector<epllipseSafeField> pruneCorridors(
    const std::vector<epllipseSafeField>& corridors,
    double overlap_thresh = 0.85)
{
    if (corridors.empty()) return {};

    std::vector<epllipseSafeField> result;

    auto isInside = [](const Vector2d& p, const epllipseSafeField& e)
    {
        double cos_t = cos(e.theta);
        double sin_t = sin(e.theta);

        Vector2d d = p - e.center;

        double x =  cos_t * d.x() + sin_t * d.y();
        double y = -sin_t * d.x() + cos_t * d.y();

        double val = (x*x)/(e.a*e.a) + (y*y)/(e.b*e.b);

        return val <= 1.0;
    };

    auto overlapRatio = [&](const epllipseSafeField& A,
                            const epllipseSafeField& B)
    {
        int N = 20;
        int inside = 0;

        for (int i = 0; i < N; i++)
        {
            double ang = 2 * M_PI * i / N;

            Vector2d p(
                A.center.x() + A.a * cos(ang),
                A.center.y() + A.b * sin(ang)
            );

            double c = cos(A.theta);
            double s = sin(A.theta);

            Vector2d pt;
            pt.x() = A.center.x() + c * (p.x() - A.center.x()) - s * (p.y() - A.center.y());
            pt.y() = A.center.y() + s * (p.x() - A.center.x()) + c * (p.y() - A.center.y());

            if (isInside(pt, B))
                inside++;
        }

        return (double)inside / N;
    };

    // =========================================================
    // 1️⃣ greedy pruning
    // =========================================================
    for (const auto& e : corridors)
    {
        bool redundant = false;

        for (const auto& kept : result)
        {
            double r1 = overlapRatio(e, kept);
            double r2 = overlapRatio(kept, e);

            double overlap = std::max(r1, r2);

            if (overlap > overlap_thresh)
            {
                redundant = true;
                break;
            }
        }

        if (!redundant)
            result.push_back(e);
    }

    return result;
}

std::vector<epllipseSafeField> getCorridorAndOptimization(
    const std::vector<Vector2d>& path,
    const std::vector<std::vector<Obstacle>>& obstacles)
{
    double safe_r = 0.3;
    std::vector<epllipseSafeField> corridors;

    if (path.size() < 2) return corridors;

    size_t i = 0;

    const double min_r = 0.2;   // min semi-axis (prevents degeneracy)

    while (i < path.size() - 1)
    {
        Vector2d p0 = path[i];
        Vector2d p1 = path[i + 1];

        // ========= 1. init =========
        Vector2d center = 0.5 * (p0 + p1);
        Vector2d dir = (p1 - p0).normalized();
        double theta = atan2(dir.y(), dir.x());
        double cos_t = cos(theta);
        double sin_t = sin(theta);

        // ========= 3. resolve the max feasible ellipse =========

        double a = safe_r;
        double b = safe_r;

        // ========= 3. shrink (guarantee feasibility) =========
        for (int iter = 0; iter < 20; iter++)
        {
            bool collided = false;

            for (const auto& group : obstacles)
            {
                for (const auto& obs : group)
                {
                    if (!obs.active) continue;

                    Vector2d d = obs.center - center;

                    double x =  cos_t * d.x() + sin_t * d.y();
                    double y = -sin_t * d.x() + cos_t * d.y();

                    double ax = a + obs.radius;
                    double by = b + obs.radius;

                    double val = (x * x) / (ax * ax) +
                                 (y * y) / (by * by);

                    if (val < 1.0)
                    {
                        collided = true;
                        break;
                    }
                }
                if (collided) break;
            }

            if (!collided) break;

            // shrink strategy (stable)
            a *= 0.85;
            b *= 0.85;

            if (a < 0.1 || b < 0.1)
                break;
        }

        // clamp to bounds
        a = std::max(min_r, std::min(a, safe_r));
        b = std::max(min_r, std::min(b, safe_r));

        // ========= 4. save =========
        epllipseSafeField field;
        field.a = a;
        field.b = b;
        field.center = center;
        field.theta = theta;

        corridors.push_back(field);

        // ========= 5. advance along the ellipse edge =========
        // walk along the path direction to near the ellipse boundary

        std::vector<Vector2d> candidates;

        int sample_num = 30;

        // ========= 1. sample the ellipse boundary =========
        for (int k = 0; k < sample_num; k++)
        {
            double ang = 2 * M_PI * k / sample_num;

            double x = field.a * cos(ang);
            double y = field.b * sin(ang);

            double cos_t = cos(field.theta);
            double sin_t = sin(field.theta);

            Vector2d pt;
            pt.x() = field.center.x() + cos_t * x - sin_t * y;
            pt.y() = field.center.y() + sin_t * x + cos_t * y;

            candidates.push_back(pt);
        }

        // ========= 2. filter: must not be inside an existing ellipse =========
        std::vector<Vector2d> valid_pts;

        for (const auto& pt : candidates)
        {
            bool inside_any = false;

            for (const auto& old_e : corridors)
            {
                if (isInsideEllipse(pt, old_e))
                {
                    inside_any = true;
                    break;
                }
            }

            if (!inside_any)
                valid_pts.push_back(pt);
        }

        // ========= 3. pick the best point (along the path direction) =========
        Vector2d best_pt = p1;
        double best_score = -1e9;

        for (const auto& pt : valid_pts)
        {
            Vector2d v = (pt - p0).normalized();

            double score = v.dot(dir);  // closer to the path direction is better

            if (score > best_score)
            {
                best_score = score;
                best_pt = pt;
            }
        }

        // ========= 4. fallback (in case no point is found) =========
        if (valid_pts.empty())
        {
            best_pt = p0 + 0.5 * field.a * dir;
        }

        // ========= 5. map back to path index =========
        double min_dist = 1e9;
        size_t next_i = i + 1;

        for (size_t j = i + 1; j < path.size(); j++)
        {
            double d = (path[j] - best_pt).norm();
            if (d < min_dist)
            {
                min_dist = d;
                next_i = j;
            }
        }

        if (next_i <= i) next_i = i + 1;

        i = next_i;
    }

    for (size_t i = 0; i < corridors.size(); i++)
    {
        refineEllipseWithObstacleRepulsion(
            corridors[i],
            obstacles,
            0.05
        );
    }
    //
    auto pruned = pruneCorridors(corridors, 0.4);
    auto filtered = removeTinyEllipses(pruned, 0.2, 0.1, 0.1);

    return pruned;
}
