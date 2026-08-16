#include "../include/PolynomialTrajectory.h"

using namespace std;

// =========================
// polynomial evaluation
// =========================
Vector2d PolyTrajectoryOptimizer::evalPoly(
    const double* ax,
    const double* ay,
    double t)
{
    double tt = 1.0;
    double x = 0, y = 0;

    for (int i = 0; i < 5; i++)
    {
        x += ax[i] * tt;
        y += ay[i] * tt;
        tt *= t;
    }

    return {x, y};
}

Vector2d PolyTrajectoryOptimizer::evalVel(
    const double* ax,
    const double* ay,
    double t)
{
    double tt = 1.0;
    double x = 0, y = 0;

    for (int i = 1; i < 5; i++)
    {
        x += i * ax[i] * tt;
        y += i * ay[i] * tt;
        tt *= t;
    }

    return {x, y};
}

// =========================
// ellipse constraint (soft barrier)
// =========================
struct CorridorResidual
{
    CorridorResidual(const std::vector<epllipseSafeField>& es, double t)
        : es_(es), t_(t) {}

    template <typename T>
    bool operator()(const T* ax, const T* ay, T* residual) const
    {
        // ========= 1. compute trajectory points =========
        T x = T(0), y = T(0), tt = T(1);

        for (int i = 0; i < 5; i++)
        {
            x += ax[i] * tt;
            y += ay[i] * tt;
            tt *= T(t_);
        }

        // ========= 2. pick the best ellipse (key change) =========
        T best_penalty = T(1e9);

        for (const auto& e : es_)
        {
            // rotate
            T cos_t = ceres::cos(T(e.theta));
            T sin_t = ceres::sin(T(e.theta));

            T dx = x - T(e.center.x());
            T dy = y - T(e.center.y());

            T ex = cos_t * dx + sin_t * dy;
            T ey = -sin_t * dx + cos_t * dy;

            // ========= avoid division by zero =========
            T a = T(e.a);
            T b = T(e.b);
            T eps = T(1e-3);

            T val = (ex * ex) / (a * a + eps)
                  + (ey * ey) / (b * b + eps);

            // ========= hinge =========
            T d = val - T(1.0);

            // 👉 softplus
            T k = T(5.0);
            T kd = k * d;

            if (kd > T(50.0)) kd = T(50.0);
            if (kd < T(-50.0)) kd = T(-50.0);

            T smooth = ceres::log(T(1.0) + ceres::exp(kd)) / k;

            T cur = smooth * smooth;

            // key: take the minimum, not the sum
            if (cur < best_penalty)
                best_penalty = cur;
        }

        // ========= 3. output =========
        if (best_penalty > T(100.0))
            best_penalty = T(100.0);

        residual[0] = T(20.0) * best_penalty;

        return true;
    }

    std::vector<epllipseSafeField> es_;
    double t_;
};

// =========================
// strong start/end constraints
// =========================
struct BoundaryResidual {
    enum Type { POSITION, VELOCITY };

    // target: goal position or goal velocity vector
    // t: time point (0.0 or 1.0)
    // type: constraint type
    BoundaryResidual(Vector2d target, double t, Type type)
        : target_(target), t_(t), type_(type) {}

    template <typename T>
    bool operator()(const T* ax, const T* ay, T* residual) const {
        T t = T(t_);
        T val_x, val_y;

        if (type_ == POSITION) {
            // position P(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4
            val_x = ax[0] + ax[1]*t + ax[2]*t*t + ax[3]*t*t*t + ax[4]*t*t*t*t;
            val_y = ay[0] + ay[1]*t + ay[2]*t*t + ay[3]*t*t*t + ay[4]*t*t*t*t;
        } else {
            // velocity V(t) = P'(t) = a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3
            val_x = ax[1] + T(2)*ax[2]*t + T(3)*ax[3]*t*t + T(4)*ax[4]*t*t*t;
            val_y = ay[1] + T(2)*ay[2]*t + T(3)*ay[3]*t*t + T(4)*ay[4]*t*t*t;
        }

        // weights: position is a hard metric (high), velocity guides the direction (lower)
        T weight = (type_ == POSITION) ? T(100.0) : T(50.0);

        residual[0] = weight * (val_x - T(target_.x()));
        residual[1] = weight * (val_y - T(target_.y()));

        return true;
    }

    Vector2d target_;
    double t_;
    Type type_;
};

// =========================
// smoothing (acceleration)
// =========================
struct SmoothResidual
{
    // add a weight parameter, default 1.0
    SmoothResidual(double t, double weight = 1.0) : t_(t), weight_(weight) {}

    template <typename T>
    bool operator()(const T* ax, const T* ay, T* residual) const
    {
        T tt = T(t_);

        // P(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4
        // P''(t) = 2*ax[2] + 6*ax[3]*t + 12*ax[4]*t^2
        T ddx = T(2.0) * ax[2] + T(6.0) * ax[3] * tt + T(12.0) * ax[4] * tt * tt;
        T ddy = T(2.0) * ay[2] + T(6.0) * ay[3] * tt + T(12.0) * ay[4] * tt * tt;

        // apply the weight coefficient
        residual[0] = T(weight_) * ddx;
        residual[1] = T(weight_) * ddy;

        return true;
    }

    double t_;
    double weight_;
};

// =========================
// single-segment optimization (core)
// =========================
vector<Vector2d> PolyTrajectoryOptimizer::optimizeSingleSegment(
    const Vector2d& p0,
    const Vector2d& p1,
    const Vector2d& v0, // start velocity/tangent
    const Vector2d& v1, // end velocity/tangent
    const vector<epllipseSafeField>& corridors)
{
    // 4th-order polynomial coefficients ax[0...4]
    double ax[5], ay[5];

    // ========= 1. improved initialization (from P0 and V0) =========
    // ax[0] is position, ax[1] is velocity; gives Ceres a more guided initial value
    ax[0] = p0.x();
    ax[1] = v0.x();
    ax[2] = ax[3] = ax[4] = 0.0;

    ay[0] = p0.y();
    ay[1] = v0.y();
    ay[2] = ay[3] = ay[4] = 0.0;

    ceres::Problem problem;

    // ========= pick the local corridor (key) =========
    auto isInside = [](const Vector2d& p, const epllipseSafeField& e)
    {
        double c = cos(e.theta);
        double s = sin(e.theta);

        Vector2d d = p - e.center;

        double x = c * d.x() + s * d.y();
        double y = -s * d.x() + c * d.y();

        return (x*x)/(e.a*e.a) + (y*y)/(e.b*e.b) <= 1.0;
    };

    std::vector<epllipseSafeField> local_corridors;

    // find a circle containing p0 or p1
    for (const auto& e : corridors)
    {
        if (isInside(p0, e) || isInside(p1, e))
            local_corridors.push_back(e);
    }

    // ========= fallback (mandatory) =========
    if (local_corridors.empty())
    {
        double best0 = 1e9, best1 = 1e9;
        epllipseSafeField e0, e1;

        for (const auto& e : corridors)
        {
            double d0 = (p0 - e.center).norm();
            double d1 = (p1 - e.center).norm();

            if (d0 < best0) { best0 = d0; e0 = e; }
            if (d1 < best1) { best1 = d1; e1 = e; }
        }

        local_corridors.push_back(e0);
        local_corridors.push_back(e1);
    }

    // ========= 2. boundary constraints (mixed P+V) =========
    // start constraint (t=0)
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<BoundaryResidual, 2, 5, 5>(
        new BoundaryResidual(p0, 0.0, BoundaryResidual::POSITION)), nullptr, ax, ay);
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<BoundaryResidual, 2, 5, 5>(
        new BoundaryResidual(v0, 0.0, BoundaryResidual::VELOCITY)), nullptr, ax, ay);

    // end constraint (t=1.0)
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<BoundaryResidual, 2, 5, 5>(
        new BoundaryResidual(p1, 1.0, BoundaryResidual::POSITION)), nullptr, ax, ay);
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<BoundaryResidual, 2, 5, 5>(
        new BoundaryResidual(v1, 1.0, BoundaryResidual::VELOCITY)), nullptr, ax, ay);

    // ========= 3. ellipse-corridor constraints =========
    for (double t = 0.1; t < 0.9; t += 0.05) // denser sampling for safety
    {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<CorridorResidual, 1, 5, 5>(
                new CorridorResidual(local_corridors, t)),
            nullptr, ax, ay);
    }

    // ========= 4. smoothing residual (minimize acceleration) =========
    for (double t = 0.0; t <= 1.0; t += 0.1)
    {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<SmoothResidual, 2, 5, 5>(
                new SmoothResidual(t, 1.0)), // weight set to 1.0
            nullptr, ax, ay);
    }

    // ========= 5. solver configuration =========
    ceres::Solver::Options options;
    options.max_num_iterations = 20; // 30 is too few; raise to 100 to guarantee convergence
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // ========= 6. high-density sampling output =========
    vector<Vector2d> traj;
    int num_samples = 20; // raised from 10 to 30 samples to remove the visual "straight-line" look
    for (int i = 0; i < num_samples; i++)
    {
        double t = double(i) / double(num_samples - 1);
        traj.push_back(evalPoly(ax, ay, t));
    }

    return traj;
}

std::vector<Vector2d> PolyTrajectoryOptimizer::samplePath(
    const std::vector<Vector2d>& path,
    const std::vector<epllipseSafeField>& corridors)
{
    std::vector<Vector2d> samples;
    if (path.empty()) return samples;

    Vector2d start = path.front();
    Vector2d goal  = path.back();

    samples.push_back(start);

    // =========================================================
    // 1️⃣ isInside
    // =========================================================
    auto isInside = [](const Vector2d& p, const epllipseSafeField& e)
    {
        double c = cos(e.theta);
        double s = sin(e.theta);

        Vector2d d = p - e.center;

        double x = c * d.x() + s * d.y();
        double y = -s * d.x() + c * d.y();

        return (x*x)/(e.a*e.a) + (y*y)/(e.b*e.b) <= 1.0;
    };

    // =========================================================
    // 2️⃣ overlap
    // =========================================================
    auto overlapRatio = [&](const epllipseSafeField& A,
                            const epllipseSafeField& B)
    {
        int N = 20;
        int inside = 0;

        for (int i = 0; i < N; i++)
        {
            double ang = 2 * M_PI * i / N;

            double x = A.a * cos(ang);
            double y = A.b * sin(ang);

            double c = cos(A.theta);
            double s = sin(A.theta);

            Vector2d pt;
            pt.x() = A.center.x() + c * x - s * y;
            pt.y() = A.center.y() + s * x + c * y;

            if (isInside(pt, B))
                inside++;
        }

        return (double)inside / N;
    };

    // =========================================================
    // params
    // =========================================================
    double overlap_thresh = 0.85;
    double near_thresh = 0.3;
    double min_dist = 0.1;

    // new: leading/trailing filter params
    Vector2d dir = (goal - start).normalized();
    double total_len = (goal - start).norm();
    double front = 0.1 * total_len;
    double back  = 0.9 * total_len;

    // =========================================================
    // 3. drop points near start/goal + leading/trailing filter
    // =========================================================
    std::vector<epllipseSafeField> filtered1;

    for (const auto& c : corridors)
    {
        // leading/trailing filter (new)
        double proj = (c.center - start).dot(dir);
        if (proj < front || proj > back)
            continue;

        // original logic
        double d_start = (c.center - start).norm();
        double d_goal  = (c.center - goal).norm();

        if (d_start < near_thresh || d_goal < near_thresh)
            continue;

        filtered1.push_back(c);
    }

    // =========================================================
    // 4. overlap dedup (preserve order)
    // =========================================================
    std::vector<epllipseSafeField> filtered2;

    for (const auto& c : filtered1)
    {
        bool redundant = false;

        for (const auto& f : filtered2)
        {
            double r1 = overlapRatio(c, f);
            double r2 = overlapRatio(f, c);

            if (std::max(r1, r2) > overlap_thresh)
            {
                redundant = true;
                break;
            }
        }

        if (!redundant)
            filtered2.push_back(c);
    }

    // =========================================================
    // 5. no sorting, just advance
    // =========================================================
    Vector2d last = start;

    for (const auto& c : filtered2)
    {
        Vector2d v = c.center - last;

        Vector2d local_dir = (goal - last).normalized();
        if (v.dot(local_dir) < 0)
            continue;

        if (v.norm() < min_dist)
            continue;

        samples.push_back(c.center);
        last = c.center;
    }

    // =========================================================
    // 6. ensure the end point
    // =========================================================
    if ((goal - samples.back()).norm() > min_dist)
        samples.push_back(goal);
    else
        samples.back() = goal;

    return samples;
}

// =========================
// main entry
// =========================
vector<Vector2d> PolyTrajectoryOptimizer::optimizePiecewiseTrajectory(
    const vector<Vector2d>& control_path,
    const vector<epllipseSafeField>& corridors,
    const Eigen::Vector2d& velocity)
{
    vector<Vector2d> samples = samplePath(control_path, corridors);

    vector<Vector2d> full_traj;

    for (size_t i = 0; i + 1 < samples.size(); i++)
    {
        // std::cout << samples.size() << std::endl;
        Vector2d p0 = samples[i];
        Vector2d p1 = samples[i + 1];

        // compute the start velocity v0:
        // first segment: use its own direction; otherwise average the previous and current segments
        // compute start velocity v0
        Vector2d v0;
        if (i != 0) {
            Vector2d diff = p1 - p0;
            double dist = diff.norm();
            // speed limit: if the distance is below 1.5, use the distance as the speed magnitude
            v0 = diff.normalized() * std::min(0.5, dist);
        } else {
            Vector2d diff = samples[i+1] - samples[0];
            double dist = diff.norm();
            v0 = velocity.normalized() * std::min(0.5, dist);
        }

        // compute end velocity v1
        Vector2d v1;
        if (i + 2 != samples.size()) {
            Vector2d diff = samples[i+2] - p1;
            double dist = diff.norm();
            v1 = diff.normalized() * std::min(0.5, dist);
        } else {
            Vector2d diff = samples[i+1] - samples[0];
            double dist = diff.norm();
            v1 = diff.normalized() * std::min(0.5, dist);
        }
        auto seg = optimizeSingleSegment(p0, p1, v0, v1, corridors);
        full_traj.insert(full_traj.end(), seg.begin(), seg.end());
    }
    return full_traj;
}
