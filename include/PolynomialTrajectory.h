#ifndef POLY_OPTIMIZER_H
#define POLY_OPTIMIZER_H

#include <Eigen/Dense>
#include <vector>
#include <ceres/ceres.h>
#include "../include/safeField.h"

using Eigen::Vector2d;

class PolyTrajectoryOptimizer {
public:

    // =========================
    // Main interface
    // =========================
    static std::vector<Vector2d> optimizePiecewiseTrajectory(
        const std::vector<Vector2d>& control_path,
        const std::vector<epllipseSafeField>& corridors,
        const Eigen::Vector2d& velocity);

    // =========================
    // Polynomial evaluation (used downstream)
    // =========================
    static Vector2d evalPoly(
        const double* ax,
        const double* ay,
        double t);

    static Vector2d evalVel(
        const double* ax,
        const double* ay,
        double t);

    static Vector2d evalAcc(
        const double* ax,
        const double* ay,
        double t);

private:

    // =========================
    // Piecewise optimization (core)
    // =========================
    static std::vector<Vector2d> optimizeSingleSegment(
        const Vector2d& p0,
        const Vector2d& p1,
        const Vector2d& v0, // start velocity/tangent
        const Vector2d& v1, // end velocity/tangent
        const std::vector<epllipseSafeField>& corridors);

    // =========================
    // Path sampling
    // =========================
    static std::vector<Vector2d> samplePath(
        const std::vector<Vector2d>& path,
        const std::vector<epllipseSafeField>& corridors);

    // =========================
    // Build the full trajectory
    // =========================
    static std::vector<Vector2d> buildPiecewiseTrajectory(
        const std::vector<Vector2d>& samples,
        const std::vector<epllipseSafeField>& corridors);
};

#endif