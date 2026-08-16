#pragma once

#include <mutex>
#include <vector>

#include "common.h"

// Dynamic cylinder obstacle (the "real" object in the simulated world)
struct Cylinder
{
    Eigen::Vector2d center;
    double radius = 0.0;
    double height = 0.0;
    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
    double speed = 0.0;

    // dense point cloud in world coordinates (visualization only, translated with the center)
    std::vector<Eigen::Vector3d> points;
    // offsets relative to the center (generated once, only translated afterwards)
    std::vector<Eigen::Vector3d> local_offsets;
};

// Simulation world parameters (overridable via YAML / CLI)
struct WorldConfig
{
    int num_cylinders = 30;   // number of dynamic obstacles (cylinders)
    double xmin = 0.0;        // world bounds (m)
    double xmax = 20.0;
    double ymin = 0.0;
    double ymax = 15.0;
    double radius_min = 0.7;  // obstacle radius range (m)
    double radius_max = 1.0;
    double height_min = 1.0;  // obstacle height range (m)
    double height_max = 2.5;
    double speed_min = 0.4;   // obstacle speed range (m/s)
    double speed_max = 1.0;
    double clearance = 1.0;   // min spacing between obstacles (m)
};

// Dynamic obstacle simulation world
class World
{
public:
    explicit World(const WorldConfig& cfg = WorldConfig());

    // advance the simulation (main thread only); robot_pos is used for obstacle-robot avoidance
    void step(double dt, const Eigen::Vector2d& robot_pos = Eigen::Vector2d(1e9, 1e9));

    double time() const { return t_; }

    const WorldConfig& config() const { return cfg_; }

    // for main-thread visualization (no lock)
    const std::vector<Cylinder>& cylinders() const { return cylinders_; }

    // for main-thread collision checks (no lock)
    ObstacleMap circleObstacles(double inflate = 0.1) const;

    // cross-thread snapshot (for the perception thread; locked)
    ObstacleMap snapshotCircleObstacles(double inflate = 0.1);

    void setSeed(unsigned seed) { seed_ = seed; }

private:
    void buildCylinders(int num);
    void updatePoints();
    void separateAndBounce(double dt, const Eigen::Vector2d& robot_pos);

    WorldConfig cfg_;
    std::vector<Cylinder> cylinders_;
    double t_ = 0.0;
    unsigned seed_ = 42;
    mutable std::mutex mutex_;
};
