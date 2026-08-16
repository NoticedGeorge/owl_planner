#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "common.h"
#include "World.h"

// Snapshot for one rendered frame (assembled on the main thread)
struct RenderState
{
    const std::vector<Cylinder>* cylinders = nullptr;
    const std::vector<Eigen::Vector2d>* path = nullptr;
    const std::vector<Eigen::Vector2d>* executed = nullptr;
    const std::vector<Eigen::Vector2d>* waypoints = nullptr; // queued user waypoints (the current goal is the GOAL marker)
    RobotState robot;
    Eigen::Vector2d goal;
    bool collision = false;
    bool reached = false;
    bool replan_pending = false;
    double sim_time = 0.0;
    double window_len_m = 0.0;
    PipelineMetrics metrics;
};

// Dynamic visualization: dark theme + gradient path + speed coloring + pulse animation + metrics HUD
// Interaction: middle-click (without dragging) sets a user waypoint anywhere inside the
// pick/work range (by default the rectangle spanned by robot start and goal)
class Visualizer
{
public:
    explicit Visualizer(const WorldConfig& world_cfg = WorldConfig());
    ~Visualizer() = default;

    bool wasStopped() const { return viewer_->wasStopped(); }
    void render(const RenderState& state);

    // Set the rectangle (m) in which user waypoints are allowed.
    // Defaults to the obstacle world bounds if not called.
    void setPickRange(double xmin, double xmax, double ymin, double ymax);

    // Take picked waypoints (consumed on the main thread; thread-safe)
    std::vector<Eigen::Vector2d> takePickedWaypoints();

private:
    void buildScene();
    void addPickedWaypoint(const Eigen::Vector2d& p);
    void updatePath(const std::vector<Eigen::Vector2d>& path);
    void updateTrail(const std::vector<Eigen::Vector2d>& executed);
    void updateCylinders(const std::vector<Cylinder>& cylinders);
    void updateRobot(const RenderState& s);
    void updateGoal(const RenderState& s);
    void updateWaypoints(const RenderState& s);
    void updateHud(const RenderState& s);
    void updateHudThrottled(const RenderState& s);
    void setText(const std::string& id, const std::string& text,
                 double x, double y, double size,
                 double r, double g, double b);

    pcl::visualization::PCLVisualizer::Ptr viewer_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr path_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr exec_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinder_cloud_;

    WorldConfig world_cfg_;
    double pick_xmin_ = 0.0, pick_xmax_ = 20.0;
    double pick_ymin_ = 0.0, pick_ymax_ = 15.0;
    std::uint32_t frame_ = 0;

    // Per-frame shape churn caches: only recreate when the data actually changes.
    std::vector<Eigen::Vector2d> waypoint_cache_;
    bool waypoint_dirty_ = true;
    bool robot_ready_ = false;
    bool goal_ready_ = false;

    std::mutex pick_mutex_;
    std::deque<Eigen::Vector2d> picked_;
};
