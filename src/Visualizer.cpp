#include "Visualizer.h"

#include <algorithm>
#include <cmath>
#include <sstream>

#include <vtkCamera.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObject.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkObjectFactory.h>

// ---------------------------------------------------------------------------
// Mouse waypoint style: keeps the standard trackball camera (LMB drag rotate / MMB pan /
// wheel zoom); on LMB "click" (press-release displacement < 6 px), ray-intersect the z=0 plane
// to get a user waypoint on the simulation ground.
// ---------------------------------------------------------------------------
class WaypointStyle : public vtkInteractorStyleTrackballCamera
{
public:
    static WaypointStyle* New();
    vtkTypeMacro(WaypointStyle, vtkInteractorStyleTrackballCamera);

    std::function<void(double x, double y)> on_click;

    void OnLeftButtonDown() override
    {
        int* p = GetInteractor()->GetEventPosition();
        down_x_ = p[0];
        down_y_ = p[1];
        Superclass::OnLeftButtonDown();
    }

    void OnLeftButtonUp() override
    {
        Superclass::OnLeftButtonUp();
        int* p = GetInteractor()->GetEventPosition();
        int dx = p[0] - down_x_;
        int dy = p[1] - down_y_;
        if (dx * dx + dy * dy <= 36)
            emitPick();
    }

private:
    void emitPick()
    {
        vtkRenderWindowInteractor* iren = GetInteractor();
        if (!iren) return;

        int* pos = iren->GetEventPosition();
        vtkRenderer* ren = iren->FindPokedRenderer(pos[0], pos[1]);
        if (!ren || !ren->GetActiveCamera()) return;

        // display coords -> world point on the near clipping plane, giving the view ray
        ren->SetDisplayPoint(pos[0], pos[1], 0.0);
        ren->DisplayToWorld();
        double* w = ren->GetWorldPoint();
        double ww = (w[3] != 0.0) ? w[3] : 1.0;
        double wx = w[0] / ww, wy = w[1] / ww, wz = w[2] / ww;

        double cam[3];
        ren->GetActiveCamera()->GetPosition(cam);
        double dx = wx - cam[0], dy = wy - cam[1], dz = wz - cam[2];
        if (std::abs(dz) < 1e-9) return;

        // intersect with z=0 (sim ground)
        double t = (0.0 - cam[2]) / dz;
        if (t <= 0.0) return;

        double gx = cam[0] + dx * t;
        double gy = cam[1] + dy * t;
        if (on_click) on_click(gx, gy);
    }

    int down_x_ = 0;
    int down_y_ = 0;
};
vtkStandardNewMacro(WaypointStyle);

namespace {

// three-color gradient
struct RGB
{
    double r, g, b;
};

RGB lerpRGB(const RGB& a, const RGB& b, double t)
{
    t = std::max(0.0, std::min(1.0, t));
    return {a.r + (b.r - a.r) * t,
            a.g + (b.g - a.g) * t,
            a.b + (b.b - a.b) * t};
}

RGB pathColor(double t)
{
    // green -> cyan -> purple
    static const RGB stops[] = {{0.15, 1.0, 0.45},
                                {0.15, 0.85, 1.0},
                                {0.75, 0.35, 1.0}};
    if (t < 0.5) return lerpRGB(stops[0], stops[1], t * 2.0);
    return lerpRGB(stops[1], stops[2], (t - 0.5) * 2.0);
}

RGB speedColor(double speed, double vmin, double vmax)
{
    double t = (speed - vmin) / std::max(vmax - vmin, 1e-6);
    return lerpRGB({1.0, 0.85, 0.15}, {1.0, 0.15, 0.10}, t);
}

pcl::PointXYZRGB makePoint(const Eigen::Vector2d& p, double z,
                           const RGB& c)
{
    pcl::PointXYZRGB pt;
    pt.x = static_cast<float>(p.x());
    pt.y = static_cast<float>(p.y());
    pt.z = static_cast<float>(z);
    pt.r = static_cast<std::uint8_t>(std::max(0.0, std::min(1.0, c.r)) * 255.0);
    pt.g = static_cast<std::uint8_t>(std::max(0.0, std::min(1.0, c.g)) * 255.0);
    pt.b = static_cast<std::uint8_t>(std::max(0.0, std::min(1.0, c.b)) * 255.0);
    return pt;
}

pcl::PointXYZRGB makePoint(const Eigen::Vector3d& p, const RGB& c)
{
    return makePoint(Eigen::Vector2d(p.x(), p.y()), p.z(), c);
}

} // namespace

Visualizer::Visualizer(const WorldConfig& world_cfg)
    : viewer_(new pcl::visualization::PCLVisualizer("OWL-Planner")),
      path_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>),
      exec_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>),
      cylinder_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>),
      world_cfg_(world_cfg)
{
    buildScene();
}

void Visualizer::buildScene()
{
    // PCL/VTK 9 triggers runtime warnings about deprecated vtkOpenGLPolyDataMapper APIs
    // (e.g. "vtkOpenGLPolyDataMapper:313 WARN| ...SetFragmentShaderCode was deprecated..."),
    // so VTK warning output is globally disabled to avoid flooding the console.
    vtkObject::GlobalWarningDisplayOff();

    viewer_->setBackgroundColor(0.055, 0.07, 0.11);
    viewer_->addCoordinateSystem(1.0);

    // background grid
    const double x0 = -12.0, x1 = 44.0;
    const double y0 = -12.0, y1 = 44.0;
    const double step = 4.0;
    int id = 0;
    for (double x = x0; x <= x1; x += step)
    {
        std::string name = "grid_v_" + std::to_string(id++);
        viewer_->addLine(pcl::PointXYZ(x, y0, -0.01),
                         pcl::PointXYZ(x, y1, -0.01),
                         0.13, 0.16, 0.24, name);
    }
    id = 0;
    for (double y = y0; y <= y1; y += step)
    {
        std::string name = "grid_h_" + std::to_string(id++);
        viewer_->addLine(pcl::PointXYZ(x0, y, -0.01),
                         pcl::PointXYZ(x1, y, -0.01),
                         0.13, 0.16, 0.24, name);
    }

    // world bounds (simulation area)
    const auto& w = world_cfg_;
    const Eigen::Vector2d a(w.xmin, w.ymin), b(w.xmax, w.ymin),
                          c(w.xmax, w.ymax), d(w.xmin, w.ymax);
    viewer_->addLine(pcl::PointXYZ(a.x(), a.y(), 0),
                     pcl::PointXYZ(b.x(), b.y(), 0), 0.35, 0.45, 0.6, "world_x0");
    viewer_->addLine(pcl::PointXYZ(b.x(), b.y(), 0),
                     pcl::PointXYZ(c.x(), c.y(), 0), 0.35, 0.45, 0.6, "world_y0");
    viewer_->addLine(pcl::PointXYZ(c.x(), c.y(), 0),
                     pcl::PointXYZ(d.x(), d.y(), 0), 0.35, 0.45, 0.6, "world_x1");
    viewer_->addLine(pcl::PointXYZ(d.x(), d.y(), 0),
                     pcl::PointXYZ(a.x(), a.y(), 0), 0.35, 0.45, 0.6, "world_y1");

    // install the mouse waypoint interaction style
    vtkSmartPointer<vtkRenderWindowInteractor> iren =
        viewer_->getRenderWindow()->GetInteractor();
    if (iren)
    {
        vtkSmartPointer<WaypointStyle> style = vtkSmartPointer<WaypointStyle>::New();
        style->on_click = [this](double x, double y)
        {
            Eigen::Vector2d p(x, y);
            // clamp waypoints into the sim area so the planner can handle them
            p.x() = std::max(world_cfg_.xmin, std::min(world_cfg_.xmax, p.x()));
            p.y() = std::max(world_cfg_.ymin, std::min(world_cfg_.ymax, p.y()));
            addPickedWaypoint(p);
        };
        iren->SetInteractorStyle(style);
    }

    // initial title
    setText("hud_title", "OWL-Planner Dynamic Planning Pipeline",
            12, 560, 22, 0.35, 0.85, 1.0);
}

void Visualizer::setText(const std::string& id, const std::string& text,
                         double x, double y, double size,
                         double r, double g, double b)
{
    if (!viewer_->updateText(text, x, y, size, r, g, b, id))
        viewer_->addText(text, x, y, size, r, g, b, id);
}

void Visualizer::addPickedWaypoint(const Eigen::Vector2d& p)
{
    std::lock_guard<std::mutex> lock(pick_mutex_);
    picked_.push_back(p);
}

std::vector<Eigen::Vector2d> Visualizer::takePickedWaypoints()
{
    std::lock_guard<std::mutex> lock(pick_mutex_);
    std::vector<Eigen::Vector2d> out(picked_.begin(), picked_.end());
    picked_.clear();
    return out;
}

void Visualizer::updatePath(const std::vector<Eigen::Vector2d>& path)
{
    path_cloud_->points.clear();
    path_cloud_->reserve(path.size());

    const double n = std::max(1, static_cast<int>(path.size()));
    for (int i = 0; i < static_cast<int>(path.size()); ++i)
        path_cloud_->points.push_back(makePoint(path[i], 0.01,
                                                pathColor(i / n)));

    path_cloud_->width = path_cloud_->points.size();
    path_cloud_->height = 1;
    path_cloud_->is_dense = true;

    if (!viewer_->updatePointCloud(path_cloud_, "path"))
    {
        viewer_->addPointCloud(path_cloud_, "path");
        viewer_->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "path");
    }
}

void Visualizer::updateTrail(const std::vector<Eigen::Vector2d>& executed)
{
    exec_cloud_->points.clear();
    exec_cloud_->reserve(executed.size());

    const int total = static_cast<int>(executed.size());
    const int stride = std::max(1, total / 2000); // keep at most ~2000 rendered trail points
    const double n = std::max(1, total);
    for (int i = 0; i < total; i += stride)
    {
        // older = dimmer (magenta fade-out)
        double alpha = std::max(0.0, 1.0 - i / n);
        exec_cloud_->points.push_back(makePoint(
            executed[i], 0.005,
            {0.55 * alpha, 0.15 * alpha, 0.75 * alpha}));
    }

    exec_cloud_->width = exec_cloud_->points.size();
    exec_cloud_->height = 1;
    exec_cloud_->is_dense = true;

    if (!viewer_->updatePointCloud(exec_cloud_, "exec_cloud"))
    {
        viewer_->addPointCloud(exec_cloud_, "exec_cloud");
        viewer_->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "exec_cloud");
    }
}

void Visualizer::updateCylinders(const std::vector<Cylinder>& cylinders)
{
    cylinder_cloud_->points.clear();

    // downsample when there are many obstacles to keep the cloud small
    const int stride = cylinders.size() > 40 ? 3 : 1;
    const double vmin = world_cfg_.speed_min;
    const double vmax = std::max(world_cfg_.speed_max, vmin + 1e-6);

    for (const auto& c : cylinders)
    {
        RGB col = speedColor(c.speed, vmin, vmax);
        for (int k = 0; k < static_cast<int>(c.points.size()); k += stride)
            cylinder_cloud_->points.push_back(makePoint(c.points[k], col));
    }

    cylinder_cloud_->width = cylinder_cloud_->points.size();
    cylinder_cloud_->height = 1;
    cylinder_cloud_->is_dense = true;

    if (!viewer_->updatePointCloud(cylinder_cloud_, "cylinders_cloud"))
    {
        viewer_->addPointCloud(cylinder_cloud_, "cylinders_cloud");
        viewer_->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cylinders_cloud");
    }

    // per-cylinder velocity arrows: rebuilding every frame is expensive,
    // so redraw them at ~10 Hz only
    if (frame_ % 10 == 0)
    {
        for (int i = 0; i < static_cast<int>(cylinders.size()); ++i)
        {
            std::string id = "cyl_arrow_" + std::to_string(i);
            const auto& c = cylinders[i];
            Eigen::Vector2d end = c.center + c.velocity * 0.6;

            viewer_->removeShape(id);
            viewer_->addArrow(pcl::PointXYZ(c.center.x(), c.center.y(), 0.05),
                              pcl::PointXYZ(end.x(), end.y(), 0.05),
                              1.0, 0.65, 0.25, false, id);
        }
    }
}

void Visualizer::updateRobot(const RenderState& s)
{
    const double r = s.collision ? 0.14 : 0.13;
    const double cr = s.collision ? 1.0 : 1.0;
    const double cg = s.collision ? 0.15 : 0.85;
    const double cb = s.collision ? 0.15 : 0.2;

    // body: create once, then update in place (cheap)
    if (!robot_ready_)
    {
        viewer_->addSphere(pcl::PointXYZ(s.robot.pos.x(), s.robot.pos.y(), 0),
                           r, cr, cg, cb, "robot");
        viewer_->addSphere(pcl::PointXYZ(s.robot.pos.x(), s.robot.pos.y(), 0),
                           0.28, 0.2, 0.9, 1.0, "robot_halo");
        viewer_->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "robot_halo");
        robot_ready_ = true;
    }
    else
    {
        viewer_->updateSphere(pcl::PointXYZ(s.robot.pos.x(), s.robot.pos.y(), 0),
                              r, cr, cg, cb, "robot");
        // pulse halo
        double halo = 0.22 + 0.06 * std::sin(2.0 * M_PI * s.sim_time / 0.7);
        viewer_->updateSphere(pcl::PointXYZ(s.robot.pos.x(), s.robot.pos.y(), 0),
                              halo, 0.2, 0.9, 1.0, "robot_halo");
    }

    // heading arrow: rebuild at ~15 Hz
    if (frame_ % 5 == 0)
    {
        viewer_->removeShape("robot_arrow");
        if (s.robot.vel.norm() > 1e-3)
        {
            Eigen::Vector2d end = s.robot.pos + s.robot.vel.normalized() * 0.55;
            viewer_->addArrow(pcl::PointXYZ(s.robot.pos.x(), s.robot.pos.y(), 0.05),
                              pcl::PointXYZ(end.x(), end.y(), 0.05),
                              1.0, 1.0, 1.0, false, "robot_arrow");
        }
    }
}

void Visualizer::updateGoal(const RenderState& s)
{
    double r = 0.18 + 0.05 * std::sin(2.0 * M_PI * s.sim_time / 0.9);

    if (!goal_ready_)
    {
        viewer_->addSphere(pcl::PointXYZ(s.goal.x(), s.goal.y(), 0),
                           r, 0.2, 1.0, 0.45, "goal");
        viewer_->addText3D("GOAL",
                           pcl::PointXYZ(s.goal.x(), s.goal.y(), 0.2),
                           0.8, 0.2, 1.0, 0.45, "goal_label");
        goal_ready_ = true;
    }
    else
    {
        viewer_->updateSphere(pcl::PointXYZ(s.goal.x(), s.goal.y(), 0),
                              r, 0.2, 1.0, 0.45, "goal");
        if (frame_ % 5 == 0)
        {
            viewer_->removeShape("goal_label");
            viewer_->addText3D("GOAL",
                               pcl::PointXYZ(s.goal.x(), s.goal.y(), 0.2),
                               0.8, 0.2, 1.0, 0.45, "goal_label");
        }
    }
}

void Visualizer::updateWaypoints(const RenderState& s)
{
    // clear the previous frame's waypoint markers
    for (int i = 0; i < 32; ++i)
    {
        viewer_->removeShape("wp_sphere_" + std::to_string(i));
        viewer_->removeShape("wp_label_" + std::to_string(i));
    }

    if (!s.waypoints) return;

    const int n = std::min(32, static_cast<int>(s.waypoints->size()));
    for (int i = 0; i < n; ++i)
    {
        const Eigen::Vector2d& p = (*s.waypoints)[i];
        std::string id = "wp_sphere_" + std::to_string(i);
        viewer_->addSphere(pcl::PointXYZ(p.x(), p.y(), 0),
                           0.12, 0.15, 1.0, 0.3, id);

        std::string label = "WP" + std::to_string(i);
        std::string lid = "wp_label_" + std::to_string(i);
        viewer_->addText3D(label, pcl::PointXYZ(p.x(), p.y(), 0.2),
                           0.6, 0.15, 1.0, 0.3, lid);
    }
}

void Visualizer::updateHud(const RenderState& s)
{
    const auto& m = s.metrics;

    std::ostringstream oss;

    oss << "sim " << s.sim_time << "s   robot " << s.robot.speed << " m/s";
    setText("hud_0", oss.str(), 12, 530, 14, 0.85, 0.9, 0.95);
    oss.str("");

    oss << "perception  " << m.perception_hz << " Hz  (proc "
        << m.perception_ms << " ms, seq " << m.perception_seq << ")";
    setText("hud_1", oss.str(), 12, 505, 14, 0.6, 0.85, 1.0);
    oss.str("");

    oss << "plan latency  " << m.plan_ms << " ms   replans  " << m.replans
        << "  (seq " << m.plan_seq << ")";
    setText("hud_2", oss.str(), 12, 480, 14, 1.0, 0.75, 0.4);
    oss.str("");

    oss << "perception@plan  " << m.perception_age_ms << " ms   plan age  "
        << m.plan_age_ms << " ms";
    setText("hud_3", oss.str(), 12, 455, 14, 0.9, 0.85, 0.6);
    oss.str("");

    oss << "E2E (perception -> exec)  " << m.end_to_end_ms << " ms";
    setText("hud_4", oss.str(), 12, 430, 14, 1.0, 0.6, 0.8);
    oss.str("");

    std::string status;
    double r = 0.3, g = 1.0, b = 0.4;
    if (s.collision)
    {
        status = "COLLISION - STOPPED";
        r = 1.0; g = 0.2; b = 0.2;
    }
    else if (s.reached)
    {
        status = "GOAL REACHED";
        r = 0.2; g = 1.0; b = 0.4;
    }
    else if (s.replan_pending)
    {
        status = "PLANNING...";
        r = 1.0; g = 0.85; b = 0.2;
    }
    else
    {
        status = "FOLLOWING";
        r = 0.3; g = 1.0; b = 0.6;
    }

    oss << "window " << s.window_len_m << " m   status: " << status;
    setText("hud_5", oss.str(), 12, 405, 16, r, g, b);

    setText("hud_legend",
            "green-blue = plan path   magenta = trail   yellow = robot   red = obstacle",
            12, 20, 12, 0.5, 0.55, 0.65);

    setText("hud_hint",
            "LMB click = set waypoint   drag = rotate   wheel = zoom",
            12, 4, 12, 0.4, 0.75, 0.6);
}

// HUD text is refreshed at ~5 Hz instead of every frame to cut VTK text churn.
void Visualizer::updateHudThrottled(const RenderState& s)
{
    if (frame_ % 5 == 0)
        updateHud(s);
}

void Visualizer::render(const RenderState& state)
{
    ++frame_;

    if (state.path) updatePath(*state.path);
    if (state.executed) updateTrail(*state.executed);
    if (state.cylinders) updateCylinders(*state.cylinders);

    updateRobot(state);
    updateGoal(state);

    // Only rebuild waypoint markers when the list actually changes.
    if (state.waypoints)
    {
        if (waypoint_dirty_ || *state.waypoints != waypoint_cache_)
        {
            waypoint_cache_ = *state.waypoints;
            updateWaypoints(state);
        }
    }

    updateHudThrottled(state);

    // force_redraw = true guarantees an actual frame: PCL's spinOnce only
    // re-renders via interactor timer events, and the default trackball
    // style's OnTimer does not render (PCL's own style does). Without this,
    // the scene freezes whenever the mouse is idle.
    viewer_->spinOnce(1, true);
}
