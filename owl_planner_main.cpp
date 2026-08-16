#include <chrono>
#include <deque>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "Config.h"
#include "Controller.h"
#include "Pipeline.h"
#include "Visualizer.h"
#include "World.h"

namespace {

void printMetrics(const PipelineMetrics& m, double sim_time, double robot_speed,
                  bool collision, bool reached)
{
    std::cout <<
        "----------------------------------------------------------------\n"
        "[metrics] t=" << sim_time << "s  speed=" << robot_speed << " m/s\n"
        "  perception : " << m.perception_hz << " Hz (proc " << m.perception_ms << " ms)\n"
        "  planning   : " << m.plan_ms << " ms/plan, " << m.replans << " replans\n"
        "  timestamps : perception@plan " << m.perception_age_ms << " ms | "
        "plan age " << m.plan_age_ms << " ms | E2E " << m.end_to_end_ms << " ms\n"
        "  status     : " << (collision ? "COLLISION" : (reached ? "GOAL REACHED" : "RUNNING"))
        << std::endl;
}

} // namespace

int main(int argc, char** argv)
{
    Config cfg;
    std::string config_path = "owl_planner.yaml";

    // Parse --config first (CLI must override YAML, so args are processed in two passes)
    for (int i = 1; i + 1 < argc; ++i)
        if (std::string(argv[i]) == "--config")
            config_path = argv[i + 1];

    // Load ./owl_planner.yaml by default (built-in defaults if missing)
    loadConfig(config_path, cfg);

    // CLI flags override the YAML config
    if (!applyCliArgs(argc, argv, cfg))
        return 0;

    std::cout << "[owl-planner] obstacles=" << cfg.world.num_cylinders
              << " speed_range=[" << cfg.world.speed_min << ", "
              << cfg.world.speed_max << "] m/s  bounds=["
              << cfg.world.xmin << ", " << cfg.world.xmax << "] x ["
              << cfg.world.ymin << ", " << cfg.world.ymax << "]\n";

    // ===== World (dynamic obstacle simulation) =====
    World world(cfg.world);
    world.setSeed(cfg.seed);

    // ===== Perception + planning pipeline =====
    PlanningPipeline pipeline(world, cfg.safe_r, cfg.perception_hz);

    // ===== Executor (sliding window + pure pursuit) =====
    Controller controller(cfg.goal, cfg.window_len, cfg.lookahead,
                          cfg.robot_speed, cfg.safe_r, cfg.step_spacing,
                          cfg.plan_expiry_ms);

    RobotState robot{cfg.start, (cfg.goal - cfg.start).normalized() * cfg.robot_speed,
                     cfg.robot_speed};

    // Initial reference path: enter through the middle of the world boundary edge, not the corner
    {
        const Eigen::Vector2d entry(cfg.world.xmin,
                                    cfg.world.ymin + 0.4 * (cfg.world.ymax - cfg.world.ymin));
        std::vector<Vector2d> initial_path;
        auto append_seg = [&](const Eigen::Vector2d& a,
                              const Eigen::Vector2d& b, double step)
        {
            Eigen::Vector2d dir = b - a;
            double len = dir.norm();
            dir /= std::max(len, 1e-9);
            int n = static_cast<int>(len / step);
            for (int i = 0; i <= n; ++i)
                initial_path.push_back(a + dir * step * i);
            if ((initial_path.back() - b).norm() > 1e-6)
                initial_path.push_back(b);
        };
        append_seg(cfg.start, entry, cfg.step_spacing);
        append_seg(entry, cfg.goal, cfg.step_spacing);
        controller.initWithPath(initial_path);
    }

    // ===== Work range (from YAML, default -20..60) =====
    // The area where user waypoints are allowed and the pipeline's
    // safety/bypass checks consider reachable. Obstacles themselves stay
    // inside world.bounds.
    const double wxmin = cfg.world.work_xmin;
    const double wxmax = cfg.world.work_xmax;
    const double wymin = cfg.world.work_ymin;
    const double wymax = cfg.world.work_ymax;

    pipeline.setWorkBounds(wxmin, wxmax, wymin, wymax);

    // ===== Visualization =====
    std::unique_ptr<Visualizer> viz;
    if (!cfg.headless)
    {
        viz = std::make_unique<Visualizer>(cfg.world);
        viz->setPickRange(wxmin, wxmax, wymin, wymax);
    }

    pipeline.start();
    pipeline.updateRobot(robot);

    // Wait for the first perception frame before the initial plan (avoid planning on an empty map)
    auto wait_t0 = Clock::now();
    while (!pipeline.perceptionReady())
    {
        if (msSince(wait_t0) > 3000.0)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    pipeline.submitRequest(controller.makeInitialRequest(robot));

    // ===== User waypoint state =====
    Eigen::Vector2d current_goal = cfg.goal;      // current goal (replaceable by user waypoints)
    bool has_user_goal = false;                    // whether a user goal is set
    std::deque<Eigen::Vector2d> waypoint_queue;    // queued waypoints to execute in order
    std::vector<Eigen::Vector2d> display_waypoints; // waypoints for rendering

    std::uint64_t applied_seq = 0;
    double last_print = 0.0;
    double sim_time = 0.0;
    bool collision = false;
    bool sim_done = false;

    // Decoupled pacing: the simulation advances with fixed 50 Hz steps (an
    // accumulator keeps it real-time even if a frame takes long), while
    // rendering happens once per outer iteration at a fixed 30 Hz. 30 Hz
    // divides evenly into common 60/120 Hz displays, so idle frames are
    // evenly spaced instead of fighting vsync (the source of the stutter).
    const double render_period = 1.0 / 30.0;
    double accumulator = 0.0;
    auto last_tick = Clock::now();

    while (!sim_done)
    {
        if (viz && viz->wasStopped()) break;

        // ---- advance the simulation with fixed 50 Hz steps ----
        double frame_seconds = std::min(
            std::chrono::duration<double>(Clock::now() - last_tick).count(),
            0.1);
        last_tick = Clock::now();
        accumulator += frame_seconds;

        while (accumulator >= cfg.world_dt && !sim_done)
        {
            accumulator -= cfg.world_dt;

            // 1. advance the simulation
            world.step(cfg.world_dt, robot.pos);
            sim_time += cfg.world_dt;

            pipeline.updateRobot(robot);

            // 2. consume the latest plan (dedup by seq, splice once)
            PlanFrame plan;
            if (pipeline.latestPlan(plan) && plan.frame.seq != applied_seq)
            {
                applied_seq = plan.frame.seq;
                controller.applyPlan(plan);

                if (cfg.debug)
                {
                    std::cout << "[debug] applied plan #" << plan.frame.seq
                              << " reason=" << plan.reason
                              << " plan_ms=" << plan.plan_ms
                              << " pts=" << plan.frame.data.size()
                              << " perc_age=" << plan.perception_age_ms
                              << "ms" << std::endl;

                    // check min clearance between the plan and real obstacles (<0 means clipping)
                    double min_clear = 1e9;
                    auto circles = world.circleObstacles();
                    for (const auto& p : plan.frame.data)
                        for (const auto& g : circles)
                            for (const auto& obs : g)
                                min_clear = std::min(min_clear,
                                    (p - obs.center).norm() - obs.radius);
                    std::cout << "[debug]   min plan clearance: "
                              << min_clear << " m" << std::endl;
                }

                double e2e = msSince(plan.perception_stamp);
                pipeline.reportEndToEnd(e2e);
            }

            // 3. real-time collision check (highest priority)
            if (!collision && !controller.reached())
            {
                auto circles = world.circleObstacles();
                for (const auto& group : circles)
                    for (const auto& obs : group)
                        if ((robot.pos - obs.center).norm() < obs.radius)
                        {
                            collision = true;
                            controller.setCollided(true);
                            std::cout << "[collision] robot stopped at ("
                                      << robot.pos.x() << ", " << robot.pos.y()
                                      << "), obstacle at (" << obs.center.x()
                                      << ", " << obs.center.y() << ") r="
                                      << obs.radius << std::endl;
                            break;
                        }
            }

            // 4. sliding-window replan decision
            auto req = controller.update(robot, world.circleObstacles(),
                                         pipeline.planAgeMs(),
                                         pipeline.replanInFlight());
            if (!req.reason.empty())
                pipeline.submitRequest(std::move(req));

            // 5. path tracking
            robot = controller.follow(robot, cfg.world_dt, world.circleObstacles());

            // 7. periodic metrics output: headless only (the GUI HUD already
            //    shows the same numbers, and console spam delays the render loop)
            if (cfg.headless && sim_time - last_print >= 2.0)
            {
                last_print = sim_time;
                printMetrics(pipeline.metrics(), sim_time, robot.speed,
                             collision, controller.reached());
            }

            if (cfg.headless && sim_time >= cfg.run_seconds) sim_done = true;
            if (cfg.headless && controller.reached()) sim_done = true;
        }

        // ---- user waypoints + render, once per 30 Hz tick ----
        if (viz)
        {
            // 3.5 user waypoints: LMB click in the GUI
            auto picks = viz->takePickedWaypoints();
            for (const auto& p : picks)
            {
                if (!has_user_goal || controller.collided())
                {
                    // first click, or recovering from a collision: retarget immediately
                    // (setGoal resets the collided state, so the robot moves again)
                    current_goal = p;
                    has_user_goal = true;
                    collision = false;
                    controller.setGoal(p, robot.pos);
                    pipeline.submitRequest(controller.makeInitialRequest(robot));
                    std::cout << "[waypoint] retarget to (" << p.x() << ", "
                              << p.y() << ")" << std::endl;
                }
                else if (waypoint_queue.size() < 3)
                {
                    // pre-placed waypoints are allowed, but at most 3 in advance
                    waypoint_queue.push_back(p);
                    std::cout << "[waypoint] queued (" << p.x() << ", "
                              << p.y() << "), queue=" << waypoint_queue.size()
                              << "/3" << std::endl;
                }
                else
                {
                    std::cout << "[waypoint] queue full (3/3) - reach the current "
                                 "goal first" << std::endl;
                }
            }

            // after reaching the current user goal, pop the next queued waypoint
            if (controller.reached() && !waypoint_queue.empty())
            {
                current_goal = waypoint_queue.front();
                waypoint_queue.pop_front();
                controller.setGoal(current_goal, robot.pos);
                pipeline.submitRequest(controller.makeInitialRequest(robot));
                std::cout << "[waypoint] next queued goal (" << current_goal.x()
                          << ", " << current_goal.y() << ")" << std::endl;
            }

            // 6. render
            // render only queued waypoints; the current goal is already shown by the GOAL marker
            display_waypoints.clear();
            display_waypoints.insert(display_waypoints.end(),
                                     waypoint_queue.begin(),
                                     waypoint_queue.end());

            RenderState rs;
            rs.cylinders = &world.cylinders();
            rs.path = &controller.remainingPath();
            rs.executed = &controller.executedPath();
            rs.waypoints = &display_waypoints;
            rs.robot = robot;
            rs.goal = current_goal;
            rs.collision = collision;
            rs.reached = controller.reached();
            rs.replan_pending = pipeline.replanInFlight();
            rs.sim_time = sim_time;
            rs.window_len_m = cfg.window_len;
            rs.metrics = pipeline.metrics();
            viz->render(rs);
        }

        // 8. pace the outer loop at a fixed 30 Hz
        double work = std::chrono::duration<double>(Clock::now() - last_tick).count();
        if (work < render_period)
            std::this_thread::sleep_for(
                std::chrono::duration<double>(render_period - work));
    }

    pipeline.stop();

    // ===== Summary =====
    auto m = pipeline.metrics();
    std::cout <<
        "======================== SUMMARY ========================\n"
        "reached goal : " << (controller.reached() ? "YES" : "NO") << "\n"
        "collision    : " << (collision ? "YES" : "NO") << "\n"
        "dodges       : " << controller.dodgeCount() << "\n"
        "replans      : " << m.replans << "\n"
        "plan latency : " << m.plan_ms << " ms (last)\n"
        "perception   : " << m.perception_hz << " Hz, proc " << m.perception_ms << " ms\n"
        "E2E latency  : " << m.end_to_end_ms << " ms\n"
        "==========================================================" << std::endl;

    return 0;
}
