#include "Config.h"

#include <cstdlib>
#include <iostream>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace {

template <typename T>
void readScalar(const YAML::Node& node, const char* key, T& out)
{
    const YAML::Node& v = node[key];
    if (v && v.IsScalar())
        out = v.as<T>();
}

void readVec2(const YAML::Node& node, const char* key, Eigen::Vector2d& out)
{
    const YAML::Node& v = node[key];
    if (v && v.IsSequence() && v.size() >= 2)
    {
        out.x() = v[0].as<double>();
        out.y() = v[1].as<double>();
    }
}

void readWorld(const YAML::Node& node, WorldConfig& w)
{
    if (!node) return;

    readScalar(node, "obstacles", w.num_cylinders);

    const YAML::Node& bounds = node["bounds"];
    if (bounds)
    {
        readScalar(bounds, "xmin", w.xmin);
        readScalar(bounds, "xmax", w.xmax);
        readScalar(bounds, "ymin", w.ymin);
        readScalar(bounds, "ymax", w.ymax);
    }

    const YAML::Node& work = node["work_range"];
    if (work)
    {
        readScalar(work, "xmin", w.work_xmin);
        readScalar(work, "xmax", w.work_xmax);
        readScalar(work, "ymin", w.work_ymin);
        readScalar(work, "ymax", w.work_ymax);
    }

    const YAML::Node& radius = node["radius"];
    if (radius)
    {
        readScalar(radius, "min", w.radius_min);
        readScalar(radius, "max", w.radius_max);
    }

    const YAML::Node& height = node["height"];
    if (height)
    {
        readScalar(height, "min", w.height_min);
        readScalar(height, "max", w.height_max);
    }

    const YAML::Node& speed = node["speed"];
    if (speed)
    {
        readScalar(speed, "min", w.speed_min);
        readScalar(speed, "max", w.speed_max);
    }

    readScalar(node, "clearance", w.clearance);
}

} // namespace

void loadConfig(const std::string& path, Config& cfg)
{
    YAML::Node root;
    try
    {
        root = YAML::LoadFile(path);
    }
    catch (const std::exception& e)
    {
        std::cerr << "[config] cannot load '" << path << "': " << e.what()
                  << " (using defaults)" << std::endl;
        return;
    }
    if (!root || !root.IsMap())
    {
        std::cerr << "[config] '" << path << "' is not a valid YAML map (using defaults)"
                  << std::endl;
        return;
    }

    const YAML::Node& sim = root["simulation"];
    if (sim)
    {
        readScalar(sim, "headless", cfg.headless);
        readScalar(sim, "debug", cfg.debug);
        readScalar(sim, "run_seconds", cfg.run_seconds);
        readScalar(sim, "dt", cfg.world_dt);
        readScalar(sim, "seed", cfg.seed);
    }

    const YAML::Node& world = root["world"];
    readWorld(world, cfg.world);

    const YAML::Node& robot = root["robot"];
    if (robot)
    {
        readVec2(robot, "start", cfg.start);
        readVec2(robot, "goal", cfg.goal);
        readScalar(robot, "speed", cfg.robot_speed);
        readScalar(robot, "safe_radius", cfg.safe_r);
        readScalar(robot, "window_len", cfg.window_len);
        readScalar(robot, "lookahead", cfg.lookahead);
        readScalar(robot, "step_spacing", cfg.step_spacing);
        readScalar(robot, "plan_expiry_ms", cfg.plan_expiry_ms);
    }

    const YAML::Node& pipe = root["pipeline"];
    if (pipe)
        readScalar(pipe, "perception_hz", cfg.perception_hz);

    std::cout << "[config] loaded '" << path << "'" << std::endl;
}

bool applyCliArgs(int argc, char** argv, Config& cfg)
{
    auto value_of = [&](int& i, double& out)
    {
        if (i + 1 < argc) out = std::atof(argv[++i]);
    };
    auto int_of = [&](int& i, int& out)
    {
        if (i + 1 < argc) out = std::atoi(argv[++i]);
    };

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--headless") cfg.headless = true;
        else if (arg == "--debug") cfg.debug = true;
        else if (arg == "--config")
        {
            if (i + 1 < argc) ++i;  // already consumed during the first pass; just skip here
        }
        else if (arg == "--seconds") value_of(i, cfg.run_seconds);
        else if (arg == "--dt") value_of(i, cfg.world_dt);
        else if (arg == "--seed") int_of(i, reinterpret_cast<int&>(cfg.seed));
        else if (arg == "--window") value_of(i, cfg.window_len);
        else if (arg == "--lookahead") value_of(i, cfg.lookahead);
        else if (arg == "--robot-speed") value_of(i, cfg.robot_speed);
        else if (arg == "--cylinders") int_of(i, cfg.world.num_cylinders);
        else if (arg == "--obstacle-speed")
        {
            double lo = 0.0, hi = 0.0;
            value_of(i, lo);
            value_of(i, hi);
            if (hi <= lo) hi = lo + 1e-6;
            cfg.world.speed_min = lo;
            cfg.world.speed_max = hi;
        }
        else if (arg == "--start")
        {
            if (i + 1 < argc)
            {
                std::string s = argv[++i];
                auto comma = s.find(',');
                if (comma != std::string::npos)
                {
                    cfg.start.x() = std::atof(s.substr(0, comma).c_str());
                    cfg.start.y() = std::atof(s.substr(comma + 1).c_str());
                }
            }
        }
        else if (arg == "--goal")
        {
            if (i + 1 < argc)
            {
                std::string s = argv[++i];
                auto comma = s.find(',');
                if (comma != std::string::npos)
                {
                    cfg.goal.x() = std::atof(s.substr(0, comma).c_str());
                    cfg.goal.y() = std::atof(s.substr(comma + 1).c_str());
                }
            }
        }
        else if (arg == "--help")
        {
            printUsage(argv[0]);
            return false;
        }
        else
        {
            std::cerr << "[config] unknown option: " << arg << std::endl;
            printUsage(argv[0]);
            return false;
        }
    }
    return true;
}

void printUsage(const char* prog)
{
    std::cout <<
        "Usage: " << prog << " [options]\n"
        "  --config <file>      YAML config file (default ./owl_planner.yaml)\n"
        "  --headless           run without GUI (latency benchmark)\n"
        "  --debug              print per-replan diagnostics\n"
        "  --seconds <s>        sim duration for headless mode\n"
        "  --dt <s>             simulation step\n"
        "  --seed <n>           RNG seed\n"
        "  --window <m>         sliding window length\n"
        "  --lookahead <m>      pure-pursuit lookahead\n"
        "  --robot-speed <m/s>  cruise speed\n"
        "  --cylinders <n>      number of dynamic obstacles\n"
        "  --obstacle-speed <lo hi>  obstacle speed range (m/s)\n"
        "  --start <x,y>        robot start position\n"
        "  --goal <x,y>         default goal position\n";
}
