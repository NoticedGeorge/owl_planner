#pragma once

#include <string>

#include "World.h"

// Global run config: defaults -> YAML file -> CLI flags (each level overrides)
struct Config
{
    // Simulation
    bool headless = false;
    bool debug = false;
    double run_seconds = 120.0;
    double world_dt = 0.02;
    unsigned seed = 42;

    // World (obstacle simulation)
    WorldConfig world;

    // Robot / executor
    Eigen::Vector2d start{-10.0, -10.0};
    Eigen::Vector2d goal{40.0, 40.0};
    double robot_speed = 2.0;
    double safe_r = 0.2;
    double window_len = 14.0;
    double lookahead = 2.5;
    double step_spacing = 0.05;
    double plan_expiry_ms = 250.0;

    // Pipeline
    double perception_hz = 30.0;
};

// Load config from a YAML file (keeps defaults silently if the file is missing)
void loadConfig(const std::string& path, Config& cfg);

// Apply CLI flags (override the YAML config)
bool applyCliArgs(int argc, char** argv, Config& cfg);

void printUsage(const char* prog);
