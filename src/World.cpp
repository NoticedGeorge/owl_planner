#include "World.h"

#include <algorithm>
#include <cmath>

namespace {

// Generate interior points for a cylinder (radial fill). Density is kept low:
// the cloud is visualization-only, and lower density keeps the renderer fast.
std::vector<Eigen::Vector3d> generateCylinderOffsets(
    double radius, double height,
    int num_angle = 12, int num_height = 6, int num_radial = 3)
{
    std::vector<Eigen::Vector3d> offsets;
    offsets.reserve((num_height + 1) * (num_radial + 1) * num_angle);

    for (int i = 0; i <= num_height; ++i)
    {
        double z = height * i / num_height;

        for (int r_i = 0; r_i <= num_radial; ++r_i)
        {
            double r = radius * r_i / num_radial;

            for (int j = 0; j < num_angle; ++j)
            {
                double theta = 2.0 * M_PI * j / num_angle;
                offsets.emplace_back(r * std::cos(theta),
                                     r * std::sin(theta), z);
            }
        }
    }
    return offsets;
}

double rand01()
{
    return std::rand() / double(RAND_MAX);
}

} // namespace

World::World(const WorldConfig& cfg)
    : cfg_(cfg)
{
    buildCylinders(cfg_.num_cylinders);
}

void World::buildCylinders(int num)
{
    cylinders_.clear();
    cylinders_.reserve(num);

    std::srand(seed_);

    const double rmin = cfg_.radius_min;
    const double rmax = cfg_.radius_max;
    const double hmin = cfg_.height_min;
    const double hmax = cfg_.height_max;
    const double vmin = cfg_.speed_min;
    const double vmax = cfg_.speed_max;
    const double clearance = cfg_.clearance;

    for (int i = 0; i < num; ++i)
    {
        Cylinder c;
        bool valid = false;

        while (!valid)
        {
            c.center = Eigen::Vector2d(
                cfg_.xmin + (cfg_.xmax - cfg_.xmin) * rand01(),
                cfg_.ymin + (cfg_.ymax - cfg_.ymin) * rand01());
            c.radius = rmin + (rmax - rmin) * rand01();
            c.height = hmin + (hmax - hmin) * rand01();

            valid = true;
            for (const auto& other : cylinders_)
            {
                double dist = (c.center.head<2>() - other.center.head<2>()).norm();
                if (dist < c.radius + other.radius + clearance)
                {
                    valid = false;
                    break;
                }
            }
        }

        c.local_offsets = generateCylinderOffsets(c.radius, c.height);
        c.points.resize(c.local_offsets.size());

        double theta = 2.0 * M_PI * rand01();
        c.speed = vmin + (vmax - vmin) * rand01();
        c.velocity = Eigen::Vector2d(std::cos(theta) * c.speed,
                                     std::sin(theta) * c.speed);

        cylinders_.push_back(std::move(c));
    }

    updatePoints();
}

void World::updatePoints()
{
    for (auto& c : cylinders_)
    {
        for (size_t k = 0; k < c.local_offsets.size(); ++k)
            c.points[k] = Eigen::Vector3d(
                c.center.x() + c.local_offsets[k].x(),
                c.center.y() + c.local_offsets[k].y(),
                c.local_offsets[k].z());
    }
}

void World::separateAndBounce(double dt, const Eigen::Vector2d& robot_pos)
{
    for (auto& c : cylinders_)
    {
        c.center += c.velocity * dt;

        if (c.center.x() < cfg_.xmin || c.center.x() > cfg_.xmax)
            c.velocity.x() *= -1;
        if (c.center.y() < cfg_.ymin || c.center.y() > cfg_.ymax)
            c.velocity.y() *= -1;
    }

    // pairwise separation
    for (size_t i = 0; i < cylinders_.size(); ++i)
    {
        for (size_t j = i + 1; j < cylinders_.size(); ++j)
        {
            double dist = (cylinders_[i].center - cylinders_[j].center).norm();
            double min_dist = cylinders_[i].radius + cylinders_[j].radius;

            if (dist < min_dist && dist > 1e-9)
            {
                Eigen::Vector2d dir =
                    (cylinders_[i].center - cylinders_[j].center).normalized();

                cylinders_[i].velocity.head<2>() = dir * cylinders_[i].speed;
                cylinders_[j].velocity.head<2>() = -dir * cylinders_[j].speed;
            }
        }
    }

    // obstacles avoid the robot (mirroring obstacle-obstacle separation)
    const double robot_r = 0.25;
    for (auto& c : cylinders_)
    {
        double d = (c.center - robot_pos).norm();
        if (d < c.radius + robot_r + 0.6 && d > 1e-6)
        {
            Eigen::Vector2d dir = (c.center - robot_pos).normalized();
            c.velocity.head<2>() = dir * c.speed;
        }
    }

    updatePoints();
}

void World::step(double dt, const Eigen::Vector2d& robot_pos)
{
    std::lock_guard<std::mutex> lock(mutex_);
    t_ += dt;
    separateAndBounce(dt, robot_pos);
}

ObstacleMap World::circleObstacles(double inflate) const
{
    ObstacleMap out;
    out.reserve(cylinders_.size());

    for (const auto& c : cylinders_)
    {
        Obstacle o;
        o.center = c.center;
        o.radius = c.radius + inflate;
        o.active = true;
        o.velocity = c.velocity;
        o.velocity_gain = c.speed;
        out.push_back({o});
    }
    return out;
}

ObstacleMap World::snapshotCircleObstacles(double inflate)
{
    std::lock_guard<std::mutex> lock(mutex_);
    return circleObstacles(inflate);
}
