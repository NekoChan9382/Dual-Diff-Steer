#ifndef STEER_DRIVE_HPP
#define STEER_DRIVE_HPP

// #include "mbed.h"
#include "coordinate.hpp"
#include <cmath>
#include <array>

namespace bit {

struct SteerValue
{
    float vel;
    float theta;

    SteerValue(const float& vel, const float& theta)
    : vel(vel), theta(theta) {}
    SteerValue() : vel(0), theta(0) {}

    explicit SteerValue(const CoordinatePolar& other) noexcept
    {
        vel = other.r;
        theta = other.theta;
    }

    explicit SteerValue(const Coordinate& other) noexcept
    {

        SteerValue(static_cast<CoordinatePolar>(other));
    }
};

template<int N>
class SteerDrive
{
    static_assert(N > 0, "N must be positive");
public:
    SteerDrive(const float& robot_radius) : robot_radius_(robot_radius)
    {
        for (int i = 0; i < N; ++i)
        {
            constexpr float k = 2 * M_PI / N;
            constexpr float ofs = k / 2;
            wheel_pos_[i] = Coordinate(robot_radius_, 0, 0, i * k + ofs);
        }
    }
    SteerDrive() : robot_radius_(0) {}

    SteerDrive(const std::array<Coordinate, N>& wheel_pos)
    {
        for (int i = 0; i < N; ++i)
        {
            wheel_pos[i] = wheel_pos[i];
        }
    }

    std::array<SteerValue, N> calc_vel(const Velocity& vel)
    {
        std::array<SteerValue, N> value;
        for (int i = 0; i<N; ++i)
        {
            CoordinatePolar pos = static_cast<CoordinatePolar>(wheel_pos_[i]);
            convert_ang(pos, 0);
            const Velocity tmp = {vel.x + pos.r * vel.ang * cos(pos.theta),
                                    vel.y + pos.r * vel.ang * cos(pos.theta)};
            value[i] = static_cast<SteerValue>(static_cast<VelocityPolar>(tmp));
        }
        return value;
    }

    
private:
    float robot_radius_;
    std::array<Coordinate, N> wheel_pos_;
};

}  // namespace bit

#endif // DIFF_STEER_HPP
