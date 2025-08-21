#include "coordinate.hpp"
#include "steer_drive.hpp"
#include "PID_new.hpp"
#include <iostream>
#include <cmath>
#include "C620.hpp"
#include "mbed.h"
#include "Rs485.h"
#include <list>

float duration_to_sec(const std::chrono::duration<float> &duration);

Rs485 rs485{PB_6, PA_10, (int)2e6, D8};

struct Ps5
{
    int8_t lstick_x = 0;
    int8_t lstick_y = 0;
    int8_t rstick_x = 0;
    int8_t rstick_y = 0;
    uint8_t l2 = 0;
    uint8_t r2 = 0;

    bool right = 0;
    bool up = 0;
    bool left = 0;
    bool down = 0;
    bool circle = 0;
    bool triangle = 0;
    bool square = 0;
    bool cross = 0;
    bool l1 = 0;
    bool r1 = 0;
    bool l3 = 0;
    bool r3 = 0;
    bool option = 0;
    bool share = 0;

    void parse(CANMessage msg)
    {
        switch (msg.id)
        {
            case 50:
            lstick_x = msg.data[0];
            lstick_y = msg.data[1];
            rstick_x = msg.data[2];
            rstick_y = msg.data[3];
            l2 = msg.data[4];
            r2 = msg.data[5];
            break;

            case 51:
            right = msg.data[0] >> 3 & 1;
            up = msg.data[0] >> 2 & 1;
            left = msg.data[0] >> 1 & 1;
            down = msg.data[0] & 1;
            circle = msg.data[1] >> 3 & 1;
            triangle = msg.data[1] >> 2 & 1;
            square = msg.data[1] >> 1 & 1;
            cross = msg.data[1] & 1;
            l1 = msg.data[2];
            r1 = msg.data[3];
            l3 = msg.data[4];
            r3 = msg.data[5];
            option = msg.data[6];
            share = msg.data[7];
            break;
        }
    }

    bool read(CAN& can)
    {
        CANMessage msg;
        if (can.read(msg); msg.id == 50 || msg.id == 51)
        {
            parse(msg);
            return true;
        }
        return false;
    }
};

struct Amt21
{
    static constexpr int rotate = 4096;

    uint8_t address;
    int32_t pos;
    int32_t fixed_pos;
    uint16_t pre_pos;
    int32_t zero_pos = 0;

    bool request_pos()
    {
        rs485.uart_transmit({address});
        if (uint16_t now_pos; rs485.uart_receive(&now_pos, sizeof(now_pos), 500us) && is_valid(now_pos))
        {
            now_pos = (now_pos & 0x3fff) >> 2;
            // printf("now_pos: %d\n", now_pos);
            int16_t diff = now_pos - pre_pos;
            if (diff > rotate / 2)
            {
                diff -= rotate;
            }
            else if (diff < -rotate / 2)
            {
                diff += rotate;
            }
            pos += diff;
            fixed_pos = -(pos - zero_pos);
            pre_pos = now_pos;
            return true;
        }
        return false;
    }

    void set_zero_pos()
    {
        zero_pos = pos;
    }
    void set_zero_pos(int32_t pos)
    {
        zero_pos = pos;
    }

    void request_reset()
    {
        rs485.uart_transmit({uint8_t(address + 2), 0x75});
    }

    static bool is_valid(uint16_t raw_data)
    {
        bool k1 = raw_data >> 15;
        bool k0 = raw_data >> 14 & 1;
        raw_data <<= 2;
        do
        {
            k1 ^= raw_data & 0x8000;         // even
            k0 ^= (raw_data <<= 1) & 0x8000; // odd
        } while (raw_data <<= 1);
        // printf("k0: %d, k1: %d\n", k0, k1);
        return k0 && k1;
    }
};

namespace bit
{

    template <int N>
    class DiffSteer
    {
    public:
        DiffSteer(const float &robot_radius, PinName rx, PinName tx) : c620_(rx, tx)
        {
            for (int i = 0; i < N * 2; ++i)
            {
                pid_rps_[i] = Pid(param_rps_);
                pid_rps_[i].reset();
            }
            for (int i = 0; i < N; ++i)
            {
                pid_theta_[i] = Pid(param_theta_);
                pid_theta_[i].reset();
            }
            steer_ = bit::SteerDrive<N>(robot_radius);
            c620_.set_max_output(max_motor_pwr_);
        }
        bool set_output(const Velocity &vel, const float (&theta)[N], const float elapsed)
        {
            std::array<SteerValue, N> vel_wheel = steer_.calc_vel(vel);
            optimize_steervalue(vel_wheel, theta);
            float rps[N * 2] = {0};
            calc_rps(rps, vel_wheel, theta, elapsed);
            // printf("\nrps: %f, %f\n", rps[0], rps[1]);
            pid(rps, elapsed);

            return c620_.write();
        }
        int read_c620()
        {
            return c620_.read_data();
        }

        bool reset()
        {
            for (int i = 0; i < N * 2; ++i)
            {
                c620_.set_output(0, i + 1);
            }
            for (int i = 0; i < N * 2; ++i)
            {
                pid_rps_[i] = Pid(param_rps_);
                pid_rps_[i].reset();
            }
            for (int i = 0; i < N; ++i)
            {
                pid_theta_[i] = Pid(param_theta_);
                pid_theta_[i].reset();
            }
            return c620_.write();
        }

    private:
        void optimize_steervalue(std::array<SteerValue, N> motor_vel, const float (&theta)[N])
        {
            for (int i = 0; i < N; ++i)
            {
                // 動径における目標角と現在角の差を求める
                float diff;
                float th = motor_vel[i].theta - (theta[i] - int(theta[i]) / (2 * M_PI));
                th = th < 0 ? th + 2 * M_PI : th;
                // thの位置する象限を求める
                switch (int(th / (M_PI / 2)))
                {
                    case 0:
                        diff = th;
                        break;
                    case 1:
                    case 2:
                        diff = th - M_PI;
                        motor_vel[i].vel *= -1;
                        break;
                    case 3:
                        diff = th - 2 * M_PI;
                        break;
                    default:
                        diff = th;
                        break;
                }
                motor_vel[i].theta = theta[i] + diff;
            }
        }
        void calc_rps(float (&value)[N * 2], const std::array<SteerValue, N> &motor_vel, const float (&theta)[N], const float elapsed)
        {
            for (int i = 0; i < N; ++i)
            {
                constexpr int gear_ratio = 19;
                const float steer_vel = pid_theta_[i].calc(motor_vel[i].theta, theta[i], elapsed) * max_steer_vel_;
                // printf("steer[%d]: %f, %f, %f\n", i, steer_vel, motor_vel[i].theta, theta[i]);
                value[2 * i] = -(motor_vel[i].vel + steer_vel) / wheel_radius_ * gear_ratio;
                value[2 * i + 1] = (motor_vel[i].vel - steer_vel) / wheel_radius_ * gear_ratio;
            }
        }
        void pid(const float (&goal)[N * 2], const float elapsed)
        {
            for (int i = 0; i < N * 2; ++i)
            {
                c620_.read_data();
            }
            printf("\n");
            for (int i = 0; i < N * 2; ++i)
            {
                constexpr float k = 2 * M_PI / 60;
                const float now = c620_.get_rpm(i + 1) * k;
                // printf("%f\n", now);
                const float percent = pid_rps_[i].calc(goal[i], now, elapsed);
                if (i == i)
                {
                    // printf("pwr[%d]: %f, %f, %f\n", i, now, goal[i], percent);
                }
                c620_.set_output_percent(percent, i + 1);
            }
        }

        const PidGain gain_rps_ = {0.0008, 0.002, 0.0};
        const PidParameter param_rps_ = {gain_rps_, -1, 1};
        const PidGain gain_theta_ = {3, 0.1, 0.1};
        const PidParameter param_theta_ = {gain_theta_, -1, 1};
        dji::C620 c620_;
        std::array<Pid, N * 2> pid_rps_;
        std::array<Pid, N> pid_theta_;
        bit::SteerDrive<N> steer_;
        const int max_motor_pwr_ = 8000;
        const float max_steer_vel_ = 0.6;
        const float wheel_radius_ = 0.05;
    };
} // namespace bit


int main()
{
    constexpr float robot_radius = 0.75;
    constexpr int wheel_amount = 4;
    constexpr int enc_zero_pos[wheel_amount] = {1490, 1590, -1020, 90};

    BufferedSerial pc(USBTX, USBRX, 115200);
    bit::DiffSteer<wheel_amount> steer(robot_radius, PA_11, PA_12);
    CAN esp(PB_12, PB_13, 1e6);
    Amt21 enc[wheel_amount] = {{0x50}, {0x54}, {0x58}, {0x5C}};
    Ps5 ps5;
    bit::Velocity vel(0, 0, 0);
    
    constexpr float max_trans_vel = 1.0;
    constexpr float max_rot_vel = 1.0;
    printf("\nreset\n");
    
    for (int i = 0; i < wheel_amount; ++i)
    {
        enc[i].set_zero_pos(enc_zero_pos[i]);
    }
    std::list<int> queue = {1, 2, 3, 4, 5, 6, 7, 8};
    while (queue.empty() == 0)
    {
        queue.remove(steer.read_c620());
        for (int val: queue)
        {
            printf("%d, ", val);
        }
        printf("\n");
    }
    while (1)
    {
        auto now = HighResClock::now();
        static auto pre = now;
        steer.read_c620();
        if (ps5.read(esp))
        {
            vel.x = ps5.lstick_x / 128.0 * max_trans_vel;
            vel.y = ps5.lstick_y / 128.0 * max_trans_vel;
            vel.ang = ps5.rstick_x / 128.0 * max_rot_vel * -1;
            // printf("%f, %f, %f\n", vel.x, vel.y, vel.ang);
            if (ps5.circle)
            {
                steer.reset();
                for (auto& e: enc)
                {
                    e.set_zero_pos();
                }
            }
        }
        if (now - pre > 10ms)
        {
            float theta[wheel_amount] = {0};
            for (int i = 0; i < wheel_amount; ++i)
            {
                constexpr float enc_to_rad = 2 * M_PI / 4096;
                enc[i].request_pos();
                theta[i] = enc[i].fixed_pos * enc_to_rad / 5;
                printf("%f, ", theta[i]);
            }
            // printf("\n");
            // steer.set_output(vel, theta, 0.01);
            printf("%d\n", steer.set_output(vel, theta, 0.01));
            // printf("1\n");
            pre = now;
        }
    }
}

float duration_to_sec(const std::chrono::duration<float> &duration)
{
    return duration.count();
}