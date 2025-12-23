#include "sim_sensor.hpp"
#include <cmath>

namespace sa {

static constexpr double PI = 3.14159265358979323846;
static constexpr double DEG2RAD = PI / 180.0;

SensorSim::SensorSim(uint32_t seed): seed_(seed)
{
    own_.t_s = 0;
    own_.pos_enu = {0, 0, 120.0};
    own_.vel_enu = {20.0, 0, 0};
    own_.yaw_rad = 0;

    truth_.push_back({101, {600,  80, 120}, {-12.0,  0.0, 0.0}});
    truth_.push_back({102, {-200, -250, 120}, {18.0,  8.0, 0.0}});
    truth_.push_back({103, {200,  0, 120}, {20.0,  0.0, 0.0}});
    truth_.push_back({104, {800, -60, 120}, {-16.0,  2.0, 0.0}});
}

double SensorSim::randu_()
{
    seed_ = 1664525u * seed_ + 1013904223u;
    return (double)(seed_ & 0x00FFFFFFu) / (double)0x01000000u;
}

double SensorSim::randn_()
{
    double u1 = randu_();
    double u2 = randu_();
    if (u1 < 1e-12) u1 = 1e-12;
    double r = std::sqrt(-2.0 * std::log(u1));
    double th = 2.0 * PI * u2;
    return r * std::cos(th);
}

void SensorSim::step(double dt)
{
    t_ += dt;
    own_.t_s = t_;

    own_.yaw_rad = 2.0 * DEG2RAD * std::sin(t_ * 0.1);
    own_.vel_enu = {20.0, 0.5 * std::sin(t_ * 0.2), 0.0};
    own_.pos_enu = own_.pos_enu + own_.vel_enu * dt;

    for (auto& c : truth_) {
        c.pos = c.pos + c.vel * dt;
    }

    meas_.clear();
    for (const auto& c : truth_) {
        if (randu_() < drop_prob_) continue;
        ContactMeas m;
        m.id = c.id;
        m.t_s = t_;
        m.pos_enu = {
            c.pos.x + randn_() * pos_noise_std_,
            c.pos.y + randn_() * pos_noise_std_,
            c.pos.z + randn_() * (pos_noise_std_ * 0.5)
        };
        meas_.push_back(m);
    }
}

} // namespace sa
