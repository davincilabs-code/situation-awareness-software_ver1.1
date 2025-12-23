#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace sa {

struct Vec3 { double x{0}, y{0}, z{0}; };
inline Vec3 operator+(const Vec3& a, const Vec3& b){ return {a.x+b.x, a.y+b.y, a.z+b.z}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b){ return {a.x-b.x, a.y-b.y, a.z-b.z}; }
inline Vec3 operator*(const Vec3& a, double s){ return {a.x*s, a.y*s, a.z*s}; }

double dot(const Vec3& a, const Vec3& b);
double norm(const Vec3& a);

struct OwnshipState {
    double t_s{0};
    Vec3 pos_enu;
    Vec3 vel_enu;
    double yaw_rad{0};
};

struct ContactMeas {
    int id{0};
    double t_s{0};
    Vec3 pos_enu;
};

struct Track {
    int id{0};
    double last_t_s{0};
    Vec3 pos_enu;
    Vec3 vel_enu;
    int hits{0};
    int misses{0};
};

struct Alert {
    int id{0};
    double t_s{0};
    double t_cpa_s{0};
    double d_cpa_m{0};
    std::string level; // WARN/CRIT
};

} // namespace sa
