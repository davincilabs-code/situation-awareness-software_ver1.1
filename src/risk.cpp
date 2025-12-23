#include "risk.hpp"
#include <algorithm>
#include <cmath>

namespace sa {

double dot(const Vec3& a, const Vec3& b){ return a.x*b.x + a.y*b.y + a.z*b.z; }
double norm(const Vec3& a){ return std::sqrt(dot(a,a)); }

CpaRisk::CpaRisk(RiskConfig cfg): cfg_(cfg) {}

std::vector<Alert> CpaRisk::evaluate(const OwnshipState& own, const std::vector<Track>& tracks)
{
    std::vector<Alert> alerts;

    for (const auto& tr : tracks) {
        Vec3 r = tr.pos_enu - own.pos_enu;
        Vec3 v = tr.vel_enu - own.vel_enu;

        double v2 = dot(v, v);
        double t_cpa = 0.0;
        if (v2 > 1e-6) t_cpa = -dot(r, v) / v2;
        if (t_cpa < 0.0) t_cpa = 0.0;
        if (t_cpa > cfg_.horizon_s) t_cpa = cfg_.horizon_s;

        double d_cpa = norm(r + v * t_cpa);

        std::string level;
        if (t_cpa <= cfg_.crit_t_s && d_cpa <= cfg_.crit_d_m) level = "CRIT";
        else if (t_cpa <= cfg_.warn_t_s && d_cpa <= cfg_.warn_d_m) level = "WARN";
        else continue;

        alerts.push_back(Alert{tr.id, own.t_s, t_cpa, d_cpa, level});
    }

    auto sev = [](const std::string& lvl){ return (lvl=="CRIT")?2:(lvl=="WARN")?1:0; };
    std::sort(alerts.begin(), alerts.end(), [&](const Alert& a, const Alert& b){
        int sa = sev(a.level), sb = sev(b.level);
        if (sa != sb) return sa > sb;
        return a.d_cpa_m < b.d_cpa_m;
    });

    return alerts;
}

} // namespace sa
