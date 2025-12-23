#include "tracker.hpp"
#include <algorithm>

namespace sa {

AlphaBetaTracker::AlphaBetaTracker(TrackerConfig cfg): cfg_(cfg) {}

void AlphaBetaTracker::ingest(const std::vector<ContactMeas>& meas)
{
    for (const auto& m : meas) {
        auto it = map_.find(m.id);
        if (it == map_.end()) {
            Track tr;
            tr.id = m.id;
            tr.last_t_s = m.t_s;
            tr.pos_enu = m.pos_enu;
            tr.vel_enu = {0,0,0};
            tr.hits = 1;
            tr.misses = 0;
            map_[m.id] = tr;
            continue;
        }

        Track& tr = it->second;
        double dt = m.t_s - tr.last_t_s;
        if (dt < 1e-3) dt = 1e-3;

        Vec3 x_pred = tr.pos_enu + tr.vel_enu * dt;
        Vec3 v_pred = tr.vel_enu;

        Vec3 r = m.pos_enu - x_pred;

        tr.pos_enu = x_pred + r * cfg_.alpha;
        tr.vel_enu = v_pred + r * (cfg_.beta / dt);

        tr.last_t_s = m.t_s;
        tr.hits += 1;
        tr.misses = 0;
    }

    for (auto& kv : map_) {
        bool seen = false;
        for (const auto& m : meas) if (m.id == kv.first) { seen = true; break; }
        if (!seen) kv.second.misses += 1;
    }
}

std::vector<Track> AlphaBetaTracker::tracks(double now_t_s) const
{
    std::vector<Track> out;
    out.reserve(map_.size());
    for (const auto& kv : map_) {
        const Track& tr = kv.second;
        if ((now_t_s - tr.last_t_s) <= cfg_.track_timeout_s) out.push_back(tr);
    }
    std::sort(out.begin(), out.end(), [](const Track& a, const Track& b){ return a.id < b.id; });
    return out;
}

} // namespace sa
