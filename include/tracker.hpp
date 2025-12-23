#pragma once
#include "types.hpp"
#include <unordered_map>
#include <vector>

namespace sa {

struct TrackerConfig {
    double alpha{0.85};
    double beta{0.10};
    double track_timeout_s{3.0};
};

class AlphaBetaTracker {
public:
    explicit AlphaBetaTracker(TrackerConfig cfg = {});
    void ingest(const std::vector<ContactMeas>& meas);
    std::vector<Track> tracks(double now_t_s) const;
private:
    TrackerConfig cfg_;
    std::unordered_map<int, Track> map_;
};

} // namespace sa
