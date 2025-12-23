#pragma once
#include "types.hpp"
#include <vector>

namespace sa {

struct RiskConfig {
    double horizon_s{30.0};
    double warn_t_s{20.0};
    double crit_t_s{10.0};
    double warn_d_m{120.0};
    double crit_d_m{60.0};
};

class CpaRisk {
public:
    explicit CpaRisk(RiskConfig cfg = {});
    std::vector<Alert> evaluate(const OwnshipState& own, const std::vector<Track>& tracks);
private:
    RiskConfig cfg_;
};

} // namespace sa
