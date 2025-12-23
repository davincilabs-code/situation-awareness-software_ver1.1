#pragma once
#include "types.hpp"
#include <vector>

namespace sa {

class SensorSim {
public:
    explicit SensorSim(uint32_t seed=1);
    void step(double dt);

    const OwnshipState& ownship() const { return own_; }
    const std::vector<ContactMeas>& contacts() const { return meas_; }

    void set_noise_std(double pos_std_m) { pos_noise_std_ = pos_std_m; }
    void set_drop_prob(double p) { drop_prob_ = p; }

private:
    uint32_t seed_{1};
    double t_{0};
    OwnshipState own_;
    std::vector<ContactMeas> meas_;

    struct TruthContact { int id; Vec3 pos; Vec3 vel; };
    std::vector<TruthContact> truth_;

    double pos_noise_std_{3.0};
    double drop_prob_{0.05};

    double randn_();
    double randu_();
};

} // namespace sa
