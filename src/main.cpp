#include "types.hpp"
#include "sim_sensor.hpp"
#include "tracker.hpp"
#include "risk.hpp"
#include "udp_pub.hpp"
#include "json_min.hpp"

#include <chrono>
#include <thread>
#include <iostream>
#include <sstream>
#include <algorithm>

using namespace sa;

static void usage(){
    std::cout << "Usage: uam_sa_demo [--rate Hz] [--duration sec] [--udp ip port]\n";
}

static bool eq(const char* a, const char* b){ return std::string(a)==std::string(b); }

static std::string state_json(const OwnshipState& own){
    std::ostringstream o;
    o << "{"
      << "\"type\":\"state\","
      << "\"t_s\":" << jnum(own.t_s,3) << ","
      << "\"pos_enu\":[" << jnum(own.pos_enu.x) << "," << jnum(own.pos_enu.y) << "," << jnum(own.pos_enu.z) << "],"
      << "\"vel_enu\":[" << jnum(own.vel_enu.x) << "," << jnum(own.vel_enu.y) << "," << jnum(own.vel_enu.z) << "]"
      << "}";
    return o.str();
}

static std::string tracks_json(double t_s, const std::vector<Track>& trks){
    std::ostringstream o;
    o << "{"
      << "\"type\":\"tracks\","
      << "\"t_s\":" << jnum(t_s,3) << ","
      << "\"tracks\":[";
    for (size_t i=0;i<trks.size();++i){
        const auto& tr = trks[i];
        o << "{"
          << "\"id\":" << tr.id << ","
          << "\"pos_enu\":[" << jnum(tr.pos_enu.x) << "," << jnum(tr.pos_enu.y) << "," << jnum(tr.pos_enu.z) << "],"
          << "\"vel_enu\":[" << jnum(tr.vel_enu.x) << "," << jnum(tr.vel_enu.y) << "," << jnum(tr.vel_enu.z) << "],"
          << "\"age_s\":" << jnum(t_s - tr.last_t_s,3) << ","
          << "\"hits\":" << tr.hits << ","
          << "\"misses\":" << tr.misses
          << "}";
        if (i+1<trks.size()) o << ",";
    }
    o << "]}";
    return o.str();
}

static std::string alerts_json(double t_s, const std::vector<Alert>& alerts){
    std::ostringstream o;
    o << "{"
      << "\"type\":\"alerts\","
      << "\"t_s\":" << jnum(t_s,3) << ","
      << "\"alerts\":[";
    for (size_t i=0;i<alerts.size();++i){
        const auto& a = alerts[i];
        o << "{"
          << "\"id\":" << a.id << ","
          << "\"level\":\"" << esc(a.level) << "\","
          << "\"t_cpa_s\":" << jnum(a.t_cpa_s,3) << ","
          << "\"d_cpa_m\":" << jnum(a.d_cpa_m,2)
          << "}";
        if (i+1<alerts.size()) o << ",";
    }
    o << "]}";
    return o.str();
}

int main(int argc, char** argv){
    int rate_hz = 20;
    double duration_s = 15.0;
    bool use_udp = false;
    std::string udp_ip = "127.0.0.1";
    uint16_t udp_port = 18000;

    for (int i=1;i<argc;i++){
        if (eq(argv[i], "--help") || eq(argv[i], "-h")) { usage(); return 0; }
        if (eq(argv[i], "--rate") && i+1<argc) { rate_hz = std::max(1, std::atoi(argv[++i])); continue; }
        if (eq(argv[i], "--duration") && i+1<argc) { duration_s = std::max(0.5, std::atof(argv[++i])); continue; }
        if (eq(argv[i], "--udp") && i+2<argc) {
            use_udp = true;
            udp_ip = argv[++i];
            udp_port = (uint16_t)std::atoi(argv[++i]);
            continue;
        }
        std::cerr << "Unknown arg: " << argv[i] << "\n";
        usage();
        return 1;
    }

    double dt = 1.0 / (double)rate_hz;

    SensorSim sim(42);
    sim.set_noise_std(3.0);
    sim.set_drop_prob(0.07);

    AlphaBetaTracker tracker({0.85, 0.10, 3.0});
    CpaRisk risk({30.0, 20.0, 10.0, 120.0, 60.0});

    UdpPublisher udp;
    if (use_udp) {
        if (!udp.open(udp_ip, udp_port)) {
            std::cerr << "UDP open failed\n";
            return 2;
        }
        std::cerr << "UDP enabled -> " << udp_ip << ":" << udp_port << "\n";
    }

    auto t0 = std::chrono::steady_clock::now();
    for (;;) {
        sim.step(dt);

        tracker.ingest(sim.contacts());
        auto trks = tracker.tracks(sim.ownship().t_s);
        auto alerts = risk.evaluate(sim.ownship(), trks);

        std::string s1 = state_json(sim.ownship());
        std::string s2 = tracks_json(sim.ownship().t_s, trks);

        std::cout << s1 << "\n" << s2 << "\n";
        if (!alerts.empty()) std::cout << alerts_json(sim.ownship().t_s, alerts) << "\n";
        std::cout.flush();

        if (udp.enabled()) {
            udp.send_line(s1);
            udp.send_line(s2);
            if (!alerts.empty()) udp.send_line(alerts_json(sim.ownship().t_s, alerts));
        }

        double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if (elapsed >= duration_s) break;

        std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    }

    return 0;
}
