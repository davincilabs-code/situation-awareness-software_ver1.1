#pragma once
#include <string>
#include <cstdint>

namespace sa {

class UdpPublisher {
public:
    UdpPublisher();
    ~UdpPublisher();
    bool open(const std::string& ip, uint16_t port);
    void send_line(const std::string& line);
    bool enabled() const { return enabled_; }
private:
    bool enabled_{false};
#ifdef _WIN32
    void* sock_{nullptr};
#else
    int sock_{-1};
#endif
    std::string ip_;
    uint16_t port_{0};
};

} // namespace sa
