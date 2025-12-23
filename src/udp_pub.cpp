#include "udp_pub.hpp"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace sa {

UdpPublisher::UdpPublisher()
{
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2,2), &wsa) == 0) {
        sock_ = (void*)INVALID_SOCKET;
    }
#endif
}

UdpPublisher::~UdpPublisher()
{
#ifdef _WIN32
    if (sock_ && sock_ != (void*)INVALID_SOCKET) closesocket((SOCKET)sock_);
    WSACleanup();
#else
    if (sock_ >= 0) close(sock_);
#endif
}

bool UdpPublisher::open(const std::string& ip, uint16_t port)
{
    ip_ = ip;
    port_ = port;

#ifdef _WIN32
    SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s == INVALID_SOCKET) return false;
    sock_ = (void*)s;
#else
    int s = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) return false;
    sock_ = s;
#endif
    enabled_ = true;
    return true;
}

void UdpPublisher::send_line(const std::string& line)
{
    if (!enabled_) return;

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
#ifdef _WIN32
    InetPtonA(AF_INET, ip_.c_str(), &addr.sin_addr);
    sendto((SOCKET)sock_, line.c_str(), (int)line.size(), 0, (sockaddr*)&addr, (int)sizeof(addr));
#else
    inet_pton(AF_INET, ip_.c_str(), &addr.sin_addr);
    sendto(sock_, line.c_str(), line.size(), 0, (sockaddr*)&addr, sizeof(addr));
#endif
}

} // namespace sa
