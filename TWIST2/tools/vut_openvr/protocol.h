// Local transport only. No axis remapping or spatial calibration in this bridge.
#pragma once
#include <openvr_driver.h>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <time.h>

namespace vut_bridge {
struct Packet {
    char magic[4];
    uint32_t version;
    uint32_t flags; // bit 0 connected, bit 1 valid Ready pose
    uint32_t reserved; // explicit alignment; Python encodes four zero padding bytes
    uint64_t sent_ns;
    uint64_t pose_ns;
    char serial[32];
    double position[3];
    double xyzw[4];
};
static_assert(sizeof(Packet) == 120, "Python/C++ bridge packet mismatch");
static_assert(offsetof(Packet, position) == 64 && offsetof(Packet, xyzw) == 88,
              "Bridge double fields must be naturally aligned");

inline uint64_t now_ns() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint64_t(ts.tv_sec) * 1000000000ULL + ts.tv_nsec;
}
inline bool fresh(uint64_t timestamp, uint64_t now) {
    const double age = (double(now) - double(timestamp)) * 1e-9;
    return timestamp != 0 && age >= -0.01 && age <= 0.20;
}
inline bool sane(const Packet& p) {
    if (std::memcmp(p.magic, "VUT1", 4) || p.version != 1 || p.reserved || (p.flags & ~3U)) return false;
    const size_t n = strnlen(p.serial, sizeof(p.serial));
    if (n == 0 || n == sizeof(p.serial)) return false;
    for (size_t i = 0; i < n; ++i) {
        const char c = p.serial[i];
        if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
              (c >= '0' && c <= '9') || c == '-' || c == '_')) return false;
    }
    for (double v : p.position) if (!std::isfinite(v)) return false;
    double norm2 = 0;
    for (double v : p.xyzw) { if (!std::isfinite(v)) return false; norm2 += v*v; }
    return std::isfinite(norm2) && norm2 > 1e-12;
}
inline vr::DriverPose_t to_pose(const Packet& p, uint64_t now) {
    vr::DriverPose_t out{};
    out.qWorldFromDriverRotation.w = 1;
    out.qDriverFromHeadRotation.w = 1;
    out.qRotation.w = 1;
    out.result = vr::TrackingResult_Uninitialized;
    if (!sane(p)) return out;
    out.deviceIsConnected = (p.flags & 1U) && fresh(p.sent_ns, now);
    out.poseIsValid = out.deviceIsConnected && (p.flags & 2U) && fresh(p.pose_ns, now);
    out.result = out.poseIsValid ? vr::TrackingResult_Running_OK : vr::TrackingResult_Running_OutOfRange;
    out.poseTimeOffset = out.poseIsValid ? (double(p.pose_ns) - double(now)) * 1e-9 : 0;
    std::memcpy(out.vecPosition, p.position, sizeof(p.position));
    double norm = 0;
    for (double v : p.xyzw) norm += v*v;
    norm = std::sqrt(norm);
    out.qRotation = {p.xyzw[3]/norm, p.xyzw[0]/norm, p.xyzw[1]/norm, p.xyzw[2]/norm};
    // Velocities intentionally zero: no extrapolation of unavailable VUT velocities.
    return out;
}
} // namespace vut_bridge
