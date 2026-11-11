#include "protocol.h"
#include <cassert>
#include <limits>
int main() {
    using namespace vut_bridge;
    Packet p{};
    std::memcpy(p.magic, "VUT1", 4); p.version = 1; p.flags = 3;
    p.sent_ns = p.pose_ns = 1000000000ULL;
    std::strcpy(p.serial, "FA61N3B00314");
    p.position[0] = 0.25; p.position[1] = 1.5; p.position[2] = -0.8;
    p.xyzw[3] = 1;
    assert(sane(p));
    auto out = to_pose(p, 1010000000ULL);
    assert(out.poseIsValid && out.deviceIsConnected && out.vecPosition[0] == 0.25);
    assert(out.vecPosition[1] == 1.5 && out.vecPosition[2] == -0.8);
    assert(out.qWorldFromDriverRotation.w == 1 && out.qRotation.w == 1);
    assert(out.poseTimeOffset < 0);
    assert(!to_pose(p, 1300000000ULL).deviceIsConnected);
    p.sent_ns = 1300000000ULL;
    assert(to_pose(p, p.sent_ns).deviceIsConnected && !to_pose(p, p.sent_ns).poseIsValid);
    p.sent_ns = p.pose_ns = 1000000000ULL;
    assert(!to_pose(p, 900000000ULL).poseIsValid);
    p.flags = 1; assert(!to_pose(p, p.sent_ns).poseIsValid);
    p.flags = 2; assert(!to_pose(p, p.sent_ns).poseIsValid);
    p.flags = 8; assert(!sane(p)); p.flags = 3;
    p.position[0] = std::numeric_limits<double>::quiet_NaN(); assert(!sane(p)); p.position[0] = 0;
    p.xyzw[3] = 0; assert(!sane(p)); p.xyzw[3] = 1;
    std::memset(p.serial, 'a', sizeof(p.serial)); assert(!sane(p));
    std::strcpy(p.serial, "bad/serial"); assert(!sane(p));
}
