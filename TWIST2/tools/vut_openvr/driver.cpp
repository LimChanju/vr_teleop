// Separate adapter; upstream xi-ve, Valve and HTC source files remain untouched.
#include "protocol.h"
#include <algorithm>
#include <array>
#include <cerrno>
#include <cstddef>
#include <map>
#include <memory>
#include <mutex>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace {
class Tracker : public vr::ITrackedDeviceServerDriver {
    std::string serial_;
    uint32_t index_ = vr::k_unTrackedDeviceIndexInvalid;
    vut_bridge::Packet packet_{};
    std::mutex mutex_;
public:
    explicit Tracker(std::string serial) : serial_(std::move(serial)) {}
    vr::EVRInitError Activate(uint32_t index) override {
        index_ = index;
        const auto handle = vr::VRProperties()->TrackedDeviceToPropertyContainer(index);
        auto props = vr::VRProperties();
        props->SetStringProperty(handle, vr::Prop_TrackingSystemName_String, "twist2_vut");
        props->SetStringProperty(handle, vr::Prop_ModelNumber_String, "VIVE Ultimate (VUT bridge)");
        props->SetStringProperty(handle, vr::Prop_SerialNumber_String, serial_.c_str());
        props->SetStringProperty(handle, vr::Prop_ManufacturerName_String, "HTC / local VUT adapter");
        props->SetStringProperty(handle, vr::Prop_RegisteredDeviceType_String, ("twist2_vut/" + serial_).c_str());
        props->SetStringProperty(handle, vr::Prop_ControllerType_String, "vive_tracker");
        props->SetStringProperty(handle, vr::Prop_RenderModelName_String, "{htc}vr_tracker_vive_1_0");
        props->SetInt32Property(handle, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_GenericTracker);
        props->SetUint64Property(handle, vr::Prop_CurrentUniverseId_Uint64, 0x56555401ULL);
        props->SetBoolProperty(handle, vr::Prop_WillDriftInYaw_Bool, false);
        return vr::VRInitError_None;
    }
    void Deactivate() override { index_ = vr::k_unTrackedDeviceIndexInvalid; }
    void EnterStandby() override {}
    void* GetComponent(const char*) override { return nullptr; }
    void DebugRequest(const char*, char* response, uint32_t size) override { if (size) response[0] = '\0'; }
    vr::DriverPose_t GetPose() override {
        std::lock_guard<std::mutex> guard(mutex_);
        return vut_bridge::to_pose(packet_, vut_bridge::now_ns());
    }
    void update(const vut_bridge::Packet& p) {
        std::lock_guard<std::mutex> guard(mutex_);
        if (p.sent_ns > packet_.sent_ns) packet_ = p;
    }
    void publish() {
        if (index_ != vr::k_unTrackedDeviceIndexInvalid) {
            auto pose = GetPose();
            vr::VRServerDriverHost()->TrackedDevicePoseUpdated(index_, pose, sizeof(pose));
        }
    }
};

class Provider : public vr::IServerTrackedDeviceProvider {
    int socket_ = -1;
    std::map<std::string, std::unique_ptr<Tracker>> trackers_;
public:
    vr::EVRInitError Init(vr::IVRDriverContext* context) override {
        VR_INIT_SERVER_DRIVER_CONTEXT(context);
        socket_ = socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        int enabled = 1;
        if (socket_ < 0 || setsockopt(socket_, SOL_SOCKET, SO_PASSCRED, &enabled, sizeof(enabled)) < 0) {
            Cleanup(); return vr::VRInitError_Driver_Failed;
        }
        // Abstract local socket: no filesystem unlink, and sender UID is checked below.
        sockaddr_un address{};
        address.sun_family = AF_UNIX;
        const std::string name = "twist2_vut_" + std::to_string(getuid());
        std::memcpy(address.sun_path + 1, name.data(), name.size());
        if (bind(socket_, reinterpret_cast<sockaddr*>(&address),
                 offsetof(sockaddr_un, sun_path) + 1 + name.size()) < 0) {
            vr::VRDriverLog()->Log("twist2_vut: failed to bind local pose socket");
            Cleanup(); return vr::VRInitError_Driver_Failed;
        }
        vr::VRDriverLog()->Log("twist2_vut: ready; poses are UNCALIBRATED VIVE world; use Space Calibrator");
        return vr::VRInitError_None;
    }
    void Cleanup() override {
        if (socket_ >= 0) { close(socket_); socket_ = -1; }
        trackers_.clear();
        VR_CLEANUP_SERVER_DRIVER_CONTEXT();
    }
    const char* const* GetInterfaceVersions() override { return vr::k_InterfaceVersions; }
    void RunFrame() override {
        if (socket_ < 0) return;
        for (int count = 0; count < 512; ++count) {
            vut_bridge::Packet packet{};
            iovec io{&packet, sizeof(packet)};
            alignas(cmsghdr) char credentials[CMSG_SPACE(sizeof(ucred))]{};
            msghdr message{};
            message.msg_iov = &io; message.msg_iovlen = 1;
            message.msg_control = credentials; message.msg_controllen = sizeof(credentials);
            const auto n = recvmsg(socket_, &message, MSG_DONTWAIT);
            if (n < 0) break;
            bool own_user = false;
            for (auto c = CMSG_FIRSTHDR(&message); c; c = CMSG_NXTHDR(&message, c)) {
                if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SCM_CREDENTIALS &&
                    c->cmsg_len >= CMSG_LEN(sizeof(ucred))) {
                    ucred peer{}; std::memcpy(&peer, CMSG_DATA(c), sizeof(peer));
                    own_user = peer.uid == getuid();
                }
            }
            if (!own_user || (message.msg_flags & (MSG_TRUNC | MSG_CTRUNC)) ||
                n != sizeof(packet) || !vut_bridge::sane(packet) ||
                !vut_bridge::fresh(packet.sent_ns, vut_bridge::now_ns())) continue;
            const std::string serial(packet.serial);
            if (!trackers_.count(serial)) {
                if (trackers_.size() >= 5) continue;
                auto tracker = std::make_unique<Tracker>(serial);
                tracker->update(packet);
                if (!vr::VRServerDriverHost()->TrackedDeviceAdded(serial.c_str(),
                        vr::TrackedDeviceClass_GenericTracker, tracker.get())) continue;
                trackers_.emplace(serial, std::move(tracker));
                vr::VRDriverLog()->Log(("twist2_vut: registered " + serial).c_str());
            } else trackers_.at(serial)->update(packet);
        }
        for (auto& tracker : trackers_) tracker.second->publish();
    }
    bool ShouldBlockStandbyMode() override { return false; }
    void EnterStandby() override {}
    void LeaveStandby() override {}
};
Provider provider;
}
extern "C" __attribute__((visibility("default"))) void* HmdDriverFactory(const char* version, int* code) {
    if (std::strcmp(version, vr::IServerTrackedDeviceProvider_Version) == 0) return &provider;
    if (code) *code = vr::VRInitError_Init_InterfaceNotFound;
    return nullptr;
}
