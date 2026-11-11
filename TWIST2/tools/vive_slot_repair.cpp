// Minimal interactive slot repair utility for HTC's official Linux SDK.
//
// This intentionally exposes only the commands needed to inspect and repair
// the two problematic dongle slots.  It does not depend on the VIVE Hub UI,
// OpenCV, GLFW, or GLEW.

#include "ultimate_tracker.h"

#include <atomic>
#include <cctype>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

std::atomic<bool> g_running{true};
std::mutex g_output_mutex;

const char* state_name(vive::TrackerState state) {
  switch (state) {
    case vive::Unpaired:
      return "NA/Unpaired";
    case vive::Pairing:
      return "Pairing";
    case vive::Paired:
      return "Paired";
    case vive::Connecting:
      return "Connecting";
    case vive::Connected:
      return "Connected";
    case vive::Initialized:
      return "Initialized";
    case vive::Syncing:
      return "Syncing";
    case vive::BuildMap:
      return "BuildMap";
    case vive::Ready:
      return "Ready";
  }
  return "Unknown";
}

std::string trim(std::string value) {
  while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front()))) {
    value.erase(value.begin());
  }
  while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back()))) {
    value.pop_back();
  }
  return value;
}

std::string lowercase(std::string value) {
  for (char& ch : value) {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }
  return value;
}

bool parse_slot_list(const std::string& text, std::vector<uint32_t>* slots) {
  slots->clear();
  std::stringstream stream(text);
  std::string token;
  while (std::getline(stream, token, ',')) {
    token = trim(token);
    if (token.size() != 1 || token[0] < '0' || token[0] > '4') {
      return false;
    }
    slots->push_back(static_cast<uint32_t>(token[0] - '0'));
  }
  return !slots->empty();
}

bool repair_slots_only(const std::vector<uint32_t>& slots) {
  for (uint32_t slot : slots) {
    if (slot != 0 && slot != 3) {
      return false;
    }
  }
  return true;
}

void print_help() {
  std::lock_guard<std::mutex> lock(g_output_mutex);
  std::cout
      << "Commands:\n"
      << "  status             Show all five SDK slot states\n"
      << "  unpair 0,3         Clear only the protected repair slots\n"
      << "  pair 0             Pair one tracker into SDK slot 0\n"
      << "  pair 3             Pair one tracker into SDK slot 3\n"
      << "  restart all        Restart tracker connections without unpairing\n"
      << "  echo 0,3           Flash trackers in the selected slots\n"
      << "  help               Show this help\n"
      << "  quit               Stop the SDK server and exit\n"
      << "\n"
      << "Safety: pair/unpair commands for slots 1, 2, and 4 are rejected.\n";
}

class SlotRepair final : public vive::ITrackerSubscriber {
 public:
  bool send(const std::string& command) {
    if (!SendCommand(command.c_str(), static_cast<int>(command.size()))) {
      std::lock_guard<std::mutex> lock(g_output_mutex);
      std::cerr << "[send] rejected: " << command << '\n';
      return false;
    }
    std::lock_guard<std::mutex> lock(g_output_mutex);
    std::cout << "[send] " << command << '\n';
    return true;
  }

 private:
  void ServerStart() override {
    std::lock_guard<std::mutex> lock(g_output_mutex);
    std::cout << "[server] started\n";
  }

  void ServerStop() override {
    g_running = false;
    std::lock_guard<std::mutex> lock(g_output_mutex);
    std::cout << "[server] stopped\n";
  }

  void Add(uint32_t tracker_id, const char* name) override {
    std::lock_guard<std::mutex> lock(g_output_mutex);
    std::cout << "[add] slot=" << tracker_id << " serial="
              << ((name && *name) ? name : "unknown") << '\n';
  }

  void Update(const vive::TrackerData& data) override {
    (void)data;
  }

  void Click(uint32_t tracker_id, int button_id, int clicks) override {
    std::lock_guard<std::mutex> lock(g_output_mutex);
    std::cout << "[button] slot=" << tracker_id << " button=" << button_id
              << " clicks=" << clicks << '\n';
  }

  void Remove(uint32_t tracker_id) override {
    std::lock_guard<std::mutex> lock(g_output_mutex);
    std::cout << "[remove] slot=" << tracker_id << '\n';
  }

  void CommandResponse(const vive::TrackerCommandResponse& response) override {
    std::lock_guard<std::mutex> lock(g_output_mutex);
    std::cout << "[response] command="
              << (response.Command ? response.Command : "") << " result="
              << (response.Result ? response.Result : "") << '\n';
    for (uint32_t slot = 0; slot < response.NumTrackers; ++slot) {
      std::cout << "  slot " << slot << ": "
                << state_name(response.TrackerStates[slot]) << " ("
                << static_cast<unsigned>(response.TrackerStates[slot]) << ")\n";
    }
  }
};

void handle_signal(int) {
  g_running = false;
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  std::cout << "VIVE slot repair using official SDK " << vive::GetVersion()
            << " (max slots=" << vive::GetNumTrackers() << ")\n";
  std::cout << "Do not run VIVE Hub or tracker_server at the same time.\n";

  SlotRepair repair;
  if (!vive::StartTrackerServer(argc, argv)) {
    std::cerr << "Failed to start the official tracker server. "
                 "Close VIVE Hub and check dongle permissions.\n";
    return 1;
  }

  print_help();
  repair.send("status");

  std::string line;
  while (g_running) {
    std::cout << "vive-slot> " << std::flush;
    if (!std::getline(std::cin, line)) {
      break;
    }
    line = trim(line);
    if (line.empty()) {
      continue;
    }

    const std::string lower = lowercase(line);
    if (lower == "quit" || lower == "exit") {
      break;
    }
    if (lower == "help") {
      print_help();
      continue;
    }
    if (lower == "status") {
      repair.send("status");
      continue;
    }
    if (lower == "restart all") {
      repair.send("Restart all");
      continue;
    }

    const auto separator = lower.find(' ');
    if (separator == std::string::npos) {
      std::cerr << "Unknown command. Type 'help'.\n";
      continue;
    }
    const std::string verb = lower.substr(0, separator);
    const std::string slot_text = trim(lower.substr(separator + 1));
    std::vector<uint32_t> slots;
    if (!parse_slot_list(slot_text, &slots)) {
      std::cerr << "Invalid slot list. Use comma-separated IDs from 0 to 4.\n";
      continue;
    }

    if (verb == "pair" || verb == "pairing" || verb == "unpair") {
      if (!repair_slots_only(slots)) {
        std::cerr << "Safety refusal: only repair slots 0 and 3 may be paired or unpaired.\n";
        continue;
      }
      repair.send((verb == "unpair" ? "Unpair " : "Pairing ") + slot_text);
      continue;
    }
    if (verb == "echo") {
      repair.send("Echo " + slot_text);
      continue;
    }

    std::cerr << "Unknown command. Type 'help'.\n";
  }

  vive::StopTrackerServer();
  std::cout << "VIVE slot repair stopped.\n";
  return 0;
}
