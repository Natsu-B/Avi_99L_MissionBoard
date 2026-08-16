#pragma once

#include <atomic>
#include <cstdint>

namespace mission {

enum class Phase : uint8_t {
  command_receive = 0,
  liftoff_detection = 1,
  flight = 2,
  descent = 3,
};

struct Snapshot {
  Phase phase{Phase::command_receive};
  uint32_t generation{};
  bool liftoff_valid{};
  uint64_t liftoff_us{};
  bool deployment_started{};
  bool power_cutoff{};
};

class StateMachine {
public:
  [[nodiscard]] bool startSequence();
  [[nodiscard]] bool reportLiftoff(uint64_t detected_us);
  [[nodiscard]] bool liftoffEmergencyRollback();
  [[nodiscard]] bool requestDescent(uint32_t generation);
  void latchPowerCutoff();

  [[nodiscard]] Snapshot snapshot() const;

private:
  std::atomic<Phase> phase_{Phase::command_receive};
  std::atomic<uint32_t> generation_{};
  std::atomic<bool> liftoff_valid_{};
  std::atomic<uint64_t> liftoff_us_{};
  std::atomic<bool> deployment_started_{};
  std::atomic<bool> power_cutoff_{};
  std::atomic<bool> liftoff_claimed_{};
};

} // namespace mission
