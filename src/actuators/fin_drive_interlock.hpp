#pragma once

#include <atomic>
#include <cstdint>

namespace actuators {

// Safety taskが一度latchした後は、旧snapshotを使ったrealtime tickも
// 非0 driveを再開できない。0 commandはcoast処理のため常に許可する。
class FinDriveInterlock {
public:
  void resetForInitialization() {
    power_cutoff_latched_.store(false, std::memory_order_release);
    recoverable_fault_.store(false, std::memory_order_release);
  }
  void latch() {
    power_cutoff_latched_.store(true, std::memory_order_release);
  }
  // Recoverable device faultは復旧成功までdriveを止めるが、power-cutoff
  // latchとは独立にする。recoveryはpower-cutoffを解除しない。
  void noteRecoverableFault() {
    recoverable_fault_.store(true, std::memory_order_release);
  }
  void noteSuccessfulRecovery() {
    recoverable_fault_.store(false, std::memory_order_release);
  }
  void clearForNewEpoch() {
    power_cutoff_latched_.store(false, std::memory_order_release);
  }
  [[nodiscard]] bool inhibited() const {
    return power_cutoff_latched_.load(std::memory_order_acquire) ||
           recoverable_fault_.load(std::memory_order_acquire);
  }
  [[nodiscard]] bool allows(int16_t command) const {
    return command == 0 || !inhibited();
  }

private:
  std::atomic<bool> power_cutoff_latched_{};
  std::atomic<bool> recoverable_fault_{};
};

} // namespace actuators
