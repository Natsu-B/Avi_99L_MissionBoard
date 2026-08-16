#pragma once

#include <atomic>
#include <cstdint>

namespace diagnostics {

enum class DeviceState : uint8_t {
  unavailable = 0,
  healthy = 1,
  degraded = 2,
  recovering = 3,
  failed = 4,
};

class DeviceHealth {
public:
  [[nodiscard]] DeviceState state() const {
    return state_.load(std::memory_order_acquire);
  }

  [[nodiscard]] uint8_t consecutiveFailures() const {
    return failures_.load(std::memory_order_acquire);
  }

  [[nodiscard]] bool recoveryRequested() const {
    const auto current = state();
    return current == DeviceState::unavailable ||
           current == DeviceState::failed;
  }

  void markUnavailable() {
    failures_.store(0, std::memory_order_release);
    state_.store(DeviceState::unavailable, std::memory_order_release);
  }

  void markHealthy() {
    failures_.store(0, std::memory_order_release);
    state_.store(DeviceState::healthy, std::memory_order_release);
  }

  void markRecovering() {
    state_.store(DeviceState::recovering, std::memory_order_release);
  }

  void markFailed() {
    state_.store(DeviceState::failed, std::memory_order_release);
  }

  void markFailure(uint8_t threshold) {
    uint8_t current = failures_.load(std::memory_order_acquire);
    if (current != UINT8_MAX)
      ++current;
    failures_.store(current, std::memory_order_release);
    state_.store(current >= threshold ? DeviceState::failed
                                      : DeviceState::degraded,
                 std::memory_order_release);
  }

private:
  std::atomic<DeviceState> state_{DeviceState::unavailable};
  std::atomic<uint8_t> failures_{0};
};

} // namespace diagnostics
