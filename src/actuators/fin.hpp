#pragma once

#include <atomic>
#include <cstdint>

#include "AS5047D.h"
#include "SPICREATE.h"
#include "esp_err.h"

namespace actuators {

enum class FinState : uint8_t { unavailable, free, hold };

struct FinTelemetry {
  FinState state{FinState::unavailable};
  bool encoder_valid{};
  bool zero_valid{};
  double angle_deg{};
  double rate_deg_s{};
};

class FinActuator {
public:
  [[nodiscard]] esp_err_t initialize();
  [[nodiscard]] esp_err_t holdCurrent();
  [[nodiscard]] esp_err_t free();
  void update(uint64_t now_us);
  void forceSafe();

  [[nodiscard]] FinTelemetry telemetry() const;

private:
  [[nodiscard]] esp_err_t initializeMotor();
  [[nodiscard]] esp_err_t drive(double duty_signed);
  [[nodiscard]] esp_err_t coast();
  [[nodiscard]] static double wrapRadians(double value);

  SPICREATE spi_{};
  AS5047D encoder_{};
  bool motor_initialized_{};
  std::atomic<FinState> state_{FinState::unavailable};
  std::atomic<bool> encoder_valid_{};
  std::atomic<bool> zero_valid_{};
  std::atomic<double> angle_rad_{};
  std::atomic<double> rate_rad_s_{};
  double zero_rad_{};
  double previous_angle_rad_{};
  uint64_t previous_timestamp_us_{};
};

} // namespace actuators
