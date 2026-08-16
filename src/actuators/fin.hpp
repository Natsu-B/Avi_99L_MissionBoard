#pragma once

#include <atomic>
#include <cstdint>

#include "AS5047D.h"
#include "SPICREATE.h"
#include "esp_err.h"

namespace actuators {

enum class FinState : uint8_t { unavailable, free, zero_hold, roll_control };

struct FinTelemetry {
  FinState state{FinState::unavailable};
  bool encoder_valid{};
  bool rate_valid{};
  bool zero_valid{};
  double angle_deg{};
  double rate_deg_s{};
  uint64_t sample_timestamp_us{};
};

class FinActuator {
public:
  [[nodiscard]] esp_err_t initialize();
  [[nodiscard]] esp_err_t holdCurrent();
  [[nodiscard]] esp_err_t zeroHold();
  [[nodiscard]] esp_err_t setRollControlTorque(double torque_nm);
  [[nodiscard]] esp_err_t free();
  void update(uint64_t now_us);
  void forceSafe();

  [[nodiscard]] FinTelemetry telemetry() const;

private:
  [[nodiscard]] esp_err_t initializeMotor();
  [[nodiscard]] esp_err_t drive(double duty_signed);
  [[nodiscard]] esp_err_t coast();
  [[nodiscard]] esp_err_t applyOutputTorque(double torque_nm,
                                            double angle_rad,
                                            double rate_rad_s);
  [[nodiscard]] double zeroHoldTorque(double angle_rad,
                                      double rate_rad_s) const;
  [[nodiscard]] static double wrapRadians(double value);

  SPICREATE spi_{};
  AS5047D encoder_{};
  bool motor_initialized_{};
  std::atomic<FinState> state_{FinState::unavailable};
  std::atomic<bool> encoder_valid_{};
  std::atomic<bool> rate_valid_{};
  std::atomic<bool> zero_valid_{};
  std::atomic<double> angle_rad_{};
  std::atomic<double> rate_rad_s_{};
  std::atomic<uint64_t> sample_timestamp_us_{};
  double zero_rad_{};
  double previous_angle_rad_{};
  uint64_t previous_timestamp_us_{};
  double requested_roll_torque_nm_{};
};

} // namespace actuators
