#pragma once

#include <atomic>
#include <cstdint>

#include "AS5047D.h"
#include "SPICREATE.h"
#include "control/zero_hold.hpp"
#include "diagnostics/device_health.hpp"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace actuators {

enum class FinState : uint8_t { unavailable, free, zero_hold, roll_control };

struct FinTelemetry {
  FinState state{FinState::unavailable};
  diagnostics::DeviceState encoder_state{
      diagnostics::DeviceState::unavailable};
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

  // 現在の連続AS5047D位置を論理Fin 0度としてRAM上にcaptureする。
  // motor modeは変更しない。
  [[nodiscard]] esp_err_t setZero();

  // 既にcapture済みの論理0度を保持する。Zeroを再captureしない。
  [[nodiscard]] esp_err_t zeroHold();

  [[nodiscard]] esp_err_t setRollControlTorque(double torque_nm);

  // motorをHi-Zへ落とすが、AS5047DのunwrapとZero referenceは保持する。
  [[nodiscard]] esp_err_t free();

  void update(uint64_t now_us);
  void forceSafe();

  [[nodiscard]] bool encoderRecoveryRequested() const;
  [[nodiscard]] esp_err_t recoverEncoder();
  [[nodiscard]] FinTelemetry telemetry() const;

private:
  [[nodiscard]] esp_err_t initializeEncoderTransport();
  [[nodiscard]] esp_err_t initializeMotor();
  [[nodiscard]] esp_err_t drive(double duty_signed);
  [[nodiscard]] esp_err_t coast();
  [[nodiscard]] esp_err_t applyOutputTorque(double torque_nm,
                                            double angle_rad,
                                            double rate_rad_s);
  [[nodiscard]] double zeroHoldTorque(double angle_rad, double rate_rad_s,
                                      double dt_s);
  void resetZeroHoldController();

  SPICREATE spi_{};
  AS5047D encoder_{};
  SemaphoreHandle_t encoder_mutex_{};
  bool motor_initialized_{};

  std::atomic<FinState> state_{FinState::unavailable};
  diagnostics::DeviceHealth encoder_health_{};
  std::atomic<bool> encoder_valid_{};
  std::atomic<bool> rate_valid_{};
  std::atomic<bool> zero_valid_{};
  std::atomic<double> angle_rad_{};
  std::atomic<double> rate_rad_s_{};
  std::atomic<uint64_t> sample_timestamp_us_{};

  // AS5047Dは1回転絶対角しか返さないため、multi-turn位置はRAM上でunwrapする。
  bool encoder_tracking_initialized_{};
  bool encoder_unwrapped_valid_{};
  double previous_encoder_raw_rad_{};
  double encoder_unwrapped_rad_{};
  double zero_encoder_unwrapped_rad_{};
  uint64_t previous_timestamp_us_{};

  control::ZeroHoldState zero_hold_state_{};
  bool zero_hold_output_limited_{};
  double requested_roll_torque_nm_{};
};

} // namespace actuators
