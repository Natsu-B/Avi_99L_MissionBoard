#pragma once

#include <atomic>
#include <cstdint>

#include "AS5047D.h"
#include "SPICREATE.h"
#include "actuators/fin_drive_interlock.hpp"
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
  bool zero_reference_valid{};
  bool zero_hold_achieved{};
  double angle_deg{};
  double rate_deg_s{};
  // RollControlはTorqueMapper値、ZeroHoldはcommand-domainのNm-equivalent値。
  // いずれも実測torque/currentではない。
  double requested_torque_nm{};
  double effective_torque_nm{};
  double estimated_motor_current_a{};
  double motor_speed_rpm{};
  double duty_signed{};
  uint16_t command_magnitude{};
  bool actuator_limited{};
  bool minimum_command_applied{};
  bool minimum_command_limited_by_current{};
  bool minimum_command_rejected_torque_direction{};
  bool current_limited{};
  bool current_limit_unrealizable{};
  bool torque_direction_unrealizable{};
  bool duty_limited{};
  bool outward_inhibited{};
  bool motor_speed_inhibited{};
  bool gearbox_speed_exceeded{};
  bool coast_required{};
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
  // Safety task専用。先にatomic inhibitをlatchし、進行中のwriterが
  // mutexを解放するまで待ってmotorを確実にcoastする。
  void latchPowerCutoff();
  [[nodiscard]] esp_err_t clearPowerCutoffForNewEpoch();

  [[nodiscard]] bool encoderRecoveryRequested() const;
  [[nodiscard]] esp_err_t recoverEncoder();
  [[nodiscard]] FinTelemetry telemetry() const;

private:
  [[nodiscard]] esp_err_t initializeEncoderTransport();
  [[nodiscard]] esp_err_t initializeMotor();
  [[nodiscard]] esp_err_t setMotorPwmFrequency(uint32_t frequency_hz);
  [[nodiscard]] esp_err_t driveCommand(int16_t command);
  [[nodiscard]] esp_err_t driveBrakeCommand(int16_t command);
  [[nodiscard]] esp_err_t coast();
  [[nodiscard]] esp_err_t applyRollControlTorque(double torque_nm,
                                                 double angle_rad,
                                                 double rate_rad_s,
                                                 bool motion_requested);
  [[nodiscard]] esp_err_t applyZeroHold(uint64_t sample_timestamp_us,
                                        double angle_rad, double rate_rad_s,
                                        double dt_s);
  void resetZeroHoldController();
  void clearZeroHoldAchievement();
  void consumeControllerResetRequest();
  void updateZeroHoldAchievement(uint64_t sample_timestamp_us,
                                 double angle_rad, double rate_rad_s);
  void resetOutputTelemetry();
  void forceSafeLocked();

  SPICREATE spi_{};
  AS5047D encoder_{};
  SemaphoreHandle_t encoder_mutex_{};
  std::atomic<bool> motor_initialized_{};
  uint32_t motor_pwm_frequency_hz_{};
  FinDriveInterlock drive_interlock_{};

  std::atomic<FinState> state_{FinState::unavailable};
  diagnostics::DeviceHealth encoder_health_{};
  std::atomic<bool> encoder_valid_{};
  std::atomic<bool> rate_valid_{};
  std::atomic<bool> zero_reference_valid_{};
  std::atomic<bool> zero_hold_achieved_{};
  // Recovery taskはnon-atomic controller stateへ触れず、realtime ownerへ
  // reset要求だけを渡す。
  std::atomic<bool> controller_reset_requested_{};
  std::atomic<double> angle_rad_{};
  std::atomic<double> rate_rad_s_{};
  std::atomic<uint64_t> sample_timestamp_us_{};
  std::atomic<double> requested_torque_nm_{};
  std::atomic<double> effective_torque_nm_{};
  std::atomic<double> estimated_motor_current_a_{};
  std::atomic<double> motor_speed_rpm_{};
  std::atomic<double> duty_signed_{};
  std::atomic<uint16_t> command_magnitude_{};
  std::atomic<bool> actuator_limited_{};
  std::atomic<bool> minimum_command_applied_{};
  std::atomic<bool> minimum_command_limited_by_current_{};
  std::atomic<bool> minimum_command_rejected_torque_direction_{};
  std::atomic<bool> current_limited_{};
  std::atomic<bool> current_limit_unrealizable_{};
  std::atomic<bool> torque_direction_unrealizable_{};
  std::atomic<bool> duty_limited_{};
  std::atomic<bool> outward_inhibited_{};
  std::atomic<bool> motor_speed_inhibited_{};
  std::atomic<bool> gearbox_speed_exceeded_{};
  std::atomic<bool> coast_required_{};

  // AS5047Dは1回転絶対角しか返さないため、multi-turn位置はRAM上でunwrapする。
  bool encoder_tracking_initialized_{};
  bool encoder_unwrapped_valid_{};
  double previous_encoder_raw_rad_{};
  double encoder_unwrapped_rad_{};
  double zero_encoder_unwrapped_rad_{};
  uint64_t previous_timestamp_us_{};
  double last_sample_dt_s_{};

  double requested_roll_torque_nm_{};
  control::ZeroHoldState zero_hold_controller_state_{};
  control::ZeroHoldAchievementState zero_hold_achievement_state_{};
  uint64_t last_zero_hold_sample_us_{};
};

} // namespace actuators
