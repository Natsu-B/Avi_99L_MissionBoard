#pragma once

#include <cstdint>

namespace control {

struct ZeroHoldConfig {
  double kp_nm_per_rad{};
  double ki_nm_per_rad_times_s{};
  double kd_nm_s_per_rad{};
  double integral_limit_rad_times_s{};
  double rate_filter_tau_s{};
  double hold_angle_deadband_rad{};
  double hold_rate_deadband_rad_s{};
  double minimum_active_error_rad{};
  double maximum_dt_s{};
  double integral_decay{};
  double integral_zero_threshold_rad_times_s{};
};

struct ZeroHoldInput {
  double angle_rad{};
  double rate_rad_s{};
  double dt_s{};
  bool integration_allowed{true};
};

struct ZeroHoldState {
  double integral_error_rad_times_s{};
  double filtered_rate_rad_s{};
  bool rate_filter_initialized{};
};

struct ZeroHoldOutput {
  double requested_torque_nm{};
  double error_rad{};
  double filtered_rate_rad_s{};
  double integral_error_rad_times_s{};
  bool in_hold_deadband{};
  bool motion_requested{};
  bool integral_frozen{};
  bool valid{};
};

struct ZeroHoldAchievementConfig {
  double maximum_angle_rad{};
  double maximum_rate_rad_s{};
  uint64_t required_duration_us{};
  uint64_t maximum_sample_gap_us{};
};

struct ZeroHoldAchievementState {
  uint64_t candidate_since_us{};
  uint64_t last_sample_us{};
  bool achieved{};
};

void resetZeroHold(ZeroHoldState &state);

// FIN0003/FIN0004取得時と同じPID/速度LPF順序でrequested torqueを計算する。
// saturation/current/limitの判定は共通actuator mapperが行い、その結果を次tickの
// integration_allowedへ戻す。
[[nodiscard]] ZeroHoldOutput computeZeroHold(const ZeroHoldInput &input,
                                             const ZeroHoldConfig &config,
                                             ZeroHoldState &state);

// actuator limitを同tickで受け、積分増分だけを戻す。速度LPF stateは保持する。
void applyZeroHoldActuatorFeedback(bool actuator_limited,
                                   double integral_before_step_rad_times_s,
                                   const ZeroHoldOutput &output,
                                   ZeroHoldState &state);

void resetZeroHoldAchievement(ZeroHoldAchievementState &state);
[[nodiscard]] bool updateZeroHoldAchievement(
    uint64_t sample_timestamp_us, bool sample_valid, double angle_rad,
    double rate_rad_s, const ZeroHoldAchievementConfig &config,
    ZeroHoldAchievementState &state);

} // namespace control
