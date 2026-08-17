#pragma once

#include <cstdint>

namespace control {

struct ZeroHoldConfig {
  double kp_command_per_deg{};
  double ki_command_per_deg_s{};
  double kd_command_per_deg_per_s{};
  double integral_limit_deg_s{};
  double rate_filter_tau_s{};
  double hold_angle_deadband_deg{};
  double hold_rate_deadband_deg_s{};
  double minimum_active_error_deg{};
  double maximum_dt_s{};
  double integral_decay{};
  double integral_zero_threshold_deg_s{};
  int16_t command_limit{};
  int16_t minimum_command{};
  double outward_angle_limit_rad{};
};

struct ZeroHoldInput {
  double angle_rad{};
  double rate_rad_s{};
  double dt_s{};
};

struct ZeroHoldState {
  double integral_error_deg_s{};
  double filtered_rate_deg_s{};
  bool rate_filter_initialized{};
};

struct ZeroHoldOutput {
  double raw_command{};
  double error_deg{};
  double filtered_rate_deg_s{};
  double integral_error_deg_s{};
  int16_t command{};
  bool in_hold_deadband{};
  bool command_limited{};
  bool minimum_command_applied{};
  bool outward_inhibited{};
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

// feat/zero-hold-nm-pidと同じPID/速度LPF/command制限順序で計算する。
[[nodiscard]] ZeroHoldOutput computeZeroHold(const ZeroHoldInput &input,
                                             const ZeroHoldConfig &config,
                                             ZeroHoldState &state);

void resetZeroHoldAchievement(ZeroHoldAchievementState &state);
[[nodiscard]] bool updateZeroHoldAchievement(
    uint64_t sample_timestamp_us, bool sample_valid, double angle_rad,
    double rate_rad_s, const ZeroHoldAchievementConfig &config,
    ZeroHoldAchievementState &state);

} // namespace control
