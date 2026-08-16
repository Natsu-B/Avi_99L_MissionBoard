#pragma once

namespace control {

struct ZeroHoldConfig {
  double kp_nm_per_rad{};
  double kd_nm_s_per_rad{};
  double torque_limit_nm{};
  double angle_dead_zone_rad{};
  double rate_dead_zone_rad_s{};
};

struct ZeroHoldInput {
  double angle_rad{};
  double rate_rad_s{};
};

struct ZeroHoldOutput {
  double raw_torque_nm{};
  double torque_nm{};
  bool saturated{};
  bool valid{};
};

// Spicaのmission_zero_hold_stepと同じ順序でZeroHold要求torqueを計算する。
[[nodiscard]] ZeroHoldOutput computeZeroHold(const ZeroHoldInput &input,
                                             const ZeroHoldConfig &config);

} // namespace control
