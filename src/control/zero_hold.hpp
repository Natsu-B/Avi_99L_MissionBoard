#pragma once

namespace control {

struct ZeroHoldConfig {
  double kp_nm_per_rad{};
  double kd_nm_s_per_rad{};
  double torque_limit_nm{};
  double angle_dead_zone_rad{};
  double rate_dead_zone_rad_s{};

  double ki_nm_per_rad_s{};
  double integral_limit_rad_s{};
  double velocity_filter_tau_s{};
  double hold_deadband_rad{};
  double hold_rate_deadband_rad_s{};

  constexpr ZeroHoldConfig() = default;

  // 既存Spica/host-test互換のPD設定。
  constexpr ZeroHoldConfig(double kp, double kd, double torque_limit,
                           double angle_dead_zone, double rate_dead_zone)
      : kp_nm_per_rad(kp), kd_nm_s_per_rad(kd),
        torque_limit_nm(torque_limit), angle_dead_zone_rad(angle_dead_zone),
        rate_dead_zone_rad_s(rate_dead_zone) {}

  // hardware-exercised PID用。人工的なtorque limitは持たせない。
  constexpr ZeroHoldConfig(double kp, double ki, double kd,
                           double integral_limit, double velocity_filter_tau,
                           double hold_deadband, double hold_rate_deadband)
      : kp_nm_per_rad(kp), kd_nm_s_per_rad(kd), ki_nm_per_rad_s(ki),
        integral_limit_rad_s(integral_limit),
        velocity_filter_tau_s(velocity_filter_tau),
        hold_deadband_rad(hold_deadband),
        hold_rate_deadband_rad_s(hold_rate_deadband) {}
};

struct ZeroHoldInput {
  double angle_rad{};
  double rate_rad_s{};
  double dt_s{};
};

struct ZeroHoldState {
  double integral_error_rad_s{};
  double filtered_rate_rad_s{};
  bool rate_filter_initialized{};
};

struct ZeroHoldOutput {
  double raw_torque_nm{};
  double torque_nm{};
  double filtered_rate_rad_s{};
  bool in_deadband{};
  bool integrator_held{};
  bool saturated{};
  bool valid{};
};

// 既存Spica/host-test互換のstateless PD。新しいflight ZeroHoldでは使用しない。
[[nodiscard]] ZeroHoldOutput computeZeroHold(const ZeroHoldInput &input,
                                             const ZeroHoldConfig &config);

// 実機characterizationで成立したPIDをNm-equivalent座標で計算する。
// allow_integrator=falseは、前tickでactuatorが飽和/制限された場合の
// conditional-integration anti-windupに使用する。
[[nodiscard]] ZeroHoldOutput computeZeroHold(const ZeroHoldInput &input,
                                             const ZeroHoldConfig &config,
                                             ZeroHoldState &state,
                                             bool allow_integrator);

void resetZeroHold(ZeroHoldState &state);

} // namespace control
