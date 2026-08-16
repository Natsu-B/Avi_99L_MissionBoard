#include "control/zero_hold.hpp"

#include <algorithm>
#include <cmath>

namespace control {
namespace {

bool validBaseConfig(const ZeroHoldConfig &config) {
  return std::isfinite(config.kp_nm_per_rad) && config.kp_nm_per_rad >= 0.0 &&
         std::isfinite(config.kd_nm_s_per_rad) &&
         config.kd_nm_s_per_rad >= 0.0 &&
         std::isfinite(config.angle_dead_zone_rad) &&
         config.angle_dead_zone_rad >= 0.0 &&
         std::isfinite(config.rate_dead_zone_rad_s) &&
         config.rate_dead_zone_rad_s >= 0.0;
}

bool validPidConfig(const ZeroHoldConfig &config) {
  return validBaseConfig(config) && std::isfinite(config.ki_nm_per_rad_s) &&
         config.ki_nm_per_rad_s >= 0.0 &&
         std::isfinite(config.integral_limit_rad_s) &&
         config.integral_limit_rad_s >= 0.0 &&
         std::isfinite(config.velocity_filter_tau_s) &&
         config.velocity_filter_tau_s >= 0.0 &&
         std::isfinite(config.hold_deadband_rad) &&
         config.hold_deadband_rad >= 0.0 &&
         std::isfinite(config.hold_rate_deadband_rad_s) &&
         config.hold_rate_deadband_rad_s >= 0.0;
}

double continuousDeadZone(double value, double width) {
  const double residual = std::max(std::abs(value) - width, 0.0);
  if (residual == 0.0)
    return 0.0;
  return value < 0.0 ? -residual : residual;
}

} // namespace

void resetZeroHold(ZeroHoldState &state) { state = {}; }

ZeroHoldOutput computeZeroHold(const ZeroHoldInput &input,
                               const ZeroHoldConfig &config) {
  if (!validBaseConfig(config) || !std::isfinite(config.torque_limit_nm) ||
      config.torque_limit_nm <= 0.0 || !std::isfinite(input.angle_rad) ||
      !std::isfinite(input.rate_rad_s))
    return {};

  const double angle =
      continuousDeadZone(input.angle_rad, config.angle_dead_zone_rad);
  const double rate =
      continuousDeadZone(input.rate_rad_s, config.rate_dead_zone_rad_s);
  const double raw =
      -config.kp_nm_per_rad * angle - config.kd_nm_s_per_rad * rate;
  if (!std::isfinite(raw))
    return {};

  const double limited =
      std::clamp(raw, -config.torque_limit_nm, config.torque_limit_nm);
  return {raw, limited, rate, false, false, limited != raw, true};
}

ZeroHoldOutput computeZeroHold(const ZeroHoldInput &input,
                               const ZeroHoldConfig &config,
                               ZeroHoldState &state, bool allow_integrator) {
  if (!validPidConfig(config) || !std::isfinite(input.angle_rad) ||
      !std::isfinite(input.rate_rad_s) || !std::isfinite(input.dt_s) ||
      input.dt_s <= 0.0)
    return {};

  if (!state.rate_filter_initialized) {
    state.filtered_rate_rad_s = 0.0;
    state.rate_filter_initialized = true;
  }

  const double alpha = config.velocity_filter_tau_s <= 0.0
                           ? 1.0
                           : input.dt_s /
                                 (config.velocity_filter_tau_s + input.dt_s);
  state.filtered_rate_rad_s +=
      alpha * (input.rate_rad_s - state.filtered_rate_rad_s);

  const double error_rad = -input.angle_rad;
  const bool in_deadband =
      std::abs(error_rad) <= config.hold_deadband_rad &&
      std::abs(state.filtered_rate_rad_s) <= config.hold_rate_deadband_rad_s;

  if (in_deadband) {
    // 実機ZeroHoldと同様、目標近傍では出力を切り積分残留を減衰させる。
    state.integral_error_rad_s *= 0.95;
    if (std::abs(state.integral_error_rad_s) < 1.0e-6)
      state.integral_error_rad_s = 0.0;
    return {0.0, 0.0, state.filtered_rate_rad_s, true, !allow_integrator, false,
            true};
  }

  if (allow_integrator) {
    state.integral_error_rad_s = std::clamp(
        state.integral_error_rad_s + error_rad * input.dt_s,
        -config.integral_limit_rad_s, config.integral_limit_rad_s);
  }

  const double raw = config.kp_nm_per_rad * error_rad +
                     config.ki_nm_per_rad_s * state.integral_error_rad_s -
                     config.kd_nm_s_per_rad * state.filtered_rate_rad_s;
  if (!std::isfinite(raw))
    return {};

  // Controller側では人工的な600/800 command clampを持たせない。
  // hardware current / bus voltage / duty / fin-angle safetyをactuator側で制限する。
  return {raw, raw, state.filtered_rate_rad_s, false, !allow_integrator, false,
          true};
}

} // namespace control
