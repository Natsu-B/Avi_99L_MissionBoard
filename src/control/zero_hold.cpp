#include "control/zero_hold.hpp"

#include <algorithm>
#include <cmath>

namespace control {
namespace {

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

bool validConfig(const ZeroHoldConfig &config) {
  return std::isfinite(config.kp_command_per_deg) &&
         config.kp_command_per_deg >= 0.0 &&
         std::isfinite(config.ki_command_per_deg_s) &&
         config.ki_command_per_deg_s >= 0.0 &&
         std::isfinite(config.kd_command_per_deg_per_s) &&
         config.kd_command_per_deg_per_s >= 0.0 &&
         std::isfinite(config.integral_limit_deg_s) &&
         config.integral_limit_deg_s > 0.0 &&
         std::isfinite(config.rate_filter_tau_s) &&
         config.rate_filter_tau_s > 0.0 &&
         std::isfinite(config.hold_angle_deadband_deg) &&
         config.hold_angle_deadband_deg >= 0.0 &&
         std::isfinite(config.hold_rate_deadband_deg_s) &&
         config.hold_rate_deadband_deg_s >= 0.0 &&
         std::isfinite(config.minimum_active_error_deg) &&
         config.minimum_active_error_deg >= config.hold_angle_deadband_deg &&
         std::isfinite(config.maximum_dt_s) && config.maximum_dt_s > 0.0 &&
         std::isfinite(config.integral_decay) && config.integral_decay >= 0.0 &&
         config.integral_decay <= 1.0 &&
         std::isfinite(config.integral_zero_threshold_deg_s) &&
         config.integral_zero_threshold_deg_s >= 0.0 &&
         config.command_limit > 0 && config.minimum_command > 0 &&
         config.minimum_command <= config.command_limit &&
         std::isfinite(config.outward_angle_limit_rad) &&
         config.outward_angle_limit_rad > 0.0;
}

} // namespace

void resetZeroHold(ZeroHoldState &state) { state = {}; }

ZeroHoldOutput computeZeroHold(const ZeroHoldInput &input,
                               const ZeroHoldConfig &config,
                               ZeroHoldState &state) {
  if (!validConfig(config) || !std::isfinite(input.angle_rad) ||
      !std::isfinite(input.rate_rad_s) || !std::isfinite(input.dt_s) ||
      input.dt_s <= 0.0 || input.dt_s > config.maximum_dt_s)
    return {};

  if (!state.rate_filter_initialized) {
    // 実機characterization同様、entry時の速度LPFは0から開始する。
    state.filtered_rate_deg_s = 0.0;
    state.rate_filter_initialized = true;
  } else {
    const double alpha = input.dt_s / (config.rate_filter_tau_s + input.dt_s);
    state.filtered_rate_deg_s +=
        alpha * (input.rate_rad_s * kRadToDeg - state.filtered_rate_deg_s);
  }

  const double error = -input.angle_rad * kRadToDeg;
  const bool in_deadband =
      std::abs(error) <= config.hold_angle_deadband_deg &&
      std::abs(state.filtered_rate_deg_s) <= config.hold_rate_deadband_deg_s;
  if (in_deadband) {
    state.integral_error_deg_s *= config.integral_decay;
    if (std::abs(state.integral_error_deg_s) <
        config.integral_zero_threshold_deg_s)
      state.integral_error_deg_s = 0.0;
    return {0.0, error, state.filtered_rate_deg_s,
            state.integral_error_deg_s, 0, true, false, false, false, false,
            true};
  }

  const double candidate = state.integral_error_deg_s + error * input.dt_s;
  state.integral_error_deg_s =
      std::clamp(candidate, -config.integral_limit_deg_s,
                 config.integral_limit_deg_s);
  const bool integral_frozen = state.integral_error_deg_s != candidate;

  const double raw_command =
      config.kp_command_per_deg * error +
      config.ki_command_per_deg_s * state.integral_error_deg_s -
      config.kd_command_per_deg_per_s * state.filtered_rate_deg_s;
  if (!std::isfinite(raw_command))
    return {};

  const double limited_command =
      std::clamp(raw_command, -static_cast<double>(config.command_limit),
                 static_cast<double>(config.command_limit));
  int16_t command = static_cast<int16_t>(std::lround(limited_command));
  const bool command_limited = limited_command != raw_command;
  bool minimum_command_applied = false;
  if (std::abs(error) >= config.minimum_active_error_deg && command != 0 &&
      std::abs(static_cast<int>(command)) < config.minimum_command) {
    command = command > 0 ? config.minimum_command : -config.minimum_command;
    minimum_command_applied = true;
  }

  const bool outward_inhibited =
      (command > 0 && input.angle_rad >= config.outward_angle_limit_rad) ||
      (command < 0 && input.angle_rad <= -config.outward_angle_limit_rad);
  if (outward_inhibited)
    command = 0;

  return {raw_command,
          error,
          state.filtered_rate_deg_s,
          state.integral_error_deg_s,
          command,
          false,
          command_limited,
          minimum_command_applied,
          outward_inhibited,
          integral_frozen,
          true};
}

void resetZeroHoldAchievement(ZeroHoldAchievementState &state) { state = {}; }

bool updateZeroHoldAchievement(uint64_t sample_timestamp_us,
                               bool sample_valid, double angle_rad,
                               double rate_rad_s,
                               const ZeroHoldAchievementConfig &config,
                               ZeroHoldAchievementState &state) {
  const bool valid = sample_valid && sample_timestamp_us != 0 &&
      std::isfinite(angle_rad) &&
      std::isfinite(rate_rad_s) && std::isfinite(config.maximum_angle_rad) &&
      config.maximum_angle_rad >= 0.0 &&
      std::isfinite(config.maximum_rate_rad_s) &&
      config.maximum_rate_rad_s >= 0.0 && config.required_duration_us > 0 &&
      config.maximum_sample_gap_us > 0;
  if (!valid || std::abs(angle_rad) > config.maximum_angle_rad ||
      std::abs(rate_rad_s) > config.maximum_rate_rad_s) {
    resetZeroHoldAchievement(state);
    return false;
  }

  if (state.last_sample_us != 0 &&
      sample_timestamp_us <= state.last_sample_us) {
    resetZeroHoldAchievement(state);
    return false;
  }
  if (state.last_sample_us != 0 &&
      sample_timestamp_us - state.last_sample_us >
          config.maximum_sample_gap_us)
    resetZeroHoldAchievement(state);

  if (state.candidate_since_us == 0)
    state.candidate_since_us = sample_timestamp_us;
  state.last_sample_us = sample_timestamp_us;
  state.achieved =
      sample_timestamp_us - state.candidate_since_us >=
      config.required_duration_us;
  return state.achieved;
}

} // namespace control
