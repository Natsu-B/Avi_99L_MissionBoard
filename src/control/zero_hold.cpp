#include "control/zero_hold.hpp"

#include <algorithm>
#include <cmath>

namespace control {
namespace {

bool validConfig(const ZeroHoldConfig &config) {
  return std::isfinite(config.kp_nm_per_rad) && config.kp_nm_per_rad >= 0.0 &&
         std::isfinite(config.ki_nm_per_rad_times_s) &&
         config.ki_nm_per_rad_times_s >= 0.0 &&
         std::isfinite(config.kd_nm_s_per_rad) &&
         config.kd_nm_s_per_rad >= 0.0 &&
         std::isfinite(config.integral_limit_rad_times_s) &&
         config.integral_limit_rad_times_s > 0.0 &&
         std::isfinite(config.rate_filter_tau_s) &&
         config.rate_filter_tau_s > 0.0 &&
         std::isfinite(config.hold_angle_deadband_rad) &&
         config.hold_angle_deadband_rad >= 0.0 &&
         std::isfinite(config.hold_rate_deadband_rad_s) &&
         config.hold_rate_deadband_rad_s >= 0.0 &&
         std::isfinite(config.minimum_active_error_rad) &&
         config.minimum_active_error_rad >= config.hold_angle_deadband_rad &&
         std::isfinite(config.maximum_dt_s) && config.maximum_dt_s > 0.0 &&
         std::isfinite(config.integral_decay) && config.integral_decay >= 0.0 &&
         config.integral_decay <= 1.0 &&
         std::isfinite(config.integral_zero_threshold_rad_times_s) &&
         config.integral_zero_threshold_rad_times_s >= 0.0;
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
    state.filtered_rate_rad_s = 0.0;
    state.rate_filter_initialized = true;
  } else {
    const double alpha = input.dt_s / (config.rate_filter_tau_s + input.dt_s);
    state.filtered_rate_rad_s +=
        alpha * (input.rate_rad_s - state.filtered_rate_rad_s);
  }

  const double error = -input.angle_rad;
  const bool in_deadband =
      std::abs(error) <= config.hold_angle_deadband_rad &&
      std::abs(state.filtered_rate_rad_s) <= config.hold_rate_deadband_rad_s;
  bool integral_frozen = !input.integration_allowed;
  if (in_deadband) {
    state.integral_error_rad_times_s *= config.integral_decay;
    if (std::abs(state.integral_error_rad_times_s) <
        config.integral_zero_threshold_rad_times_s)
      state.integral_error_rad_times_s = 0.0;
    return {0.0, error, state.filtered_rate_rad_s,
            state.integral_error_rad_times_s, true, false, integral_frozen,
            true};
  }

  if (input.integration_allowed) {
    const double candidate =
        state.integral_error_rad_times_s + error * input.dt_s;
    const double limited =
        std::clamp(candidate, -config.integral_limit_rad_times_s,
                   config.integral_limit_rad_times_s);
    integral_frozen = limited != candidate;
    state.integral_error_rad_times_s = limited;
  }

  const double requested =
      config.kp_nm_per_rad * error +
      config.ki_nm_per_rad_times_s * state.integral_error_rad_times_s -
      config.kd_nm_s_per_rad * state.filtered_rate_rad_s;
  if (!std::isfinite(requested))
    return {};

  const bool motion_requested =
      std::abs(error) >= config.minimum_active_error_rad && requested != 0.0;
  return {requested, error, state.filtered_rate_rad_s,
          state.integral_error_rad_times_s, false, motion_requested,
          integral_frozen, true};
}

void applyZeroHoldActuatorFeedback(
    bool actuator_limited, double integral_before_step_rad_times_s,
    const ZeroHoldOutput &output, ZeroHoldState &state) {
  if (!std::isfinite(integral_before_step_rad_times_s) || !output.valid) {
    resetZeroHold(state);
    return;
  }
  if (actuator_limited && !output.in_hold_deadband)
    state.integral_error_rad_times_s = integral_before_step_rad_times_s;
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
