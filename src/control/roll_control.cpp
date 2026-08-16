#include "control/roll_control.hpp"

#include <algorithm>
#include <cmath>

namespace control {
namespace {

bool finiteInput(const RollControlInput &input) {
  return std::isfinite(input.roll_deviation_rad) &&
         std::isfinite(input.fin_angle_rad) &&
         std::isfinite(input.roll_rate_rad_s) &&
         std::isfinite(input.fin_rate_rad_s) &&
         std::isfinite(input.airspeed_mps);
}

} // namespace

RollController::RollController(const std::array<GainPoint, 7> &schedule,
                               double torque_limit_nm)
    : schedule_(schedule), torque_limit_nm_(torque_limit_nm) {}

RollControlOutput RollController::compute(const RollControlInput &input) const {
  if (!finiteInput(input) || !std::isfinite(torque_limit_nm_) ||
      torque_limit_nm_ <= 0.0)
    return {};

  for (std::size_t index = 0; index < schedule_.size(); ++index) {
    if (!std::isfinite(schedule_[index].airspeed_mps))
      return {};
    for (const double gain : schedule_[index].gain) {
      if (!std::isfinite(gain))
        return {};
    }
    if (index != 0 &&
        schedule_[index - 1].airspeed_mps >= schedule_[index].airspeed_mps)
      return {};
  }

  const double limited_speed =
      std::clamp(input.airspeed_mps, schedule_.front().airspeed_mps,
                 schedule_.back().airspeed_mps);
  std::size_t high = 1;
  while (high < schedule_.size() &&
         schedule_[high].airspeed_mps < limited_speed)
    ++high;
  high = std::min(high, schedule_.size() - 1);
  const std::size_t low = high == 0 ? 0 : high - 1;

  const auto &left = schedule_[low];
  const auto &right = schedule_[high];
  const double denominator = right.airspeed_mps - left.airspeed_mps;
  const double fraction = denominator > 0.0
                              ? (limited_speed - left.airspeed_mps) / denominator
                              : 0.0;
  const std::array<double, 4> state{
      input.roll_deviation_rad, input.fin_angle_rad, input.roll_rate_rad_s,
      input.fin_rate_rad_s};

  double torque = 0.0;
  for (std::size_t index = 0; index < state.size(); ++index) {
    const double gain = left.gain[index] +
                        fraction * (right.gain[index] - left.gain[index]);
    torque -= gain * state[index];
  }
  if (!std::isfinite(torque))
    return {};

  const double limited = std::clamp(torque, -torque_limit_nm_, torque_limit_nm_);
  return {limited, limited != torque, true};
}

RollEstimator::RollEstimator(uint64_t maximum_gap_us)
    : maximum_gap_us_(maximum_gap_us) {}

void RollEstimator::reset() {
  last_sample_us_ = 0;
  previous_rate_rad_s_ = 0.0;
  deviation_rad_ = 0.0;
  started_ = false;
  valid_ = false;
}

bool RollEstimator::start(uint64_t timestamp_us, double roll_rate_rad_s) {
  reset();
  if (timestamp_us == 0 || !std::isfinite(roll_rate_rad_s) ||
      maximum_gap_us_ == 0)
    return false;
  last_sample_us_ = timestamp_us;
  previous_rate_rad_s_ = roll_rate_rad_s;
  started_ = true;
  valid_ = true;
  return true;
}

bool RollEstimator::observe(uint64_t timestamp_us, bool sample_valid,
                            double roll_rate_rad_s) {
  if (!started_ || !valid_)
    return false;
  if (timestamp_us <= last_sample_us_) {
    valid_ = false;
    return false;
  }

  const uint64_t gap_us = timestamp_us - last_sample_us_;
  if (!sample_valid || !std::isfinite(roll_rate_rad_s)) {
    if (gap_us > maximum_gap_us_)
      valid_ = false;
    return valid_;
  }
  if (gap_us > maximum_gap_us_) {
    valid_ = false;
    return false;
  }

  const double dt = static_cast<double>(gap_us) * 1.0e-6;
  deviation_rad_ += 0.5 * (previous_rate_rad_s_ + roll_rate_rad_s) * dt;
  if (!std::isfinite(deviation_rad_)) {
    valid_ = false;
    return false;
  }
  previous_rate_rad_s_ = roll_rate_rad_s;
  last_sample_us_ = timestamp_us;
  return true;
}

FlightControlSession::FlightControlSession(uint64_t gyro_maximum_gap_us,
                                           double permanent_stop_airspeed_mps)
    : estimator_(gyro_maximum_gap_us),
      permanent_stop_airspeed_mps_(permanent_stop_airspeed_mps) {}

void FlightControlSession::reset() {
  estimator_.reset();
  gate_evaluated_ = false;
  permanently_disabled_ = false;
}

bool FlightControlSession::evaluateEligibility(bool imu_roll_rate_available,
                                               bool fin_zero_valid) {
  if (gate_evaluated_)
    return !permanently_disabled_;
  gate_evaluated_ = true;
  if (!imu_roll_rate_available || !fin_zero_valid)
    permanently_disabled_ = true;
  return !permanently_disabled_;
}

void FlightControlSession::observeAirspeed(bool valid, double airspeed_mps) {
  if (!gate_evaluated_ || permanently_disabled_ || !valid ||
      !std::isfinite(airspeed_mps))
    return;
  if (airspeed_mps <= permanent_stop_airspeed_mps_)
    permanently_disabled_ = true;
}

bool FlightControlSession::startReference(uint64_t timestamp_us,
                                          double roll_rate_rad_s) {
  if (!gate_evaluated_ || permanently_disabled_ || estimator_.started())
    return false;
  if (!estimator_.start(timestamp_us, roll_rate_rad_s)) {
    permanently_disabled_ = true;
    return false;
  }
  return true;
}

bool FlightControlSession::observeGyro(uint64_t timestamp_us,
                                       bool sample_valid,
                                       double roll_rate_rad_s) {
  if (!estimator_.started())
    return true;
  if (!estimator_.observe(timestamp_us, sample_valid, roll_rate_rad_s)) {
    if (!estimator_.valid())
      permanently_disabled_ = true;
    return false;
  }
  return true;
}

void FlightControlSession::disablePermanently() {
  permanently_disabled_ = true;
}

} // namespace control
