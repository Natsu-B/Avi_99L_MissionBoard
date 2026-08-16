#include "mission/flight_detectors.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace mission {
namespace {

template <std::size_t N>
double mean(const std::array<double, N> &samples) {
  return std::accumulate(samples.begin(), samples.end(), 0.0) /
         static_cast<double>(N);
}

} // namespace

void ImuLiftoffDetector::reset() { *this = {}; }

bool ImuLiftoffDetector::update(double ax_g, double ay_g, double az_g,
                                bool valid) {
  if (!valid || !std::isfinite(ax_g) || !std::isfinite(ay_g) ||
      !std::isfinite(az_g)) {
    consecutive_ = 0;
    return false;
  }
  ax_[head_] = ax_g;
  ay_[head_] = ay_g;
  az_[head_] = az_g;
  head_ = (head_ + 1U) % ax_.size();
  count_ = std::min(count_ + 1U, ax_.size());
  if (count_ != ax_.size())
    return false;
  const double x = mean(ax_);
  const double y = mean(ay_);
  const double z = mean(az_);
  if (x * x + y * y + z * z > 4.0)
    consecutive_ = std::min<uint8_t>(50U, static_cast<uint8_t>(consecutive_ + 1U));
  else
    consecutive_ = 0;
  return consecutive_ >= 50U;
}

void LpsLiftoffDetector::reset() { *this = {}; }

bool LpsLiftoffDetector::update(double pressure_hpa, bool valid) {
  if (!valid || !std::isfinite(pressure_hpa)) {
    reset();
    return false;
  }
  samples_[head_] = pressure_hpa;
  head_ = (head_ + 1U) % samples_.size();
  count_ = std::min(count_ + 1U, samples_.size());
  if (count_ != samples_.size())
    return false;
  const double current = mean(samples_);
  if (have_previous_mean_) {
    if (previous_mean_ - current >= 0.05)
      consecutive_ = std::min<uint8_t>(5U, static_cast<uint8_t>(consecutive_ + 1U));
    else
      consecutive_ = 0;
  }
  previous_mean_ = current;
  have_previous_mean_ = true;
  return consecutive_ >= 5U;
}

void PressureApexDetector::reset() { *this = {}; }

bool PressureApexDetector::update(double pressure_hpa, bool valid,
                                  uint64_t elapsed_us) {
  if (!valid || !std::isfinite(pressure_hpa)) {
    reset();
    return false;
  }
  samples_[head_] = pressure_hpa;
  head_ = (head_ + 1U) % samples_.size();
  count_ = std::min(count_ + 1U, samples_.size());
  if (count_ != samples_.size())
    return false;
  const double current = mean(samples_);
  if (have_previous_mean_ && elapsed_us >= 10'000'000ULL) {
    if (current > previous_mean_)
      consecutive_ = std::min<uint8_t>(25U, static_cast<uint8_t>(consecutive_ + 1U));
    else
      consecutive_ = 0;
  } else {
    consecutive_ = 0;
  }
  previous_mean_ = current;
  have_previous_mean_ = true;
  return consecutive_ >= 25U;
}

} // namespace mission
