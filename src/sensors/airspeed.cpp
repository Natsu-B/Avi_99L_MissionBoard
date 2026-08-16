#include "sensors/airspeed.hpp"

#include <algorithm>
#include <cmath>

namespace sensors {
namespace {
constexpr double kSpecificGasConstantAir = 287.05;
constexpr double kHeatCapacityRatioAir = 1.4;
constexpr double kKelvinOffset = 273.15;
} // namespace

AirspeedResult
computeSaintVenantAirspeed(double static_pressure_pa,
                          double differential_pressure_pa,
                          double temperature_celsius,
                          double pitot_pressure_correction_coefficient) {
  if (!std::isfinite(static_pressure_pa) || static_pressure_pa <= 0.0 ||
      !std::isfinite(differential_pressure_pa) || differential_pressure_pa < 0.0 ||
      !std::isfinite(temperature_celsius) ||
      !std::isfinite(pitot_pressure_correction_coefficient) ||
      pitot_pressure_correction_coefficient <= 0.0)
    return {};

  const double temperature_kelvin = temperature_celsius + kKelvinOffset;
  if (temperature_kelvin <= 0.0)
    return {};
  if (differential_pressure_pa == 0.0)
    return {0.0, true};

  const double corrected_pressure_ratio =
      pitot_pressure_correction_coefficient *
      pitot_pressure_correction_coefficient * differential_pressure_pa /
      static_pressure_pa;
  if (!std::isfinite(corrected_pressure_ratio) || corrected_pressure_ratio < 0.0)
    return {};

  const double exponent =
      (kHeatCapacityRatioAir - 1.0) / kHeatCapacityRatioAir;
  const double expansion =
      std::expm1(exponent * std::log1p(corrected_pressure_ratio));
  const double radicand =
      (2.0 * kHeatCapacityRatioAir / (kHeatCapacityRatioAir - 1.0)) *
      kSpecificGasConstantAir * temperature_kelvin * expansion;
  if (!std::isfinite(radicand) || radicand < 0.0)
    return {};
  const double airspeed = std::sqrt(radicand);
  return std::isfinite(airspeed) ? AirspeedResult{airspeed, true} : AirspeedResult{};
}

DifferentialPressureFilter::DifferentialPressureFilter(
    double zero_offset_pa, double negative_tolerance_pa,
    std::size_t moving_average_samples)
    : zero_offset_pa_(zero_offset_pa),
      negative_tolerance_pa_(negative_tolerance_pa),
      moving_average_samples_(
          std::min(moving_average_samples, kMaximumSamples)) {}

void DifferentialPressureFilter::reset() {
  samples_.fill(0.0);
  head_ = 0;
  count_ = 0;
  sum_ = 0.0;
}

bool DifferentialPressureFilter::update(double raw_pressure_pa,
                                        double &filtered_pressure_pa) {
  if (!std::isfinite(raw_pressure_pa) || !std::isfinite(zero_offset_pa_) ||
      !std::isfinite(negative_tolerance_pa_) || negative_tolerance_pa_ < 0.0 ||
      moving_average_samples_ == 0)
    return false;

  double corrected = raw_pressure_pa - zero_offset_pa_;
  if (corrected < -negative_tolerance_pa_)
    return false;
  corrected = std::max(0.0, corrected);

  if (count_ == moving_average_samples_) {
    sum_ -= samples_[head_];
  } else {
    ++count_;
  }
  samples_[head_] = corrected;
  sum_ += corrected;
  head_ = (head_ + 1U) % moving_average_samples_;
  filtered_pressure_pa = sum_ / static_cast<double>(count_);
  return std::isfinite(filtered_pressure_pa);
}

} // namespace sensors
