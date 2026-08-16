#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace control {

struct GainPoint {
  double airspeed_mps{};
  std::array<double, 4> gain{};
};

struct RollControlInput {
  double roll_deviation_rad{};
  double fin_angle_rad{};
  double roll_rate_rad_s{};
  double fin_rate_rad_s{};
  double airspeed_mps{};
};

struct RollControlOutput {
  double torque_nm{};
  bool saturated{};
  bool valid{};
};

class RollController {
public:
  RollController(const std::array<GainPoint, 7> &schedule,
                 double torque_limit_nm);

  [[nodiscard]] RollControlOutput
  compute(const RollControlInput &input) const;

private:
  std::array<GainPoint, 7> schedule_{};
  double torque_limit_nm_{};
};

class RollEstimator {
public:
  explicit RollEstimator(uint64_t maximum_gap_us);

  void reset();
  [[nodiscard]] bool start(uint64_t timestamp_us, double roll_rate_rad_s);
  [[nodiscard]] bool observe(uint64_t timestamp_us, bool sample_valid,
                             double roll_rate_rad_s);

  [[nodiscard]] bool started() const { return started_; }
  [[nodiscard]] bool valid() const { return valid_; }
  [[nodiscard]] double deviationRad() const { return deviation_rad_; }
  [[nodiscard]] uint64_t lastSampleUs() const { return last_sample_us_; }

private:
  uint64_t maximum_gap_us_{};
  uint64_t last_sample_us_{};
  double previous_rate_rad_s_{};
  double deviation_rad_{};
  bool started_{};
  bool valid_{};
};

class FlightControlSession {
public:
  FlightControlSession(uint64_t gyro_maximum_gap_us,
                       double permanent_stop_airspeed_mps);

  void reset();
  [[nodiscard]] bool evaluateEligibility(bool imu_roll_rate_available,
                                         bool fin_zero_valid);
  void observeAirspeed(bool valid, double airspeed_mps);
  [[nodiscard]] bool startReference(uint64_t timestamp_us,
                                    double roll_rate_rad_s);
  [[nodiscard]] bool observeGyro(uint64_t timestamp_us, bool sample_valid,
                                 double roll_rate_rad_s);
  void disablePermanently();

  [[nodiscard]] bool gateEvaluated() const { return gate_evaluated_; }
  [[nodiscard]] bool permanentlyDisabled() const {
    return permanently_disabled_;
  }
  [[nodiscard]] bool referenceStarted() const { return estimator_.started(); }
  [[nodiscard]] bool estimatorValid() const { return estimator_.valid(); }
  [[nodiscard]] double rollDeviationRad() const {
    return estimator_.deviationRad();
  }

private:
  RollEstimator estimator_;
  double permanent_stop_airspeed_mps_{};
  bool gate_evaluated_{};
  bool permanently_disabled_{};
};

} // namespace control
