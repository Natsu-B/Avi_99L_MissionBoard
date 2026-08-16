#include "actuators/fin.hpp"

#include <algorithm>
#include <cmath>

#include "actuators/safe_outputs.hpp"
#include "config/board.hpp"
#include "config/flight.hpp"
#include "driver/ledc.h"

namespace actuators {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr ledc_mode_t kLedcMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kLedcTimer = LEDC_TIMER_0;
constexpr ledc_channel_t kIn1Channel = LEDC_CHANNEL_0;
constexpr ledc_channel_t kIn2Channel = LEDC_CHANNEL_1;
constexpr ledc_timer_bit_t kDutyResolution = LEDC_TIMER_10_BIT;
constexpr uint32_t kMaximumDutyCount = (1U << 10U) - 1U;

} // namespace

esp_err_t FinActuator::initializeMotor() {
  ledc_timer_config_t timer{};
  timer.speed_mode = kLedcMode;
  timer.duty_resolution = kDutyResolution;
  timer.timer_num = kLedcTimer;
  timer.freq_hz = board::kMotorPwmFrequencyHz;
  timer.clk_cfg = LEDC_USE_APB_CLK;
  esp_err_t result = ledc_timer_config(&timer);
  if (result != ESP_OK)
    return result;

  ledc_channel_config_t channel{};
  channel.speed_mode = kLedcMode;
  channel.timer_sel = kLedcTimer;
  channel.duty = 0;
  channel.hpoint = 0;
  channel.gpio_num = board::kMotorIn1;
  channel.channel = kIn1Channel;
  result = ledc_channel_config(&channel);
  if (result != ESP_OK)
    return result;

  channel.gpio_num = board::kMotorIn2;
  channel.channel = kIn2Channel;
  result = ledc_channel_config(&channel);
  if (result == ESP_OK)
    motor_initialized_ = true;
  return result;
}

esp_err_t FinActuator::initialize() {
  SPICREATE::Config spi_config{};
  spi_config.host = board::kEncoderSpiHost;
  spi_config.sck = board::kEncoderSclk;
  spi_config.miso = board::kEncoderMiso;
  spi_config.mosi = board::kEncoderMosi;
  spi_config.transaction_timeout = avi::Timeout::milliseconds(2);
  esp_err_t result = spi_.begin(spi_config);
  if (result == ESP_OK) {
    AS5047D::Config encoder_config{};
    encoder_config.frequency_hz = board::kEncoderSpiFrequencyHz;
    result = encoder_.begin(spi_, board::kEncoderCs, encoder_config);
  }
  if (result == ESP_OK)
    result = initializeMotor();
  if (result != ESP_OK) {
    forceSafe();
    return result;
  }
  state_.store(FinState::free, std::memory_order_release);
  return coast();
}

esp_err_t FinActuator::holdCurrent() {
  if (!encoder_.initialized() || !motor_initialized_)
    return ESP_ERR_INVALID_STATE;
  AS5047D::Data data{};
  const esp_err_t result = encoder_.read(data);
  if (result != ESP_OK || !std::isfinite(data.angle_radians)) {
    encoder_valid_.store(false, std::memory_order_release);
    zero_valid_.store(false, std::memory_order_release);
    (void)coast();
    return result == ESP_OK ? ESP_ERR_INVALID_RESPONSE : result;
  }
  zero_rad_ = data.angle_radians;
  previous_angle_rad_ = 0.0;
  previous_timestamp_us_ = 0;
  angle_rad_.store(0.0, std::memory_order_release);
  rate_rad_s_.store(0.0, std::memory_order_release);
  encoder_valid_.store(true, std::memory_order_release);
  zero_valid_.store(true, std::memory_order_release);
  state_.store(FinState::hold, std::memory_order_release);
  return ESP_OK;
}

esp_err_t FinActuator::free() {
  zero_valid_.store(false, std::memory_order_release);
  state_.store(FinState::free, std::memory_order_release);
  return coast();
}

double FinActuator::wrapRadians(double value) {
  return std::remainder(value, kTwoPi);
}

void FinActuator::update(uint64_t now_us) {
  if (!encoder_.initialized())
    return;

  AS5047D::Data data{};
  const esp_err_t read = encoder_.read(data);
  if (read != ESP_OK || !std::isfinite(data.angle_radians)) {
    encoder_valid_.store(false, std::memory_order_release);
    if (state_.load(std::memory_order_acquire) == FinState::hold)
      (void)coast();
    return;
  }

  const bool zero_valid = zero_valid_.load(std::memory_order_acquire);
  const double relative = zero_valid ? wrapRadians(data.angle_radians - zero_rad_) : 0.0;
  double rate = 0.0;
  bool rate_valid = false;
  if (previous_timestamp_us_ != 0 && now_us > previous_timestamp_us_) {
    const double dt = static_cast<double>(now_us - previous_timestamp_us_) * 1.0e-6;
    const double delta = wrapRadians(relative - previous_angle_rad_);
    if (dt > 0.0 && dt <= 0.01) {
      rate = delta / dt;
      rate_valid = std::isfinite(rate);
    }
  }
  previous_angle_rad_ = relative;
  previous_timestamp_us_ = now_us;
  encoder_valid_.store(true, std::memory_order_release);
  angle_rad_.store(relative, std::memory_order_release);
  rate_rad_s_.store(rate_valid ? rate : 0.0, std::memory_order_release);

  if (state_.load(std::memory_order_acquire) != FinState::hold || !zero_valid)
    return;

  const double effective_rate = rate_valid ? rate : 0.0;
  double torque = -flight_config::kFinZeroHoldKpNmPerRad * relative -
                  flight_config::kFinZeroHoldKdNmPerRadS * effective_rate;
  torque = std::clamp(torque, -flight_config::kFinZeroHoldTorqueLimitNm,
                      flight_config::kFinZeroHoldTorqueLimitNm);

  const double motor_torque =
      torque / (flight_config::kTotalGearRatio *
                flight_config::kDrivetrainEfficiency);
  double current = motor_torque / flight_config::kMotorTorqueConstantNmPerA;
  current = std::clamp(current, -flight_config::kMotorMaxCurrentA,
                       flight_config::kMotorMaxCurrentA);

  const double motor_speed_rad_s = effective_rate * flight_config::kTotalGearRatio;
  const double motor_speed_rpm = motor_speed_rad_s * 60.0 / kTwoPi;
  const double back_emf = motor_speed_rpm / flight_config::kMotorSpeedConstantRpmPerV;
  const double voltage = current * flight_config::kMotorResistanceOhm + back_emf;
  double duty = std::clamp(std::abs(voltage) / flight_config::kMotorBusVoltageV,
                           0.0, flight_config::kMotorMaximumDuty);
  if (std::abs(torque) < 1.0e-4 && std::abs(effective_rate) < 1.0e-3)
    duty = 0.0;
  const bool positive_voltage = voltage >= 0.0;
  const bool positive_in1 = flight_config::kPositiveTorqueUsesIn1
                                ? positive_voltage
                                : !positive_voltage;
  (void)drive(positive_in1 ? duty : -duty);
}

esp_err_t FinActuator::drive(double duty_signed) {
  if (!motor_initialized_ || !std::isfinite(duty_signed))
    return ESP_ERR_INVALID_STATE;
  const double magnitude = std::clamp(std::abs(duty_signed), 0.0, 1.0);
  const uint32_t count = static_cast<uint32_t>(std::lround(magnitude * kMaximumDutyCount));
  const ledc_channel_t active = duty_signed >= 0.0 ? kIn1Channel : kIn2Channel;
  const ledc_channel_t inactive = duty_signed >= 0.0 ? kIn2Channel : kIn1Channel;
  esp_err_t result = ledc_set_duty(kLedcMode, inactive, 0);
  if (result == ESP_OK)
    result = ledc_update_duty(kLedcMode, inactive);
  if (result == ESP_OK)
    result = ledc_set_duty(kLedcMode, active, count);
  if (result == ESP_OK)
    result = ledc_update_duty(kLedcMode, active);
  return result;
}

esp_err_t FinActuator::coast() {
  if (!motor_initialized_)
    return safe_outputs::motorCoast();
  esp_err_t result = ledc_stop(kLedcMode, kIn1Channel, 0);
  const esp_err_t second = ledc_stop(kLedcMode, kIn2Channel, 0);
  if (result == ESP_OK)
    result = second;
  const esp_err_t gpio_result = safe_outputs::motorCoast();
  if (result == ESP_OK)
    result = gpio_result;
  return result;
}

void FinActuator::forceSafe() {
  state_.store(FinState::free, std::memory_order_release);
  zero_valid_.store(false, std::memory_order_release);
  (void)coast();
}

FinTelemetry FinActuator::telemetry() const {
  FinTelemetry result{};
  result.state = state_.load(std::memory_order_acquire);
  result.encoder_valid = encoder_valid_.load(std::memory_order_acquire);
  result.zero_valid = zero_valid_.load(std::memory_order_acquire);
  result.angle_deg = angle_rad_.load(std::memory_order_acquire) * kRadToDeg;
  result.rate_deg_s = rate_rad_s_.load(std::memory_order_acquire) * kRadToDeg;
  return result;
}

} // namespace actuators
