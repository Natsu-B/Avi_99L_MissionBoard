#include "actuators/fin.hpp"

#include <algorithm>
#include <cmath>

#include "actuators/fin_kinematics.hpp"
#include "actuators/safe_outputs.hpp"
#include "config/board.hpp"
#include "config/flight.hpp"
#include "driver/ledc.h"

namespace actuators {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDegToRad = kPi / 180.0;
constexpr ledc_mode_t kLedcMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kLedcTimer = LEDC_TIMER_0;
constexpr ledc_channel_t kIn1Channel = LEDC_CHANNEL_0;
constexpr ledc_channel_t kIn2Channel = LEDC_CHANNEL_1;
constexpr ledc_timer_bit_t kDutyResolution = LEDC_TIMER_10_BIT;
constexpr uint32_t kMaximumDutyCount = (1U << 10U) - 1U;
constexpr uint8_t kEncoderFailureThreshold = 3U;

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

esp_err_t FinActuator::initializeEncoderTransport() {
  if (encoder_.initialized()) {
    const esp_err_t result = encoder_.end();
    if (result != ESP_OK)
      return result;
  }
  if (spi_.initialized()) {
    const esp_err_t result = spi_.end();
    if (result != ESP_OK)
      return result;
  }

  SPICREATE::Config spi_config{};
  spi_config.host = board::kEncoderSpiHost;
  spi_config.sck = board::kEncoderSclk;
  spi_config.miso = board::kEncoderMiso;
  spi_config.mosi = board::kEncoderMosi;
  // SPI2はAS5047D専用。bus lock競合で1 kHz taskを待たせない。
  spi_config.transaction_timeout = avi::Timeout::noWait();

  esp_err_t result = spi_.begin(spi_config);
  if (result != ESP_OK)
    return result;

  AS5047D::Config encoder_config{};
  encoder_config.frequency_hz = board::kEncoderSpiFrequencyHz;
  result = encoder_.begin(spi_, board::kEncoderCs, encoder_config);
  if (result == ESP_OK) {
    AS5047D::Status status{};
    result = encoder_.getStatus(status);
    if (result == ESP_OK &&
        (status.magnetic_too_low || status.magnetic_too_high ||
         status.cordic_overflow || !status.offset_compensation_finished))
      result = ESP_ERR_INVALID_RESPONSE;
  }
  if (result == ESP_OK)
    result = encoder_.startPipelinedRead();

  if (result != ESP_OK) {
    if (encoder_.initialized())
      (void)encoder_.end();
    if (spi_.initialized())
      (void)spi_.end();
  }
  return result;
}

esp_err_t FinActuator::initialize() {
  if (encoder_mutex_ == nullptr)
    encoder_mutex_ = xSemaphoreCreateMutex();
  if (encoder_mutex_ == nullptr)
    return ESP_ERR_NO_MEM;

  esp_err_t result = initializeEncoderTransport();
  if (result == ESP_OK)
    result = initializeMotor();
  if (result != ESP_OK) {
    encoder_health_.markFailed();
    forceSafe();
    return result;
  }

  encoder_tracking_initialized_ = false;
  encoder_unwrapped_valid_ = false;
  zero_valid_.store(false, std::memory_order_release);
  encoder_valid_.store(false, std::memory_order_release);
  rate_valid_.store(false, std::memory_order_release);
  encoder_health_.markHealthy();
  state_.store(FinState::free, std::memory_order_release);
  return coast();
}

esp_err_t FinActuator::setZero() {
  if (!motor_initialized_ || !encoder_tracking_initialized_ ||
      !encoder_valid_.load(std::memory_order_acquire))
    return ESP_ERR_INVALID_STATE;

  zero_encoder_unwrapped_rad_ = encoder_unwrapped_rad_;
  angle_rad_.store(0.0, std::memory_order_release);
  zero_valid_.store(true, std::memory_order_release);
  return ESP_OK;
}

esp_err_t FinActuator::zeroHold() {
  if (!motor_initialized_ || !zero_valid_.load(std::memory_order_acquire))
    return ESP_ERR_INVALID_STATE;

  state_.store(FinState::zero_hold, std::memory_order_release);
  requested_roll_torque_nm_ = 0.0;
  if (!encoder_valid_.load(std::memory_order_acquire))
    return coast();

  const double angle = angle_rad_.load(std::memory_order_acquire);
  const double rate = rate_valid_.load(std::memory_order_acquire)
                          ? rate_rad_s_.load(std::memory_order_acquire)
                          : 0.0;
  return applyOutputTorque(zeroHoldTorque(angle, rate), angle, rate);
}

esp_err_t FinActuator::setRollControlTorque(double torque_nm) {
  if (!motor_initialized_ || !zero_valid_.load(std::memory_order_acquire) ||
      !encoder_valid_.load(std::memory_order_acquire) ||
      !rate_valid_.load(std::memory_order_acquire) || !std::isfinite(torque_nm))
    return ESP_ERR_INVALID_STATE;

  requested_roll_torque_nm_ = torque_nm;
  state_.store(FinState::roll_control, std::memory_order_release);
  const double angle = angle_rad_.load(std::memory_order_acquire);
  const double rate = rate_rad_s_.load(std::memory_order_acquire);
  const esp_err_t result = applyOutputTorque(torque_nm, angle, rate);
  if (result != ESP_OK)
    (void)coast();
  return result;
}

esp_err_t FinActuator::free() {
  requested_roll_torque_nm_ = 0.0;
  state_.store(FinState::free, std::memory_order_release);
  return coast();
}

double FinActuator::zeroHoldTorque(double angle_rad, double rate_rad_s) const {
  double torque = -flight_config::kFinZeroHoldKpNmPerRad * angle_rad -
                  flight_config::kFinZeroHoldKdNmPerRadS * rate_rad_s;
  return std::clamp(torque, -flight_config::kFinZeroHoldTorqueLimitNm,
                    flight_config::kFinZeroHoldTorqueLimitNm);
}

void FinActuator::update(uint64_t now_us) {
  if (encoder_mutex_ == nullptr)
    return;

  // Recovery taskがbegin/end中なら待たず、そのtickを欠測として扱う。
  if (xSemaphoreTake(encoder_mutex_, 0) != pdTRUE)
    return;

  AS5047D::Data data{};
  esp_err_t read_result = ESP_ERR_INVALID_STATE;
  if (encoder_.initialized() && encoder_.pipelinedReadActive())
    read_result = encoder_.readPipelined(data);
  xSemaphoreGive(encoder_mutex_);

  if (read_result != ESP_OK || !std::isfinite(data.angle_radians)) {
    encoder_valid_.store(false, std::memory_order_release);
    rate_valid_.store(false, std::memory_order_release);
    encoder_health_.markFailure(kEncoderFailureThreshold);
    if (state_.load(std::memory_order_acquire) != FinState::free)
      (void)coast();
    return;
  }

  encoder_health_.markHealthy();

  double encoder_delta_rad = 0.0;
  bool rate_valid = false;
  if (!encoder_tracking_initialized_) {
    if (encoder_unwrapped_valid_) {
      encoder_unwrapped_rad_ = fin_kinematics::nearestEquivalentAngle(
          data.angle_radians, encoder_unwrapped_rad_);
    } else {
      encoder_unwrapped_rad_ = data.angle_radians;
      encoder_unwrapped_valid_ = true;
    }
    encoder_tracking_initialized_ = true;
    previous_encoder_raw_rad_ = data.angle_radians;
    previous_timestamp_us_ = 0;
  } else {
    encoder_delta_rad = fin_kinematics::unwrapEncoderDelta(
        data.angle_radians, previous_encoder_raw_rad_);
    encoder_unwrapped_rad_ += encoder_delta_rad;
    previous_encoder_raw_rad_ = data.angle_radians;

    if (previous_timestamp_us_ != 0 && now_us > previous_timestamp_us_) {
      const double dt =
          static_cast<double>(now_us - previous_timestamp_us_) * 1.0e-6;
      if (dt > 0.0 && dt <= 0.01) {
        const double fin_delta_rad = fin_kinematics::encoderToFinRadians(
            encoder_delta_rad, flight_config::kTotalGearRatio);
        const double rate = fin_delta_rad / dt;
        if (std::isfinite(rate)) {
          rate_rad_s_.store(rate, std::memory_order_release);
          rate_valid = true;
        }
      }
    }
  }

  previous_timestamp_us_ = now_us;
  encoder_valid_.store(true, std::memory_order_release);
  rate_valid_.store(rate_valid, std::memory_order_release);
  if (!rate_valid)
    rate_rad_s_.store(0.0, std::memory_order_release);
  sample_timestamp_us_.store(now_us, std::memory_order_release);

  const bool zero_valid = zero_valid_.load(std::memory_order_acquire);
  const double angle =
      zero_valid
          ? fin_kinematics::encoderToFinRadians(
                encoder_unwrapped_rad_ - zero_encoder_unwrapped_rad_,
                flight_config::kTotalGearRatio)
          : 0.0;
  angle_rad_.store(angle, std::memory_order_release);

  if (!zero_valid)
    return;

  const FinState state = state_.load(std::memory_order_acquire);
  const double effective_rate =
      rate_valid ? rate_rad_s_.load(std::memory_order_acquire) : 0.0;
  if (state == FinState::zero_hold) {
    (void)applyOutputTorque(zeroHoldTorque(angle, effective_rate), angle,
                            effective_rate);
  } else if (state == FinState::roll_control) {
    if (!rate_valid) {
      (void)coast();
      return;
    }
    (void)applyOutputTorque(requested_roll_torque_nm_, angle, effective_rate);
  }
}

esp_err_t FinActuator::recoverEncoder() {
  if (!encoderRecoveryRequested())
    return ESP_OK;
  if (encoder_mutex_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  if (xSemaphoreTake(encoder_mutex_, pdMS_TO_TICKS(20)) != pdTRUE)
    return ESP_ERR_TIMEOUT;

  encoder_health_.markRecovering();
  encoder_valid_.store(false, std::memory_order_release);
  rate_valid_.store(false, std::memory_order_release);
  if (state_.load(std::memory_order_acquire) != FinState::free)
    (void)coast();

  esp_err_t result = initializeEncoderTransport();
  if (result == ESP_OK && !motor_initialized_)
    result = initializeMotor();

  if (result == ESP_OK) {
    // zeroと切断前multi-turn位置は保持し、次sampleを最寄りbranchへ接続する。
    encoder_tracking_initialized_ = false;
    previous_timestamp_us_ = 0;
    rate_rad_s_.store(0.0, std::memory_order_release);
    // 最初のvalid sampleが来るまではRecoveringのままとする。
  } else {
    encoder_health_.markFailed();
  }

  xSemaphoreGive(encoder_mutex_);
  return result;
}

bool FinActuator::encoderRecoveryRequested() const {
  return encoder_health_.recoveryRequested();
}

esp_err_t FinActuator::applyOutputTorque(double torque_nm, double angle_rad,
                                         double rate_rad_s) {
  if (!motor_initialized_ || !std::isfinite(torque_nm) ||
      !std::isfinite(angle_rad) || !std::isfinite(rate_rad_s))
    return ESP_ERR_INVALID_STATE;

  const double command_limit =
      flight_config::kFinOutwardCommandLimitDeg * kDegToRad;
  if ((angle_rad >= command_limit && torque_nm > 0.0) ||
      (angle_rad <= -command_limit && torque_nm < 0.0))
    torque_nm = 0.0;

  const double motor_torque =
      torque_nm / (flight_config::kTotalGearRatio *
                   flight_config::kDrivetrainEfficiency);
  double current = motor_torque / flight_config::kMotorTorqueConstantNmPerA;
  current = std::clamp(current, -flight_config::kMotorMaxCurrentA,
                       flight_config::kMotorMaxCurrentA);

  const double motor_speed_rad_s =
      rate_rad_s * flight_config::kTotalGearRatio;
  const double motor_speed_rpm = motor_speed_rad_s * 60.0 / kTwoPi;
  const double back_emf =
      motor_speed_rpm / flight_config::kMotorSpeedConstantRpmPerV;
  const double voltage =
      current * flight_config::kMotorResistanceOhm + back_emf;
  double duty =
      std::clamp(std::abs(voltage) / flight_config::kMotorBusVoltageV, 0.0,
                 flight_config::kMotorMaximumDuty);
  if (std::abs(torque_nm) < 1.0e-4 && std::abs(rate_rad_s) < 1.0e-3)
    duty = 0.0;

  const bool positive_voltage = voltage >= 0.0;
  const bool positive_in1 = flight_config::kPositiveTorqueUsesIn1
                                ? positive_voltage
                                : !positive_voltage;
  return drive(positive_in1 ? duty : -duty);
}

esp_err_t FinActuator::drive(double duty_signed) {
  if (!motor_initialized_ || !std::isfinite(duty_signed))
    return ESP_ERR_INVALID_STATE;

  const double magnitude = std::clamp(std::abs(duty_signed), 0.0, 1.0);
  const uint32_t count = static_cast<uint32_t>(
      std::lround(magnitude * static_cast<double>(kMaximumDutyCount)));
  const ledc_channel_t active =
      duty_signed >= 0.0 ? kIn1Channel : kIn2Channel;
  const ledc_channel_t inactive =
      duty_signed >= 0.0 ? kIn2Channel : kIn1Channel;

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
  requested_roll_torque_nm_ = 0.0;
  state_.store(FinState::free, std::memory_order_release);
  zero_valid_.store(false, std::memory_order_release);
  encoder_valid_.store(false, std::memory_order_release);
  rate_valid_.store(false, std::memory_order_release);
  (void)coast();
}

FinTelemetry FinActuator::telemetry() const {
  FinTelemetry result{};
  result.state = state_.load(std::memory_order_acquire);
  result.encoder_state = encoder_health_.state();
  result.encoder_valid = encoder_valid_.load(std::memory_order_acquire);
  result.rate_valid = rate_valid_.load(std::memory_order_acquire);
  result.zero_valid = zero_valid_.load(std::memory_order_acquire);
  result.angle_deg = angle_rad_.load(std::memory_order_acquire) * kRadToDeg;
  result.rate_deg_s = rate_rad_s_.load(std::memory_order_acquire) * kRadToDeg;
  result.sample_timestamp_us =
      sample_timestamp_us_.load(std::memory_order_acquire);
  return result;
}

} // namespace actuators
