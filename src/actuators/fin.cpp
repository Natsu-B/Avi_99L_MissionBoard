#include "actuators/fin.hpp"

#include <algorithm>
#include <cmath>

#include "actuators/fin_kinematics.hpp"
#include "actuators/fin_torque_mapper.hpp"
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

class SemaphoreGuard {
public:
  explicit SemaphoreGuard(SemaphoreHandle_t semaphore)
      : semaphore_(semaphore) {}
  ~SemaphoreGuard() {
    if (semaphore_ != nullptr)
      xSemaphoreGive(semaphore_);
  }

  SemaphoreGuard(const SemaphoreGuard &) = delete;
  SemaphoreGuard &operator=(const SemaphoreGuard &) = delete;

private:
  SemaphoreHandle_t semaphore_{};
};

static_assert(board::kFinZeroHoldPwmFrequencyHz == 20'000U);
static_assert(board::kFinRollControlPwmFrequencyHz == 30'000U);
static_assert(flight_config::kMotorCommandFullScale == 1'024U);
static_assert(kMaximumDutyCount == 1'023U);

control::ZeroHoldConfig zeroHoldConfig() {
  return {flight_config::kFinZeroHoldKpCommandPerDeg,
          flight_config::kFinZeroHoldKiCommandPerDegS,
          flight_config::kFinZeroHoldKdCommandPerDegPerS,
          flight_config::kFinZeroHoldIntegralLimitDegS,
          flight_config::kFinZeroHoldRateFilterTauS,
          flight_config::kFinZeroHoldAngleDeadbandDeg,
          flight_config::kFinZeroHoldRateDeadbandDegS,
          flight_config::kFinZeroHoldMinimumActiveErrorDeg,
          flight_config::kFinZeroHoldMaximumDtS,
          flight_config::kFinZeroHoldIntegralDecay,
          flight_config::kFinZeroHoldIntegralZeroThresholdDegS,
          flight_config::kFinZeroHoldControlCommandLimit,
          flight_config::kFinZeroHoldMinimumCommand,
          flight_config::kFinOutwardCommandLimitDeg * kDegToRad};
}

FinTorqueMapperConfig torqueMapperConfig() {
  return {flight_config::kMotorResistanceOhm,
          flight_config::kMotorTorqueConstantNmPerA,
          flight_config::kMotorSpeedConstantRpmPerV,
          flight_config::kTotalGearRatio,
          flight_config::kDrivetrainEfficiency,
          flight_config::kMotorBusVoltageV,
          flight_config::kMotorMaxCurrentA,
          flight_config::kMotorHardSpeedRpm,
          flight_config::kGearboxContinuousSpeedRpm,
          flight_config::kMotorMaximumDuty,
          flight_config::kFinOutwardCommandLimitDeg * kDegToRad,
          flight_config::kMotorCommandFullScale,
          flight_config::kMotorMinimumActiveCommand,
          flight_config::kPositiveTorqueUsesIn1};
}

} // namespace

esp_err_t FinActuator::initializeMotor() {
  ledc_timer_config_t timer{};
  timer.speed_mode = kLedcMode;
  timer.duty_resolution = kDutyResolution;
  timer.timer_num = kLedcTimer;
  timer.freq_hz = board::kFinZeroHoldPwmFrequencyHz;
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
  if (result == ESP_OK) {
    motor_initialized_ = true;
    motor_pwm_frequency_hz_ = board::kFinZeroHoldPwmFrequencyHz;
  }
  return result;
}

esp_err_t FinActuator::setMotorPwmFrequency(uint32_t frequency_hz) {
  if (!motor_initialized_.load(std::memory_order_acquire) || frequency_hz == 0)
    return ESP_ERR_INVALID_STATE;
  if (motor_pwm_frequency_hz_ == frequency_hz)
    return ESP_OK;
  const esp_err_t result = ledc_set_freq(kLedcMode, kLedcTimer, frequency_hz);
  if (result == ESP_OK)
    motor_pwm_frequency_hz_ = frequency_hz;
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
  drive_interlock_.resetForInitialization();
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
  zero_reference_valid_.store(false, std::memory_order_release);
  zero_hold_achieved_.store(false, std::memory_order_release);
  controller_reset_requested_.store(false, std::memory_order_release);
  encoder_valid_.store(false, std::memory_order_release);
  rate_valid_.store(false, std::memory_order_release);
  resetZeroHoldController();
  encoder_health_.markHealthy();
  state_.store(FinState::free, std::memory_order_release);
  return coast();
}

esp_err_t FinActuator::setZero() {
  if (encoder_mutex_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  if (xSemaphoreTake(encoder_mutex_, 0) != pdTRUE)
    return ESP_ERR_TIMEOUT;
  const SemaphoreGuard encoder_guard{encoder_mutex_};
  consumeControllerResetRequest();

  // Zero captureはAS5047Dの現在位置だけで成立し、motor初期化状態には
  // 依存させない。controller stateとの同期だけはmutex内で行う。
  if (drive_interlock_.inhibited() || !encoder_tracking_initialized_ ||
      !encoder_valid_.load(std::memory_order_acquire))
    return ESP_ERR_INVALID_STATE;

  zero_encoder_unwrapped_rad_ = encoder_unwrapped_rad_;
  angle_rad_.store(0.0, std::memory_order_release);
  zero_reference_valid_.store(true, std::memory_order_release);
  clearZeroHoldAchievement();
  resetZeroHoldController();
  return ESP_OK;
}

esp_err_t FinActuator::zeroHold() {
  if (encoder_mutex_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  if (xSemaphoreTake(encoder_mutex_, 0) != pdTRUE)
    return ESP_ERR_TIMEOUT;
  const SemaphoreGuard encoder_guard{encoder_mutex_};
  consumeControllerResetRequest();

  if (drive_interlock_.inhibited() || !motor_initialized_ ||
      !zero_reference_valid_.load(std::memory_order_acquire))
    return ESP_ERR_INVALID_STATE;

  const FinState previous = state_.load(std::memory_order_acquire);
  requested_roll_torque_nm_ = 0.0;
  if (previous != FinState::zero_hold) {
    resetZeroHoldController();
    clearZeroHoldAchievement();
    // Roll torqueを先に失効させ、周波数変更失敗時もControl出力を再適用しない。
    state_.store(FinState::free, std::memory_order_release);
    // characterization版と同様、ZeroHold開始時は両入力LOWにして20 kHzへ戻す。
    const esp_err_t arming_result = driveCommand(0);
    if (arming_result != ESP_OK)
      return arming_result;
    const esp_err_t frequency_result =
        setMotorPwmFrequency(board::kFinZeroHoldPwmFrequencyHz);
    if (frequency_result != ESP_OK)
      return frequency_result;
  }
  state_.store(FinState::zero_hold, std::memory_order_release);
  if (!encoder_valid_.load(std::memory_order_acquire) ||
      !rate_valid_.load(std::memory_order_acquire))
    return coast();

  const uint64_t sample_timestamp =
      sample_timestamp_us_.load(std::memory_order_acquire);
  if (sample_timestamp == last_zero_hold_sample_us_)
    return ESP_OK;
  const double angle = angle_rad_.load(std::memory_order_acquire);
  const double rate = rate_rad_s_.load(std::memory_order_acquire);
  return applyZeroHold(sample_timestamp, angle, rate, last_sample_dt_s_);
}

esp_err_t FinActuator::setRollControlTorque(double torque_nm) {
  if (encoder_mutex_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  if (xSemaphoreTake(encoder_mutex_, 0) != pdTRUE)
    return ESP_ERR_TIMEOUT;
  const SemaphoreGuard encoder_guard{encoder_mutex_};
  consumeControllerResetRequest();

  if (drive_interlock_.inhibited() || !motor_initialized_ ||
      !zero_reference_valid_.load(std::memory_order_acquire) ||
      !zero_hold_achieved_.load(std::memory_order_acquire) ||
      !encoder_valid_.load(std::memory_order_acquire) ||
      !rate_valid_.load(std::memory_order_acquire) || !std::isfinite(torque_nm))
    return ESP_ERR_INVALID_STATE;

  const FinState previous = state_.load(std::memory_order_acquire);
  if (previous != FinState::roll_control) {
    const esp_err_t arming_result = driveCommand(0);
    if (arming_result != ESP_OK)
      return arming_result;
    const esp_err_t frequency_result =
        setMotorPwmFrequency(board::kFinRollControlPwmFrequencyHz);
    if (frequency_result != ESP_OK)
      return frequency_result;
  }
  if (previous == FinState::zero_hold)
    resetZeroHoldController();
  state_.store(FinState::roll_control, std::memory_order_release);
  requested_roll_torque_nm_ = torque_nm;
  const double angle = angle_rad_.load(std::memory_order_acquire);
  const double rate = rate_rad_s_.load(std::memory_order_acquire);
  const esp_err_t result =
      applyRollControlTorque(torque_nm, angle, rate, torque_nm != 0.0);
  if (result != ESP_OK) {
    requested_roll_torque_nm_ = 0.0;
    state_.store(FinState::free, std::memory_order_release);
    clearZeroHoldAchievement();
    resetZeroHoldController();
    (void)coast();
  }
  return result;
}

esp_err_t FinActuator::free() {
  if (encoder_mutex_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  if (xSemaphoreTake(encoder_mutex_, 0) != pdTRUE)
    return ESP_ERR_TIMEOUT;
  const SemaphoreGuard encoder_guard{encoder_mutex_};
  consumeControllerResetRequest();

  requested_roll_torque_nm_ = 0.0;
  state_.store(FinState::free, std::memory_order_release);
  clearZeroHoldAchievement();
  resetZeroHoldController();
  return coast();
}

void FinActuator::resetZeroHoldController() {
  control::resetZeroHold(zero_hold_controller_state_);
  last_zero_hold_sample_us_ = 0;
}

void FinActuator::clearZeroHoldAchievement() {
  control::resetZeroHoldAchievement(zero_hold_achievement_state_);
  zero_hold_achieved_.store(false, std::memory_order_release);
}

void FinActuator::consumeControllerResetRequest() {
  if (!controller_reset_requested_.exchange(false,
                                             std::memory_order_acq_rel))
    return;
  resetZeroHoldController();
  clearZeroHoldAchievement();
}

void FinActuator::updateZeroHoldAchievement(uint64_t sample_timestamp_us,
                                            double angle_rad,
                                            double rate_rad_s) {
  const bool achieved = control::updateZeroHoldAchievement(
      sample_timestamp_us, true, angle_rad, rate_rad_s,
      {flight_config::kFinZeroHoldAchievedAngleDeg * kDegToRad,
       flight_config::kFinZeroHoldAchievedRateDegS * kDegToRad,
       flight_config::kFinZeroHoldAchievedDurationUs,
       flight_config::kFinFreshUs},
      zero_hold_achievement_state_);
  zero_hold_achieved_.store(achieved, std::memory_order_release);
}

esp_err_t FinActuator::applyZeroHold(uint64_t sample_timestamp_us,
                                    double angle_rad, double rate_rad_s,
                                    double dt_s) {
  const auto output = control::computeZeroHold(
      {angle_rad, rate_rad_s, dt_s}, zeroHoldConfig(),
      zero_hold_controller_state_);
  if (!output.valid) {
    resetZeroHoldController();
    clearZeroHoldAchievement();
    return coast();
  }

  const int16_t hardware_command = flight_config::kPositiveTorqueUsesIn1
                                       ? output.command
                                       : -output.command;
  resetOutputTelemetry();
  requested_torque_nm_.store(
      output.raw_command * flight_config::kFinZeroHoldNmPerCommand,
      std::memory_order_release);
  effective_torque_nm_.store(
      static_cast<double>(output.command) *
          flight_config::kFinZeroHoldNmPerCommand,
      std::memory_order_release);
  motor_speed_rpm_.store(rate_rad_s * flight_config::kTotalGearRatio * 60.0 /
                             kTwoPi,
                         std::memory_order_release);
  duty_signed_.store(
      static_cast<double>(hardware_command) /
          static_cast<double>(flight_config::kMotorCommandFullScale),
      std::memory_order_release);
  command_magnitude_.store(
      static_cast<uint16_t>(std::abs(static_cast<int>(hardware_command))),
      std::memory_order_release);
  actuator_limited_.store(output.command_limited || output.outward_inhibited,
                          std::memory_order_release);
  minimum_command_applied_.store(output.minimum_command_applied,
                                 std::memory_order_release);
  outward_inhibited_.store(output.outward_inhibited,
                           std::memory_order_release);
  coast_required_.store(output.command == 0, std::memory_order_release);

  const esp_err_t result = driveCommand(hardware_command);
  if (result != ESP_OK) {
    resetZeroHoldController();
    clearZeroHoldAchievement();
    (void)coast();
    return result;
  }
  last_zero_hold_sample_us_ = sample_timestamp_us;
  updateZeroHoldAchievement(sample_timestamp_us, angle_rad, rate_rad_s);
  return ESP_OK;
}

void FinActuator::update(uint64_t now_us) {
  if (encoder_mutex_ == nullptr)
    return;

  // Recovery taskがbegin/end中なら待たず、そのtickを欠測として扱う。
  if (xSemaphoreTake(encoder_mutex_, 0) != pdTRUE) {
    encoder_valid_.store(false, std::memory_order_release);
    rate_valid_.store(false, std::memory_order_release);
    zero_hold_achieved_.store(false, std::memory_order_release);
    controller_reset_requested_.store(true, std::memory_order_release);
    // mutex ownerのRecovery taskがmotorをcoastする。ここから同じdriverを
    // 並行操作しない。
    return;
  }
  const SemaphoreGuard encoder_guard{encoder_mutex_};
  consumeControllerResetRequest();

  AS5047D::Data data{};
  esp_err_t read_result = ESP_ERR_INVALID_STATE;
  if (encoder_.initialized() && encoder_.pipelinedReadActive())
    read_result = encoder_.readPipelined(data);

  if (read_result != ESP_OK || !std::isfinite(data.angle_radians)) {
    encoder_valid_.store(false, std::memory_order_release);
    rate_valid_.store(false, std::memory_order_release);
    resetZeroHoldController();
    clearZeroHoldAchievement();
    encoder_health_.markFailure(kEncoderFailureThreshold);
    if (state_.load(std::memory_order_acquire) != FinState::free)
      (void)coast();
    return;
  }

  encoder_health_.markHealthy();

  double encoder_delta_rad = 0.0;
  bool rate_valid = false;
  double sample_dt_s = 0.0;
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
        sample_dt_s = dt;
        const double fin_delta_rad = fin_kinematics::encoderToFinRadians(
            encoder_delta_rad, flight_config::kTotalGearRatio);
        const double rate = fin_delta_rad / dt;
        if (std::isfinite(rate)) {
          rate_rad_s_.store(rate, std::memory_order_release);
          rate_valid = true;
          sample_dt_s = dt;
        }
      }
    }
  }

  previous_timestamp_us_ = now_us;
  encoder_valid_.store(true, std::memory_order_release);
  rate_valid_.store(rate_valid, std::memory_order_release);
  if (!rate_valid)
    rate_rad_s_.store(0.0, std::memory_order_release);
  else
    last_sample_dt_s_ = sample_dt_s;
  sample_timestamp_us_.store(now_us, std::memory_order_release);

  const bool zero_reference_valid =
      zero_reference_valid_.load(std::memory_order_acquire);
  const double angle =
      zero_reference_valid
          ? fin_kinematics::encoderToFinRadians(
                encoder_unwrapped_rad_ - zero_encoder_unwrapped_rad_,
                flight_config::kTotalGearRatio)
          : 0.0;
  angle_rad_.store(angle, std::memory_order_release);

  if (!zero_reference_valid)
    return;

  const FinState state = state_.load(std::memory_order_acquire);
  if (state == FinState::zero_hold) {
    if (!rate_valid) {
      resetZeroHoldController();
      clearZeroHoldAchievement();
      (void)coast();
      return;
    }
    const double rate = rate_rad_s_.load(std::memory_order_acquire);
    (void)applyZeroHold(now_us, angle, rate, sample_dt_s);
  } else if (state == FinState::roll_control) {
    if (!rate_valid) {
      clearZeroHoldAchievement();
      (void)coast();
      return;
    }
    const double rate = rate_rad_s_.load(std::memory_order_acquire);
    const esp_err_t result =
        applyRollControlTorque(requested_roll_torque_nm_, angle, rate,
                               requested_roll_torque_nm_ != 0.0);
    if (result != ESP_OK) {
      // LEDC/map fault後に旧PWMを残さない。Runtimeはfree stateを
      // Control actuator faultとしてlatchし、必要ならZeroHoldへ移る。
      requested_roll_torque_nm_ = 0.0;
      state_.store(FinState::free, std::memory_order_release);
      clearZeroHoldAchievement();
      resetZeroHoldController();
      if (coast() != ESP_OK)
        (void)safe_outputs::motorCoast();
    }
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
  drive_interlock_.noteRecoverableFault();
  // controller/achievement stateはrealtime taskだけがresetする。
  controller_reset_requested_.store(true, std::memory_order_release);
  zero_hold_achieved_.store(false, std::memory_order_release);
  encoder_valid_.store(false, std::memory_order_release);
  rate_valid_.store(false, std::memory_order_release);
  // Roll中のdropoutから復帰したtickで古いtorqueを再適用しない。
  requested_roll_torque_nm_ = 0.0;
  state_.store(zero_reference_valid_.load(std::memory_order_acquire)
                   ? FinState::zero_hold
                   : FinState::free,
               std::memory_order_release);
  (void)coast();

  esp_err_t result = initializeEncoderTransport();
  if (result == ESP_OK && !motor_initialized_)
    result = initializeMotor();

  if (result == ESP_OK) {
    // zeroと切断前multi-turn位置は保持し、次sampleを最寄りbranchへ接続する。
    encoder_tracking_initialized_ = false;
    previous_timestamp_us_ = 0;
    rate_rad_s_.store(0.0, std::memory_order_release);
    drive_interlock_.noteSuccessfulRecovery();
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

esp_err_t FinActuator::applyRollControlTorque(double torque_nm,
                                              double angle_rad,
                                              double rate_rad_s,
                                              bool motion_requested) {
  if (drive_interlock_.inhibited()) {
    (void)coast();
    return ESP_ERR_INVALID_STATE;
  }
  if (!motor_initialized_ || !std::isfinite(torque_nm) ||
      !std::isfinite(angle_rad) || !std::isfinite(rate_rad_s))
    return ESP_ERR_INVALID_STATE;

  const auto mapped = mapFinOutputTorque(
      {torque_nm, angle_rad, rate_rad_s, motion_requested},
      torqueMapperConfig());
  if (!mapped.valid)
    return ESP_ERR_INVALID_ARG;
  // Safety taskがlatchする直前のsnapshotから計算を開始していても、
  // hardware書き込みの前に再確認して非0 driveを禁止する。
  if (drive_interlock_.inhibited()) {
    (void)coast();
    return ESP_ERR_INVALID_STATE;
  }

  requested_torque_nm_.store(mapped.requested_torque_nm,
                             std::memory_order_release);
  effective_torque_nm_.store(mapped.effective_torque_nm,
                             std::memory_order_release);
  estimated_motor_current_a_.store(mapped.estimated_motor_current_a,
                                   std::memory_order_release);
  motor_speed_rpm_.store(mapped.motor_speed_rpm, std::memory_order_release);
  duty_signed_.store(mapped.duty_signed, std::memory_order_release);
  command_magnitude_.store(mapped.command_magnitude,
                           std::memory_order_release);
  actuator_limited_.store(mapped.antiWindupRequired(),
                          std::memory_order_release);
  minimum_command_applied_.store(mapped.minimum_command_applied,
                                 std::memory_order_release);
  minimum_command_limited_by_current_.store(
      mapped.minimum_command_limited_by_current, std::memory_order_release);
  minimum_command_rejected_torque_direction_.store(
      mapped.minimum_command_rejected_torque_direction,
      std::memory_order_release);
  current_limited_.store(mapped.current_limited, std::memory_order_release);
  current_limit_unrealizable_.store(mapped.current_limit_unrealizable,
                                    std::memory_order_release);
  torque_direction_unrealizable_.store(
      mapped.torque_direction_unrealizable, std::memory_order_release);
  duty_limited_.store(mapped.duty_limited, std::memory_order_release);
  outward_inhibited_.store(mapped.outward_inhibited,
                           std::memory_order_release);
  motor_speed_inhibited_.store(mapped.motor_speed_inhibited,
                               std::memory_order_release);
  gearbox_speed_exceeded_.store(mapped.gearbox_speed_exceeded,
                                std::memory_order_release);
  coast_required_.store(mapped.coast_required, std::memory_order_release);

  const int16_t magnitude = static_cast<int16_t>(mapped.command_magnitude);
  const int16_t command = mapped.duty_signed < 0.0 ? -magnitude : magnitude;
  // RollControlだけがmainのtorque mapperとdrive/brakeを使用する。
  if (mapped.coast_required)
    return driveCommand(0);
  return driveBrakeCommand(command);
}

esp_err_t FinActuator::driveCommand(int16_t command) {
  if (!motor_initialized_)
    return ESP_ERR_INVALID_STATE;
  if (!drive_interlock_.allows(command))
    return ESP_ERR_INVALID_STATE;

  const int32_t clamped = std::clamp<int32_t>(
      command, -static_cast<int32_t>(flight_config::kMotorCommandFullScale),
      static_cast<int32_t>(flight_config::kMotorCommandFullScale));

  if (clamped == 0) {
    esp_err_t result = ledc_set_duty(kLedcMode, kIn1Channel, 0);
    if (result == ESP_OK)
      result = ledc_update_duty(kLedcMode, kIn1Channel);
    if (result == ESP_OK)
      result = ledc_set_duty(kLedcMode, kIn2Channel, 0);
    if (result == ESP_OK)
      result = ledc_update_duty(kLedcMode, kIn2Channel);
    return result;
  }

  const uint32_t magnitude = static_cast<uint32_t>(clamped < 0 ? -clamped
                                                               : clamped);
  // characterization版MotorDriverと同じ整数演算。1024 commandでも1023 count。
  const uint32_t duty = finCommandToPwmCount(
      static_cast<uint16_t>(magnitude),
      flight_config::kMotorCommandFullScale, kMaximumDutyCount);

  // 呼出元がmotor polarityを反映済みのhardware符号を渡す。
  const ledc_channel_t active = clamped > 0 ? kIn1Channel : kIn2Channel;
  const ledc_channel_t inactive = clamped > 0 ? kIn2Channel : kIn1Channel;

  esp_err_t result = ledc_set_duty(kLedcMode, inactive, 0);
  if (result == ESP_OK)
    result = ledc_update_duty(kLedcMode, inactive);
  if (result == ESP_OK)
    result = ledc_set_duty(kLedcMode, active, duty);
  if (result == ESP_OK)
    result = ledc_update_duty(kLedcMode, active);
  return result;
}

esp_err_t FinActuator::driveBrakeCommand(int16_t command) {
  if (!motor_initialized_)
    return ESP_ERR_INVALID_STATE;
  // drive/brakeの0 commandは両入力Hのshort brakeでありcoastではない。
  // cutoff後はcommand値にかかわらずこの経路を拒否する。
  if (drive_interlock_.inhibited())
    return ESP_ERR_INVALID_STATE;

  const int32_t clamped = std::clamp<int32_t>(
      command, -static_cast<int32_t>(flight_config::kMotorCommandFullScale),
      static_cast<int32_t>(flight_config::kMotorCommandFullScale));
  const uint32_t magnitude = static_cast<uint32_t>(clamped < 0 ? -clamped
                                                               : clamped);
  const uint32_t drive_count = finCommandToPwmCount(
      static_cast<uint16_t>(magnitude),
      flight_config::kMotorCommandFullScale, kMaximumDutyCount);
  const uint32_t brake_count = kMaximumDutyCount - drive_count;

  // held-high側を先に更新し、short brakeを経由してから逆側の
  // brake dutyを下げる。command=0では両入力がHになる。
  const ledc_channel_t held_high = clamped >= 0 ? kIn1Channel : kIn2Channel;
  const ledc_channel_t brake_pwm = clamped >= 0 ? kIn2Channel : kIn1Channel;
  esp_err_t result =
      ledc_set_duty(kLedcMode, held_high, kMaximumDutyCount);
  if (result == ESP_OK)
    result = ledc_update_duty(kLedcMode, held_high);
  if (result == ESP_OK)
    result = ledc_set_duty(kLedcMode, brake_pwm, brake_count);
  if (result == ESP_OK)
    result = ledc_update_duty(kLedcMode, brake_pwm);
  return result;
}

void FinActuator::resetOutputTelemetry() {
  requested_torque_nm_.store(0.0, std::memory_order_release);
  effective_torque_nm_.store(0.0, std::memory_order_release);
  estimated_motor_current_a_.store(0.0, std::memory_order_release);
  motor_speed_rpm_.store(0.0, std::memory_order_release);
  duty_signed_.store(0.0, std::memory_order_release);
  command_magnitude_.store(0, std::memory_order_release);
  actuator_limited_.store(false, std::memory_order_release);
  minimum_command_applied_.store(false, std::memory_order_release);
  minimum_command_limited_by_current_.store(false,
                                            std::memory_order_release);
  minimum_command_rejected_torque_direction_.store(false,
                                                   std::memory_order_release);
  current_limited_.store(false, std::memory_order_release);
  current_limit_unrealizable_.store(false, std::memory_order_release);
  torque_direction_unrealizable_.store(false, std::memory_order_release);
  duty_limited_.store(false, std::memory_order_release);
  outward_inhibited_.store(false, std::memory_order_release);
  motor_speed_inhibited_.store(false, std::memory_order_release);
  gearbox_speed_exceeded_.store(false, std::memory_order_release);
  coast_required_.store(false, std::memory_order_release);
}

esp_err_t FinActuator::coast() {
  resetOutputTelemetry();
  if (!motor_initialized_)
    return safe_outputs::motorCoast();

  // ledc_stop()やGPIO mode変更は行わない。characterization版と同じく
  // 両PWM dutyを0にし、次の制御tickでそのまま再駆動できる状態を保つ。
  const esp_err_t result = driveCommand(0);
  if (result != ESP_OK) {
    // LEDCの部分書込み失敗後に旧PWMを残さず、GPIO両側LOWを
    // 最終fallbackとする。再初期化まで非0駆動を禁止する。
    motor_initialized_.store(false, std::memory_order_release);
    (void)safe_outputs::motorCoast();
  }
  return result;
}

void FinActuator::forceSafeLocked() {
  requested_roll_torque_nm_ = 0.0;
  state_.store(FinState::free, std::memory_order_release);
  zero_reference_valid_.store(false, std::memory_order_release);
  clearZeroHoldAchievement();
  resetZeroHoldController();
  controller_reset_requested_.store(false, std::memory_order_release);
  encoder_valid_.store(false, std::memory_order_release);
  rate_valid_.store(false, std::memory_order_release);
  (void)coast();
  // forceSafeは再駆動を前提としないため、最後にGPIOも明示LOWへ固定する。
  (void)safe_outputs::motorCoast();
}

void FinActuator::forceSafe() {
  // boot/recoverable faultのfail-safeはpower-cutoff latchではない。復旧後に
  // CommandReceiveからFinZero/ZeroHoldをやり直せるようinterlockを保持する。
  drive_interlock_.noteRecoverableFault();
  state_.store(FinState::free, std::memory_order_release);
  zero_reference_valid_.store(false, std::memory_order_release);
  zero_hold_achieved_.store(false, std::memory_order_release);
  controller_reset_requested_.store(true, std::memory_order_release);

  if (encoder_mutex_ == nullptr) {
    forceSafeLocked();
    return;
  }
  // recoverable fault経路も待たない。他taskがdriver所有中なら
  // interlockで非0再driveを拒否し、recovery側のcoastに委ねる。
  if (xSemaphoreTake(encoder_mutex_, 0) != pdTRUE)
    return;
  const SemaphoreGuard encoder_guard{encoder_mutex_};
  forceSafeLocked();
}

void FinActuator::latchPowerCutoff() {
  drive_interlock_.latch();
  state_.store(FinState::free, std::memory_order_release);
  zero_reference_valid_.store(false, std::memory_order_release);
  zero_hold_achieved_.store(false, std::memory_order_release);
  controller_reset_requested_.store(true, std::memory_order_release);

  if (encoder_mutex_ == nullptr) {
    forceSafeLocked();
    return;
  }
  // Safety taskだけは進行中の短いrealtime/recovery writerの終了を待つ。
  // realtime 1 kHz taskはこのblocking APIを呼ばない。
  if (xSemaphoreTake(encoder_mutex_, portMAX_DELAY) != pdTRUE)
    return;
  const SemaphoreGuard encoder_guard{encoder_mutex_};
  // new epoch clearと競合して待っていた場合も、cutoffを最終優先する。
  drive_interlock_.latch();
  forceSafeLocked();
}

esp_err_t FinActuator::clearPowerCutoffForNewEpoch() {
  if (encoder_mutex_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  if (xSemaphoreTake(encoder_mutex_, 0) != pdTRUE)
    return ESP_ERR_TIMEOUT;
  const SemaphoreGuard encoder_guard{encoder_mutex_};
  drive_interlock_.clearForNewEpoch();
  return ESP_OK;
}

FinTelemetry FinActuator::telemetry() const {
  FinTelemetry result{};
  result.state = state_.load(std::memory_order_acquire);
  result.encoder_state = encoder_health_.state();
  result.encoder_valid = encoder_valid_.load(std::memory_order_acquire);
  result.rate_valid = rate_valid_.load(std::memory_order_acquire);
  result.zero_reference_valid =
      zero_reference_valid_.load(std::memory_order_acquire);
  result.zero_hold_achieved =
      zero_hold_achieved_.load(std::memory_order_acquire);
  result.angle_deg = angle_rad_.load(std::memory_order_acquire) * kRadToDeg;
  result.rate_deg_s = rate_rad_s_.load(std::memory_order_acquire) * kRadToDeg;
  result.requested_torque_nm =
      requested_torque_nm_.load(std::memory_order_acquire);
  result.effective_torque_nm =
      effective_torque_nm_.load(std::memory_order_acquire);
  result.estimated_motor_current_a =
      estimated_motor_current_a_.load(std::memory_order_acquire);
  result.motor_speed_rpm =
      motor_speed_rpm_.load(std::memory_order_acquire);
  result.duty_signed = duty_signed_.load(std::memory_order_acquire);
  result.command_magnitude =
      command_magnitude_.load(std::memory_order_acquire);
  result.actuator_limited =
      actuator_limited_.load(std::memory_order_acquire);
  result.minimum_command_applied =
      minimum_command_applied_.load(std::memory_order_acquire);
  result.minimum_command_limited_by_current =
      minimum_command_limited_by_current_.load(std::memory_order_acquire);
  result.minimum_command_rejected_torque_direction =
      minimum_command_rejected_torque_direction_.load(
          std::memory_order_acquire);
  result.current_limited =
      current_limited_.load(std::memory_order_acquire);
  result.current_limit_unrealizable =
      current_limit_unrealizable_.load(std::memory_order_acquire);
  result.torque_direction_unrealizable =
      torque_direction_unrealizable_.load(std::memory_order_acquire);
  result.duty_limited = duty_limited_.load(std::memory_order_acquire);
  result.outward_inhibited =
      outward_inhibited_.load(std::memory_order_acquire);
  result.motor_speed_inhibited =
      motor_speed_inhibited_.load(std::memory_order_acquire);
  result.gearbox_speed_exceeded =
      gearbox_speed_exceeded_.load(std::memory_order_acquire);
  result.coast_required = coast_required_.load(std::memory_order_acquire);
  result.sample_timestamp_us =
      sample_timestamp_us_.load(std::memory_order_acquire);
  return result;
}

} // namespace actuators
