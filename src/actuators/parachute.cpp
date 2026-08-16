#include "actuators/parachute.hpp"

#include <cmath>

#include "actuators/safe_outputs.hpp"
#include "config/board.hpp"
#include "config/flight.hpp"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace actuators {

esp_err_t ParachuteActuator::initialize() {
  if (power_cutoff_.load(std::memory_order_acquire))
    return ESP_ERR_INVALID_STATE;
  const esp_err_t power = safe_outputs::setParaPower(true);
  if (power != ESP_OK)
    return power;
  vTaskDelay(pdMS_TO_TICKS(flight_config::kParaPowerStabilizationMs));
  return ensureReady();
}

esp_err_t ParachuteActuator::ensureReady() {
  if (power_cutoff_.load(std::memory_order_acquire))
    return ESP_ERR_INVALID_STATE;
  if (ready_.load(std::memory_order_acquire) && servo_.initialized() &&
      bus_.initialized())
    return ESP_OK;

  endTransport();
  STSCREATE::Config bus_config{};
  bus_config.port = board::kParaUart;
  bus_config.tx = board::kParaTx;
  bus_config.rx = board::kParaRx;
  bus_config.direction_enable = GPIO_NUM_NC;
  bus_config.baudrate = STSCREATE::Baudrate::bps1000000;
  bus_config.lock_timeout = avi::Timeout::milliseconds(10);
  bus_config.tx_timeout = avi::Timeout::milliseconds(100);
  bus_config.response_timeout = avi::Timeout::milliseconds(100);
  esp_err_t result = bus_.begin(bus_config);
  if (result == ESP_OK)
    result = servo_.begin(bus_, board::kParaServoId);
  if (result == ESP_OK)
    result = servo_.configureStepMode(STS3215::Persistence::volatile_only);
  if (result == ESP_OK)
    result = servo_.setDirection(STS3215::Direction::normal,
                                 STS3215::Persistence::volatile_only);
  if (result == ESP_OK)
    result = holdCurrent();
  ready_.store(result == ESP_OK, std::memory_order_release);
  if (result != ESP_OK)
    endTransport();
  return result;
}

esp_err_t ParachuteActuator::holdCurrent() {
  if (!servo_.initialized())
    return ESP_ERR_INVALID_STATE;
  const esp_err_t result = servo_.holdCurrentPosition(
      {STS3215::TorqueLimit::percent(flight_config::kParaHoldTorquePercent)});
  if (result == ESP_OK)
    mode_.store(protocol::ParaMode::hold, std::memory_order_release);
  return result;
}

esp_err_t ParachuteActuator::startCommand(uint8_t transaction_id,
                                          protocol::CommandCode command) {
  if (transaction_id == 0 || busy())
    return ESP_ERR_INVALID_STATE;
  if (command == protocol::CommandCode::para_open)
    return startRelative(flight_config::kParaOpenDeltaDeg, ParaOperation::open,
                         transaction_id, static_cast<uint8_t>(command), 0);
  if (command == protocol::CommandCode::para_close)
    return startRelative(flight_config::kParaCloseDeltaDeg, ParaOperation::close,
                         transaction_id, static_cast<uint8_t>(command), 0);
  return ESP_ERR_INVALID_ARG;
}

esp_err_t ParachuteActuator::startDeployment(uint32_t generation) {
  if (generation == 0 || busy())
    return ESP_ERR_INVALID_STATE;
  return startRelative(flight_config::kParaOpenDeltaDeg,
                       ParaOperation::deployment, 0,
                       static_cast<uint8_t>(protocol::CommandCode::para_open),
                       generation);
}

esp_err_t ParachuteActuator::startRelative(float delta_deg,
                                           ParaOperation operation,
                                           uint8_t transaction_id,
                                           uint8_t command,
                                           uint32_t generation) {
  esp_err_t result = ensureReady();
  if (result != ESP_OK)
    return result;
  result = holdCurrent();
  if (result != ESP_OK)
    return result;

  STS3215::Motion motion{};
  motion.speed_deg_s = flight_config::kParaSpeedDegS;
  motion.acceleration_deg_s2 = flight_config::kParaAccelerationDegS2;
  motion.torque_limit =
      STS3215::TorqueLimit::percent(flight_config::kParaTorqueLimitPercent);
  result = servo_.moveRelativeDegrees(delta_deg, motion);
  if (result != ESP_OK) {
    (void)holdCurrent();
    return result;
  }

  operation_ = operation;
  transaction_id_ = transaction_id;
  command_ = command;
  generation_ = generation;
  deadline_us_ = static_cast<uint64_t>(esp_timer_get_time()) +
                 static_cast<uint64_t>(flight_config::kParaMotionTimeoutMs) *
                     1'000ULL;
  command_issued_ = true;
  mode_.store(operation == ParaOperation::close ? protocol::ParaMode::closing
                                                : protocol::ParaMode::opening_or_retrying,
              std::memory_order_release);
  return ESP_OK;
}

std::optional<ParaCompletion> ParachuteActuator::update(uint64_t now_us) {
  if (power_cutoff_.load(std::memory_order_acquire))
    return std::nullopt;

  if (!ready_.load(std::memory_order_acquire)) {
    if (now_us >= next_reconnect_us_) {
      (void)ensureReady();
      next_reconnect_us_ = now_us +
                           static_cast<uint64_t>(flight_config::kParaReconnectMs) *
                               1'000ULL;
    }
    return std::nullopt;
  }

  STS3215::Data data{};
  const esp_err_t read = servo_.read(data);
  if (read == ESP_OK && std::isfinite(data.position_deg)) {
    position_deg_.store(data.position_deg, std::memory_order_release);
    position_valid_.store(true, std::memory_order_release);
  } else {
    position_valid_.store(false, std::memory_order_release);
  }

  if (operation_ == ParaOperation::none)
    return std::nullopt;

  protocol::CommandReason reason = protocol::CommandReason::none;
  bool finished = false;
  if (read != ESP_OK) {
    reason = read == ESP_ERR_TIMEOUT ? protocol::CommandReason::timeout
                                     : protocol::CommandReason::device_unavailable;
    finished = true;
  } else if (!data.moving && command_issued_) {
    const esp_err_t hold = holdCurrent();
    reason = hold == ESP_OK ? protocol::CommandReason::none
                            : protocol::CommandReason::internal_error;
    finished = true;
  } else if (now_us >= deadline_us_) {
    reason = protocol::CommandReason::timeout;
    (void)holdCurrent();
    finished = true;
  }

  if (!finished)
    return std::nullopt;

  const ParaCompletion completion{transaction_id_, command_, reason};
  operation_ = ParaOperation::none;
  transaction_id_ = 0;
  command_ = 0;
  generation_ = 0;
  deadline_us_ = 0;
  command_issued_ = false;
  mode_.store(protocol::ParaMode::hold, std::memory_order_release);
  if (completion.transaction_id == 0)
    return std::nullopt;
  return completion;
}

void ParachuteActuator::endTransport() {
  ready_.store(false, std::memory_order_release);
  position_valid_.store(false, std::memory_order_release);
  if (servo_.initialized())
    (void)servo_.end();
  if (bus_.initialized())
    (void)bus_.end();
}

void ParachuteActuator::forcePowerOff() {
  power_cutoff_.store(true, std::memory_order_release);
  operation_ = ParaOperation::none;
  endTransport();
  (void)safe_outputs::setParaPower(false);
  mode_.store(protocol::ParaMode::powered_off, std::memory_order_release);
}

void ParachuteActuator::allowPower() {
  power_cutoff_.store(false, std::memory_order_release);
}

ParaTelemetry ParachuteActuator::telemetry() const {
  ParaTelemetry result{};
  result.mode = mode_.load(std::memory_order_acquire);
  result.position_valid = position_valid_.load(std::memory_order_acquire);
  result.position_deg = position_deg_.load(std::memory_order_acquire);
  result.ready = ready_.load(std::memory_order_acquire);
  return result;
}

} // namespace actuators
