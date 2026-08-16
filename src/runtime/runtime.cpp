#include "runtime/runtime.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <limits>

#include "actuators/safe_outputs.hpp"
#include "config/board.hpp"
#include "config/flight.hpp"
#include "esp_timer.h"
#include "freertos/task.h"
#include "protocol/wire.hpp"

namespace runtime {
namespace {

constexpr uint32_t kTaskStackWords = 6144;

protocol::CommandReason reasonForEsp(esp_err_t result) {
  if (result == ESP_OK)
    return protocol::CommandReason::none;
  if (result == ESP_ERR_TIMEOUT)
    return protocol::CommandReason::timeout;
  if (result == ESP_ERR_INVALID_STATE)
    return protocol::CommandReason::busy;
  return protocol::CommandReason::device_unavailable;
}

bool allZero(const std::array<uint8_t, 6> &args) {
  return std::all_of(args.begin(), args.end(), [](uint8_t value) {
    return value == 0;
  });
}

uint64_t elapsed(const mission::Snapshot &snapshot, uint64_t now_us) {
  if (!snapshot.liftoff_valid || now_us < snapshot.liftoff_us)
    return 0;
  return now_us - snapshot.liftoff_us;
}

} // namespace

esp_err_t Runtime::start() {
  fin_command_queue_ = xQueueCreate(4, sizeof(ActuatorCommand));
  para_command_queue_ = xQueueCreate(4, sizeof(ActuatorCommand));
  result_queue_ = xQueueCreate(16, sizeof(protocol::CommandResult));
  if (fin_command_queue_ == nullptr || para_command_queue_ == nullptr ||
      result_queue_ == nullptr)
    return ESP_ERR_NO_MEM;

  const esp_err_t log = logger_.prepare();
  if (log != ESP_OK)
    std::printf("logger PSRAM prepare failed: %s\n", esp_err_to_name(log));

  const esp_err_t fin = fin_.initialize();
  if (fin != ESP_OK)
    std::printf("fin init failed: %s\n", esp_err_to_name(fin));

  (void)actuators::safe_outputs::setAux5v(true);
  const esp_err_t imu = initializeImu();
  if (imu != ESP_OK)
    std::printf("imu init failed: %s\n", esp_err_to_name(imu));
  const esp_err_t air = initializeAirData();
  if (air != ESP_OK)
    std::printf("air data init failed: %s\n", esp_err_to_name(air));
  const esp_err_t can = initializeCan();
  if (can != ESP_OK)
    std::printf("can init failed: %s\n", esp_err_to_name(can));

  if (xTaskCreate(safetyTaskEntry, "Safety", kTaskStackWords, this, 22,
                  nullptr) != pdPASS ||
      xTaskCreate(paraTaskEntry, "Para", kTaskStackWords, this, 21,
                  nullptr) != pdPASS ||
      xTaskCreate(realtimeTaskEntry, "Realtime", kTaskStackWords, this, 20,
                  nullptr) != pdPASS ||
      xTaskCreate(airTaskEntry, "AirData", kTaskStackWords, this, 18,
                  nullptr) != pdPASS ||
      xTaskCreate(canTaskEntry, "CAN", kTaskStackWords, this, 16,
                  nullptr) != pdPASS)
    return ESP_ERR_NO_MEM;

  return ESP_OK;
}

esp_err_t Runtime::initializeImu() {
  if (imu_.initialized())
    return ESP_OK;
  if (imu_spi_.initialized())
    (void)imu_spi_.end();
  SPICREATE::Config spi{};
  spi.host = board::kImuSpiHost;
  spi.sck = board::kImuSclk;
  spi.miso = board::kImuMiso;
  spi.mosi = board::kImuMosi;
  spi.transaction_timeout = avi::Timeout::milliseconds(2);
  esp_err_t result = imu_spi_.begin(spi);
  if (result != ESP_OK)
    return result;
  ICM42688::Config config{};
  config.frequency_hz = board::kImuSpiFrequencyHz;
  config.accel_range = ICM42688::AccelRange::g16;
  config.gyro_range = ICM42688::GyroRange::dps2000;
  config.accel_odr = ICM42688::AccelOdr::hz1000;
  config.gyro_odr = ICM42688::GyroOdr::hz1000;
  config.filter = ICM42688::Filter::odr_div4;
  config.int_gpio = board::kImuInterrupt;
  result = imu_.begin(imu_spi_, board::kImuCs, config);
  return result;
}

esp_err_t Runtime::initializeAirData() {
  if (lps_.initialized())
    return ESP_OK;
  if (air_i2c_.initialized())
    (void)air_i2c_.end();
  I2CCREATE::Config config{};
  config.port = board::kAirDataI2cPort;
  config.sda = board::kAirDataSda;
  config.scl = board::kAirDataScl;
  config.frequency_hz = board::kAirDataI2cFrequencyHz;
  config.operation_timeout = avi::Timeout::milliseconds(10);
  esp_err_t result = air_i2c_.begin(config);
  if (result != ESP_OK)
    return result;
  LPS25HB::Config lps_config{};
  lps_config.odr = LPS25HB::Odr::hz25;
  lps_config.pressure_average = LPS25HB::PressureAverage::samples8;
  lps_config.temperature_average = LPS25HB::TemperatureAverage::samples8;
  if (air_i2c_.probe(0x5C) == ESP_OK)
    result = lps_.begin(air_i2c_, LPS25HB::Address::low, lps_config);
  else if (air_i2c_.probe(0x5D) == ESP_OK)
    result = lps_.begin(air_i2c_, LPS25HB::Address::high, lps_config);
  else
    result = ESP_ERR_NOT_FOUND;
  return result;
}

esp_err_t Runtime::initializeCan() {
  if (can_.initialized())
    return ESP_OK;
  CANCREATE::Config config{};
  config.tx = board::kCanTx;
  config.rx = board::kCanRx;
  config.bitrate = CANCREATE::Bitrate::kbps125;
  config.mode = CANCREATE::Mode::normal;
  config.rx_queue_depth = 32;
  return can_.begin(config);
}

void Runtime::safetyTaskEntry(void *context) {
  static_cast<Runtime *>(context)->safetyTask();
}
void Runtime::realtimeTaskEntry(void *context) {
  static_cast<Runtime *>(context)->realtimeTask();
}
void Runtime::airTaskEntry(void *context) {
  static_cast<Runtime *>(context)->airTask();
}
void Runtime::paraTaskEntry(void *context) {
  static_cast<Runtime *>(context)->paraTask();
}
void Runtime::canTaskEntry(void *context) {
  static_cast<Runtime *>(context)->canTask();
}

void Runtime::safetyTask() {
  TickType_t wake = xTaskGetTickCount();
  bool cutoff_applied = false;
  for (;;) {
    const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
    const auto snapshot = state_.snapshot();
    if ((snapshot.phase == mission::Phase::flight ||
         snapshot.phase == mission::Phase::descent) &&
        snapshot.liftoff_valid) {
      const uint64_t flight_elapsed = elapsed(snapshot, now);
      if (snapshot.phase == mission::Phase::flight &&
          flight_elapsed >= flight_config::kDeploymentFallbackUs)
        (void)state_.requestDescent(snapshot.generation);
      if (flight_elapsed >= flight_config::kAbsolutePowerCutoffUs &&
          !cutoff_applied) {
        state_.latchPowerCutoff();
        (void)actuators::safe_outputs::setParaPower(false);
        (void)actuators::safe_outputs::setAux5v(false);
        (void)actuators::safe_outputs::motorCoast();
        cutoff_applied = true;
      }
    }
    if (snapshot.phase == mission::Phase::liftoff_detection)
      cutoff_applied = false;
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
  }
}

void Runtime::realtimeTask() {
  TickType_t wake = xTaskGetTickCount();
  mission::Phase previous_phase = mission::Phase::command_receive;
  uint64_t next_imu_retry = 0;
  bool local_logger_finished = false;

  for (;;) {
    const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
    auto snapshot = state_.snapshot();

    if (snapshot.phase != previous_phase) {
      if (snapshot.phase == mission::Phase::liftoff_detection)
        imu_liftoff_.reset();
      if (previous_phase == mission::Phase::flight &&
          snapshot.phase == mission::Phase::liftoff_detection) {
        logger_.finishFlight();
        logger_started_.store(false, std::memory_order_release);
      }
      previous_phase = snapshot.phase;
    }

    ICM42688::Data imu_data{};
    bool imu_sample_valid = false;
    if (!imu_.initialized() && now >= next_imu_retry) {
      const esp_err_t result = initializeImu();
      next_imu_retry = now + 1'000'000ULL;
      if (result != ESP_OK)
        imu_valid_.store(false, std::memory_order_release);
    }
    if (imu_.initialized() && imu_.read(imu_data) == ESP_OK) {
      imu_sample_valid = std::isfinite(imu_data.acceleration_g[0]) &&
                         std::isfinite(imu_data.acceleration_g[1]) &&
                         std::isfinite(imu_data.acceleration_g[2]) &&
                         std::isfinite(imu_data.angular_velocity_dps[2]);
      imu_valid_.store(imu_sample_valid, std::memory_order_release);
      if (imu_sample_valid)
        gyro_roll_rate_dps_.store(-imu_data.angular_velocity_dps[2],
                                  std::memory_order_release);
    } else {
      imu_valid_.store(false, std::memory_order_release);
    }

    if (snapshot.phase == mission::Phase::liftoff_detection && !imu_sample_valid)
      (void)imu_liftoff_.update(0.0, 0.0, 0.0, false);

    if (snapshot.phase == mission::Phase::liftoff_detection && imu_sample_valid) {
      if (imu_liftoff_.update(imu_data.acceleration_g[0],
                              imu_data.acceleration_g[1],
                              imu_data.acceleration_g[2], true)) {
        if (state_.reportLiftoff(now)) {
          snapshot = state_.snapshot();
          if (logger_.startFlight(snapshot.generation) == ESP_OK) {
            logger_started_.store(true, std::memory_order_release);
            logger_finished_.store(false, std::memory_order_release);
          }
          local_logger_finished = false;
        }
      }
    }

    fin_.update(now);

    ActuatorCommand fin_command{};
    while (xQueueReceive(fin_command_queue_, &fin_command, 0) == pdTRUE) {
      esp_err_t result = ESP_ERR_INVALID_ARG;
      if (fin_command.command ==
          static_cast<uint8_t>(protocol::CommandCode::fin_hold_current))
        result = fin_.holdCurrent();
      else if (fin_command.command ==
               static_cast<uint8_t>(protocol::CommandCode::fin_free))
        result = fin_.free();
      pushResult({fin_command.transaction_id, fin_command.command,
                  result == ESP_OK ? protocol::CommandPhase::completed
                                   : protocol::CommandPhase::failed,
                  reasonForEsp(result), 0});
      fin_command_pending_.store(false, std::memory_order_release);
    }

    snapshot = state_.snapshot();
    if (snapshot.power_cutoff) {
      fin_.forceSafe();
      if (!local_logger_finished) {
        logger_.finishFlight();
        logger_finished_.store(true, std::memory_order_release);
        local_logger_finished = true;
      }
    }

    if (logger_started_.load(std::memory_order_acquire) &&
        snapshot.liftoff_valid && !snapshot.power_cutoff) {
      const auto fin = fin_.telemetry();
      const auto para = para_.telemetry();
      storage::LogSample sample{};
      sample.monotonic_us = now;
      sample.flight_elapsed_us = elapsed(snapshot, now);
      sample.generation = snapshot.generation;
      sample.phase = static_cast<uint8_t>(snapshot.phase);
      sample.flags = static_cast<uint8_t>((imu_sample_valid ? 1U : 0U) |
                                          (lps_valid_.load() ? 2U : 0U));
      sample.fin_mode = static_cast<uint8_t>(
          fin.state == actuators::FinState::hold ? protocol::FinMode::zero_hold
          : fin.state == actuators::FinState::free ? protocol::FinMode::free
                                                   : protocol::FinMode::unknown);
      sample.para_mode = static_cast<uint8_t>(para.mode);
      if (imu_sample_valid) {
        for (std::size_t i = 0; i < 3; ++i) {
          sample.acceleration_mg[i] = static_cast<int16_t>(std::clamp<long>(
              std::lround(imu_data.acceleration_g[i] * 1000.0), -32768, 32767));
          sample.gyro_decidps[i] = static_cast<int16_t>(std::clamp<long>(
              std::lround(imu_data.angular_velocity_dps[i] * 10.0), -32768,
              32767));
        }
      }
      sample.fin_angle_cdeg = static_cast<int16_t>(std::clamp<long>(
          std::lround(fin.angle_deg * 100.0), -32768, 32767));
      sample.fin_rate_cdeg_s = static_cast<int16_t>(std::clamp<long>(
          std::lround(fin.rate_deg_s * 100.0), -32768, 32767));
      sample.pressure_pa = static_cast<int32_t>(std::clamp<long long>(
          std::llround(lps_pressure_hpa_.load(std::memory_order_acquire) * 100.0),
          std::numeric_limits<int32_t>::min(),
          std::numeric_limits<int32_t>::max()));
      sample.para_angle_decideg = static_cast<int16_t>(std::clamp<long>(
          std::lround(para.position_deg * 10.0), -32768, 32767));
      (void)logger_.append(sample);
    }

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
  }
}

void Runtime::airTask() {
  TickType_t wake = xTaskGetTickCount();
  mission::Phase previous_phase = mission::Phase::command_receive;
  uint64_t next_retry = 0;
  for (;;) {
    const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
    const auto snapshot = state_.snapshot();
    if (snapshot.phase != previous_phase) {
      if (snapshot.phase == mission::Phase::liftoff_detection)
        lps_liftoff_.reset();
      if (snapshot.phase == mission::Phase::flight)
        apex_.reset();
      previous_phase = snapshot.phase;
    }

    if (!lps_.initialized() && now >= next_retry) {
      const esp_err_t result = initializeAirData();
      next_retry = now + 1'000'000ULL;
      if (result != ESP_OK)
        lps_valid_.store(false, std::memory_order_release);
    }

    bool valid = false;
    LPS25HB::Data data{};
    if (lps_.initialized()) {
      bool ready = false;
      if (lps_.available(ready) == ESP_OK && ready && lps_.read(data) == ESP_OK &&
          std::isfinite(data.pressure_pa) &&
          std::isfinite(data.temperature_celsius)) {
        valid = true;
        lps_pressure_hpa_.store(data.pressure_pa / 100.0,
                                std::memory_order_release);
        lps_temperature_c_.store(data.temperature_celsius,
                                 std::memory_order_release);
      }
    }
    lps_valid_.store(valid, std::memory_order_release);

    if (!valid && snapshot.phase == mission::Phase::liftoff_detection)
      (void)lps_liftoff_.update(0.0, false);
    if (!valid && snapshot.phase == mission::Phase::flight)
      (void)apex_.update(0.0, false, elapsed(snapshot, now));

    if (valid && snapshot.phase == mission::Phase::liftoff_detection) {
      if (lps_liftoff_.update(data.pressure_pa / 100.0, true)) {
        if (state_.reportLiftoff(now)) {
          const auto current = state_.snapshot();
          if (logger_.startFlight(current.generation) == ESP_OK) {
            logger_started_.store(true, std::memory_order_release);
            logger_finished_.store(false, std::memory_order_release);
          }
        }
      }
    } else if (valid && snapshot.phase == mission::Phase::flight &&
               snapshot.liftoff_valid) {
      if (apex_.update(data.pressure_pa / 100.0, true, elapsed(snapshot, now)))
        (void)state_.requestDescent(snapshot.generation);
    }

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(40));
  }
}

void Runtime::paraTask() {
  (void)para_.initialize();
  uint32_t deployment_started_generation = 0;
  uint64_t next_deployment_retry = 0;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
    const auto snapshot = state_.snapshot();
    if (snapshot.power_cutoff) {
      para_.forcePowerOff();
    } else {
      para_.allowPower();
      const auto completion = para_.update(now);
      if (completion.has_value()) {
        pushResult({completion->transaction_id, completion->command,
                    completion->reason == protocol::CommandReason::none
                        ? protocol::CommandPhase::completed
                        : protocol::CommandPhase::failed,
                    completion->reason, 0});
        para_command_pending_.store(false, std::memory_order_release);
      }

      ActuatorCommand command{};
      if (!para_.busy() &&
          xQueueReceive(para_command_queue_, &command, 0) == pdTRUE) {
        const esp_err_t result = para_.startCommand(
            command.transaction_id,
            static_cast<protocol::CommandCode>(command.command));
        if (result != ESP_OK) {
          pushResult({command.transaction_id, command.command,
                      protocol::CommandPhase::failed, reasonForEsp(result), 0});
          para_command_pending_.store(false, std::memory_order_release);
        }
      }

      if (snapshot.phase == mission::Phase::descent &&
          snapshot.generation != 0 &&
          deployment_started_generation != snapshot.generation && !para_.busy() &&
          now >= next_deployment_retry) {
        const esp_err_t result = para_.startDeployment(snapshot.generation);
        if (result == ESP_OK)
          deployment_started_generation = snapshot.generation;
        else
          next_deployment_retry = now + 250'000ULL;
      }
      if (snapshot.phase == mission::Phase::liftoff_detection)
        deployment_started_generation = 0;
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
  }
}

void Runtime::pushResult(const protocol::CommandResult &result) {
  if (result_queue_ != nullptr)
    (void)xQueueSend(result_queue_, &result, 0);
}

protocol::WireMissionState Runtime::wireState() const {
  switch (state_.snapshot().phase) {
  case mission::Phase::command_receive:
    return protocol::WireMissionState::command_receive;
  case mission::Phase::liftoff_detection:
    return protocol::WireMissionState::liftoff_detection;
  case mission::Phase::flight:
    return protocol::WireMissionState::engine_burn;
  case mission::Phase::descent:
    return protocol::WireMissionState::descent;
  }
  return protocol::WireMissionState::command_receive;
}

void Runtime::sendCanFrame(const protocol::CanFrame &input) {
  CANCREATE::Frame frame{};
  frame.identifier = input.identifier;
  frame.data_length = input.data_length;
  frame.extended = input.extended;
  frame.remote = input.remote;
  std::copy(input.data.begin(), input.data.end(), std::begin(frame.data));
  (void)can_.write(frame, avi::Timeout::milliseconds(2));
}

void Runtime::canTask() {
  uint8_t status_sequence = 0;
  uint8_t kinematics_sequence = 0;
  uint8_t lps_sequence = 0;
  uint8_t airspeed_sequence = 0;
  uint64_t next_status = 0;
  uint64_t next_kinematics = 0;
  uint64_t next_lps = 0;
  uint64_t next_airspeed = 0;
  TickType_t wake = xTaskGetTickCount();
  uint64_t next_can_retry = 0;
  uint64_t next_can_status = 0;

  for (;;) {
    const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
    if (!can_.initialized() && now >= next_can_retry) {
      (void)initializeCan();
      next_can_retry = now + 1'000'000ULL;
    }
    if (can_.initialized() && now >= next_can_status) {
      CANCREATE::Status status{};
      if (can_.getStatus(status) == ESP_OK &&
          status.state == CANCREATE::State::bus_off)
        (void)can_.recover(avi::Timeout::milliseconds(10));
      next_can_status = now + 100'000ULL;
    }
    CANCREATE::Frame raw{};
    while (can_.read(raw, avi::Timeout::noWait()) == ESP_OK) {
      protocol::CanFrame frame{};
      frame.identifier = raw.identifier;
      frame.data_length = raw.data_length;
      frame.extended = raw.extended;
      frame.remote = raw.remote;
      std::copy(std::begin(raw.data), std::end(raw.data), frame.data.begin());

      protocol::GenericCommandRequest request{};
      if (protocol::decodeGenericCommand(frame, request)) {
        if (request.transaction_id == 0 || !allZero(request.arguments)) {
          sendCanFrame(protocol::encode(
              {request.transaction_id, request.command,
               protocol::CommandPhase::rejected,
               protocol::CommandReason::invalid_argument, 0}));
          continue;
        }
        const auto lookup = command_cache_.lookup(request);
        if (lookup.kind != protocol::CommandCache::Lookup::miss) {
          sendCanFrame(protocol::encode(lookup.result));
          continue;
        }

        const auto code = static_cast<protocol::CommandCode>(request.command);
        const auto phase = state_.snapshot().phase;
        if (code == protocol::CommandCode::start_sequence) {
          command_cache_.rememberAccepted(request);
          sendCanFrame(protocol::encode(
              {request.transaction_id, request.command,
               protocol::CommandPhase::accepted, protocol::CommandReason::none,
               0}));
          const bool ok = state_.startSequence();
          const protocol::CommandResult final{
              request.transaction_id, request.command,
              ok ? protocol::CommandPhase::completed
                 : protocol::CommandPhase::failed,
              ok ? protocol::CommandReason::none
                 : protocol::CommandReason::invalid_state,
              0};
          command_cache_.finish(final);
          sendCanFrame(protocol::encode(final));
          continue;
        }

        const bool is_fin = code == protocol::CommandCode::fin_free ||
                            code == protocol::CommandCode::fin_hold_current;
        const bool is_para = code == protocol::CommandCode::para_open ||
                             code == protocol::CommandCode::para_close;
        if ((is_fin || is_para) && phase != mission::Phase::command_receive) {
          const protocol::CommandResult rejected{
              request.transaction_id, request.command,
              protocol::CommandPhase::rejected,
              protocol::CommandReason::invalid_state, 0};
          command_cache_.rememberAccepted(request);
          command_cache_.finish(rejected);
          sendCanFrame(protocol::encode(rejected));
          continue;
        }

        if (is_fin) {
          bool expected = false;
          if (!fin_command_pending_.compare_exchange_strong(expected, true)) {
            const protocol::CommandResult rejected{
                request.transaction_id, request.command,
                protocol::CommandPhase::rejected,
                protocol::CommandReason::busy, 0};
            command_cache_.rememberAccepted(request);
            command_cache_.finish(rejected);
            sendCanFrame(protocol::encode(rejected));
            continue;
          }
          const ActuatorCommand command{request.transaction_id, request.command};
          if (xQueueSend(fin_command_queue_, &command, 0) != pdTRUE) {
            fin_command_pending_.store(false);
            sendCanFrame(protocol::encode(
                {request.transaction_id, request.command,
                 protocol::CommandPhase::rejected,
                 protocol::CommandReason::busy, 0}));
            continue;
          }
          command_cache_.rememberAccepted(request);
          sendCanFrame(protocol::encode(
              {request.transaction_id, request.command,
               protocol::CommandPhase::accepted, protocol::CommandReason::none,
               0}));
          continue;
        }

        if (is_para) {
          bool expected = false;
          if (!para_command_pending_.compare_exchange_strong(expected, true)) {
            const protocol::CommandResult rejected{
                request.transaction_id, request.command,
                protocol::CommandPhase::rejected,
                protocol::CommandReason::busy, 0};
            command_cache_.rememberAccepted(request);
            command_cache_.finish(rejected);
            sendCanFrame(protocol::encode(rejected));
            continue;
          }
          const ActuatorCommand command{request.transaction_id, request.command};
          if (xQueueSend(para_command_queue_, &command, 0) != pdTRUE) {
            para_command_pending_.store(false);
            sendCanFrame(protocol::encode(
                {request.transaction_id, request.command,
                 protocol::CommandPhase::rejected,
                 protocol::CommandReason::busy, 0}));
            continue;
          }
          command_cache_.rememberAccepted(request);
          sendCanFrame(protocol::encode(
              {request.transaction_id, request.command,
               protocol::CommandPhase::accepted, protocol::CommandReason::none,
               0}));
          continue;
        }

        const protocol::CommandResult rejected{
            request.transaction_id, request.command,
            protocol::CommandPhase::rejected,
            protocol::CommandReason::not_supported, 0};
        command_cache_.rememberAccepted(request);
        command_cache_.finish(rejected);
        sendCanFrame(protocol::encode(rejected));
        continue;
      }

      uint8_t emergency_transaction = 0;
      if (protocol::decodeEmergency(frame, emergency_transaction)) {
        protocol::GenericCommandRequest replay_key{};
        replay_key.transaction_id = emergency_transaction;
        replay_key.command = static_cast<uint8_t>(
            protocol::CommandCode::liftoff_emergency_result);
        const auto lookup = command_cache_.lookup(replay_key);
        if (lookup.kind != protocol::CommandCache::Lookup::miss) {
          sendCanFrame(protocol::encode(lookup.result));
          continue;
        }
        if (emergency_transaction != 0)
          command_cache_.rememberAccepted(replay_key);
        const bool accepted = emergency_transaction != 0 &&
                              state_.liftoffEmergencyRollback();
        const protocol::CommandResult result{
            emergency_transaction,
            static_cast<uint8_t>(protocol::CommandCode::liftoff_emergency_result),
            accepted ? protocol::CommandPhase::completed
                     : protocol::CommandPhase::rejected,
            accepted ? protocol::CommandReason::none
                     : (emergency_transaction == 0
                            ? protocol::CommandReason::invalid_argument
                            : protocol::CommandReason::invalid_state),
            0};
        if (emergency_transaction != 0)
          command_cache_.finish(result);
        sendCanFrame(protocol::encode(result));
      }
    }

    protocol::CommandResult task_result{};
    while (xQueueReceive(result_queue_, &task_result, 0) == pdTRUE) {
      command_cache_.finish(task_result);
      sendCanFrame(protocol::encode(task_result));
    }

    if (now >= next_kinematics) {
      const auto fin = fin_.telemetry();
      protocol::Kinematics message{};
      message.sequence = kinematics_sequence++;
      message.roll_rate_raw = protocol::encodeRollRate(
          gyro_roll_rate_dps_.load(std::memory_order_acquire),
          imu_valid_.load(std::memory_order_acquire));
      message.fin_angle_raw = protocol::encodeFinAngle(fin.angle_deg,
                                                       fin.encoder_valid && fin.zero_valid);
      message.fin_rate_raw = protocol::encodeFinRate(fin.rate_deg_s,
                                                     fin.encoder_valid);
      sendCanFrame(protocol::encode(message));
      next_kinematics = now + 10'000ULL;
    }

    if (now >= next_status) {
      const auto snapshot = state_.snapshot();
      const auto fin = fin_.telemetry();
      const auto para = para_.telemetry();
      protocol::MissionStatus message{};
      message.sequence = status_sequence++;
      message.state = wireState();
      message.flight_status = static_cast<uint16_t>(
          (imu_valid_.load() ? 1U : 0U) |
          (lps_valid_.load() ? 2U : 0U) |
          (fin.zero_valid ? 4U : 0U) |
          (para.ready ? 8U : 0U) |
          (snapshot.deployment_started ? 16U : 0U) |
          (snapshot.power_cutoff ? 32U : 0U));
      message.fin_mode =
          fin.state == actuators::FinState::hold ? protocol::FinMode::zero_hold
          : fin.state == actuators::FinState::free ? protocol::FinMode::free
                                                   : protocol::FinMode::unknown;
      message.para_mode = para.mode;
      message.parachute_angle_raw =
          protocol::encodeParachuteAngle(para.position_deg, para.position_valid);
      sendCanFrame(protocol::encode(message));
      next_status = now + 100'000ULL;
    }

    if (now >= next_lps) {
      protocol::LpsTelemetry message{};
      message.sequence = lps_sequence++;
      const bool valid = lps_valid_.load(std::memory_order_acquire);
      message.pressure_raw = protocol::encodeLpsPressure(
          lps_pressure_hpa_.load(std::memory_order_acquire), valid);
      message.temperature_raw = protocol::encodeLpsTemperature(
          lps_temperature_c_.load(std::memory_order_acquire), valid);
      sendCanFrame(protocol::encode(message));
      next_lps = now + 40'000ULL;
    }

    if (now >= next_airspeed) {
      protocol::AirspeedTelemetry message{};
      message.sequence = airspeed_sequence++;
      message.airspeed_raw = 249; // SSCは次段で追加する。
      sendCanFrame(protocol::encode(message));
      next_airspeed = now + 10'000ULL;
    }

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
  }
}

} // namespace runtime
