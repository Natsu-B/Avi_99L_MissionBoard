#include "runtime/runtime.hpp"

#include <algorithm>
#include <cstdio>
#include <iterator>

#include "actuators/safe_outputs.hpp"
#include "config/board.hpp"
#include "esp_timer.h"
#include "freertos/task.h"
#include "protocol/wire.hpp"

namespace runtime {
namespace {

constexpr uint32_t kTaskStackWords = 6144;
constexpr uint32_t kRecoveryTaskStackWords = 4096;
constexpr UBaseType_t kRecoveryTaskPriority = 8;

} // namespace

void Runtime::resetControlSession() {
  control_session_.reset();
  control_active_.store(false, std::memory_order_release);
  control_permanently_disabled_.store(false, std::memory_order_release);
  control_reference_valid_.store(false, std::memory_order_release);
  control_roll_deviation_rad_.store(0.0, std::memory_order_release);
  requested_control_torque_nm_.store(0.0, std::memory_order_release);
}

esp_err_t Runtime::start() {
  fin_command_queue_ = xQueueCreate(4, sizeof(ActuatorCommand));
  para_command_queue_ = xQueueCreate(4, sizeof(ActuatorCommand));
  result_queue_ = xQueueCreate(16, sizeof(protocol::CommandResult));
  imu_mutex_ = xSemaphoreCreateMutex();
  if (fin_command_queue_ == nullptr || para_command_queue_ == nullptr ||
      result_queue_ == nullptr || imu_mutex_ == nullptr)
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
                  nullptr) != pdPASS ||
      xTaskCreate(recoveryTaskEntry, "Recovery", kRecoveryTaskStackWords, this,
                  kRecoveryTaskPriority, nullptr) != pdPASS)
    return ESP_ERR_NO_MEM;

  return ESP_OK;
}

esp_err_t Runtime::initializeImu() {
  if (imu_.initialized()) {
    imu_health_.markHealthy();
    return ESP_OK;
  }

  if (imu_spi_.initialized()) {
    const esp_err_t end_result = imu_spi_.end();
    if (end_result != ESP_OK) {
      imu_health_.markFailed();
      return end_result;
    }
  }

  SPICREATE::Config spi{};
  spi.host = board::kImuSpiHost;
  spi.sck = board::kImuSclk;
  spi.miso = board::kImuMiso;
  spi.mosi = board::kImuMosi;
  // SPI3はICM42688専用。Recoveryとのmutex競合で1 kHz taskを待たせない。
  spi.transaction_timeout = avi::Timeout::noWait();

  esp_err_t result = imu_spi_.begin(spi);
  if (result != ESP_OK) {
    imu_health_.markFailed();
    return result;
  }

  ICM42688::Config config{};
  config.frequency_hz = board::kImuSpiFrequencyHz;
  config.accel_range = ICM42688::AccelRange::g16;
  config.gyro_range = ICM42688::GyroRange::dps2000;
  config.accel_odr = ICM42688::AccelOdr::hz1000;
  config.gyro_odr = ICM42688::GyroOdr::hz1000;
  config.filter = ICM42688::Filter::odr_div4;
  config.int_gpio = board::kImuInterrupt;

  result = imu_.begin(imu_spi_, board::kImuCs, config);
  if (result != ESP_OK) {
    if (imu_spi_.initialized())
      (void)imu_spi_.end();
    imu_health_.markFailed();
    return result;
  }

  imu_health_.markHealthy();
  return ESP_OK;
}

esp_err_t Runtime::recoverImu() {
  if (!imu_health_.recoveryRequested())
    return ESP_OK;
  if (imu_mutex_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  if (xSemaphoreTake(imu_mutex_, pdMS_TO_TICKS(20)) != pdTRUE)
    return ESP_ERR_TIMEOUT;

  imu_health_.markRecovering();
  imu_valid_.store(false, std::memory_order_release);

  esp_err_t result = ESP_OK;
  if (imu_.initialized())
    result = imu_.end();
  if (result == ESP_OK && imu_spi_.initialized())
    result = imu_spi_.end();
  if (result == ESP_OK)
    result = initializeImu();

  if (result != ESP_OK)
    imu_health_.markFailed();

  xSemaphoreGive(imu_mutex_);
  return result;
}

esp_err_t Runtime::initializeAirData() {
  if (!air_i2c_.initialized()) {
    I2CCREATE::Config config{};
    config.port = board::kAirDataI2cPort;
    config.sda = board::kAirDataSda;
    config.scl = board::kAirDataScl;
    config.frequency_hz = board::kAirDataI2cFrequencyHz;
    config.operation_timeout = avi::Timeout::milliseconds(10);
    const esp_err_t bus_result = air_i2c_.begin(config);
    if (bus_result != ESP_OK) {
      if (!lps_.initialized())
        lps_health_.markFailed();
      if (!ssc_.initialized())
        ssc_health_.markFailed();
      return bus_result;
    }
  }

  esp_err_t lps_result = ESP_OK;
  if (!lps_.initialized()) {
    LPS25HB::Config lps_config{};
    lps_config.odr = LPS25HB::Odr::hz25;
    lps_config.pressure_average = LPS25HB::PressureAverage::samples8;
    lps_config.temperature_average = LPS25HB::TemperatureAverage::samples8;

    if (air_i2c_.probe(0x5C) == ESP_OK)
      lps_result = lps_.begin(air_i2c_, LPS25HB::Address::low, lps_config);
    else if (air_i2c_.probe(0x5D) == ESP_OK)
      lps_result = lps_.begin(air_i2c_, LPS25HB::Address::high, lps_config);
    else
      lps_result = ESP_ERR_NOT_FOUND;

    if (lps_result == ESP_OK)
      lps_health_.markHealthy();
    else
      lps_health_.markFailed();
  }

  esp_err_t ssc_result = ESP_OK;
  if (!ssc_.initialized()) {
    ssc_result = ssc_.begin(air_i2c_);
    if (ssc_result == ESP_OK)
      ssc_health_.markHealthy();
    else
      ssc_health_.markFailed();
  }

  if (lps_.initialized() || ssc_.initialized())
    return ESP_OK;
  return lps_result != ESP_OK ? lps_result : ssc_result;
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
void Runtime::recoveryTaskEntry(void *context) {
  static_cast<Runtime *>(context)->recoveryTask();
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
    return control_active_.load(std::memory_order_acquire)
               ? protocol::WireMissionState::control
               : protocol::WireMissionState::engine_burn;
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

void Runtime::recoveryTask() {
  TickType_t wake = xTaskGetTickCount();
  uint64_t next_imu_retry = 0;
  uint64_t next_encoder_retry = 0;

  for (;;) {
    const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
    const auto snapshot = state_.snapshot();

    // 回収clockやPara taskの前提条件にはせず、低優先度best-effortで復旧する。
    if (!snapshot.power_cutoff && imu_health_.recoveryRequested()) {
      if (next_imu_retry == 0) {
        next_imu_retry = now + 1'000'000ULL;
      } else if (now >= next_imu_retry) {
        const esp_err_t result = recoverImu();
        next_imu_retry = now + 1'000'000ULL;
        if (result != ESP_OK)
          std::printf("imu recovery failed: %s\n", esp_err_to_name(result));
      }
    } else {
      next_imu_retry = 0;
    }

    if (!snapshot.power_cutoff && fin_.encoderRecoveryRequested()) {
      if (next_encoder_retry == 0) {
        next_encoder_retry = now + 1'000'000ULL;
      } else if (now >= next_encoder_retry) {
        const esp_err_t result = fin_.recoverEncoder();
        next_encoder_retry = now + 1'000'000ULL;
        if (result != ESP_OK)
          std::printf("encoder recovery failed: %s\n", esp_err_to_name(result));
      }
    } else {
      next_encoder_retry = 0;
    }

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(20));
  }
}

} // namespace runtime
