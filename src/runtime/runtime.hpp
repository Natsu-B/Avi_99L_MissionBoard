#pragma once

#include <atomic>
#include <cstdint>

#include "CANCREATE.h"
#include "ICM42688.h"
#include "I2CCREATE.h"
#include "LPS25HB.h"
#include "SPICREATE.h"
#include "actuators/fin.hpp"
#include "actuators/parachute.hpp"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "mission/flight_detectors.hpp"
#include "mission/state_machine.hpp"
#include "protocol/command_cache.hpp"
#include "storage/flight_logger.hpp"

namespace runtime {

class Runtime {
public:
  [[nodiscard]] esp_err_t start();

private:
  struct ActuatorCommand {
    uint8_t transaction_id{};
    uint8_t command{};
  };

  static void safetyTaskEntry(void *context);
  static void realtimeTaskEntry(void *context);
  static void airTaskEntry(void *context);
  static void paraTaskEntry(void *context);
  static void canTaskEntry(void *context);

  void safetyTask();
  void realtimeTask();
  void airTask();
  void paraTask();
  void canTask();

  [[nodiscard]] esp_err_t initializeImu();
  [[nodiscard]] esp_err_t initializeAirData();
  [[nodiscard]] esp_err_t initializeCan();
  void pushResult(const protocol::CommandResult &result);
  [[nodiscard]] protocol::WireMissionState wireState() const;
  void sendCanFrame(const protocol::CanFrame &frame);

  mission::StateMachine state_{};
  actuators::FinActuator fin_{};
  actuators::ParachuteActuator para_{};
  storage::FlightLogger logger_{};

  SPICREATE imu_spi_{};
  ICM42688 imu_{};
  I2CCREATE air_i2c_{};
  LPS25HB lps_{};
  CANCREATE can_{};

  mission::ImuLiftoffDetector imu_liftoff_{};
  mission::LpsLiftoffDetector lps_liftoff_{};
  mission::PressureApexDetector apex_{};

  QueueHandle_t fin_command_queue_{};
  QueueHandle_t para_command_queue_{};
  QueueHandle_t result_queue_{};

  std::atomic<bool> fin_command_pending_{};
  std::atomic<bool> para_command_pending_{};
  std::atomic<bool> imu_valid_{};
  std::atomic<double> gyro_roll_rate_dps_{};
  std::atomic<bool> lps_valid_{};
  std::atomic<double> lps_pressure_hpa_{};
  std::atomic<double> lps_temperature_c_{};
  std::atomic<bool> logger_started_{};
  std::atomic<bool> logger_finished_{};

  protocol::CommandCache command_cache_{};
};

} // namespace runtime
