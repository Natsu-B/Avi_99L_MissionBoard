#pragma once

#include <atomic>
#include <cstdint>

#include "CANCREATE.h"
#include "ICM42688.h"
#include "I2CCREATE.h"
#include "LPS25HB.h"
#include "SPICREATE.h"
#include "SSCDRRN005PD2A5.h"
#include "actuators/fin.hpp"
#include "actuators/parachute.hpp"
#include "config/flight.hpp"
#include "control/roll_control.hpp"
#include "diagnostics/device_health.hpp"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "mission/flight_detectors.hpp"
#include "mission/state_machine.hpp"
#include "protocol/command_cache.hpp"
#include "sensors/airspeed.hpp"
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
  static void recoveryTaskEntry(void *context);

  void safetyTask();
  void realtimeTask();
  void airTask();
  void paraTask();
  void canTask();
  void recoveryTask();

  [[nodiscard]] esp_err_t initializeImu();
  [[nodiscard]] esp_err_t recoverImu();
  [[nodiscard]] esp_err_t initializeAirData();
  [[nodiscard]] esp_err_t initializeCan();
  void pushResult(const protocol::CommandResult &result);
  [[nodiscard]] protocol::WireMissionState wireState() const;
  void sendCanFrame(const protocol::CanFrame &frame);
  void resetControlSession();

  mission::StateMachine state_{};
  actuators::FinActuator fin_{};
  actuators::ParachuteActuator para_{};
  storage::FlightLogger logger_{};

  SPICREATE imu_spi_{};
  ICM42688 imu_{};
  SemaphoreHandle_t imu_mutex_{};
  I2CCREATE air_i2c_{};
  LPS25HB lps_{};
  SSCDRRN005PD2A5 ssc_{};
  CANCREATE can_{};

  diagnostics::DeviceHealth imu_health_{};
  diagnostics::DeviceHealth lps_health_{};
  diagnostics::DeviceHealth ssc_health_{};

  mission::ImuLiftoffDetector imu_liftoff_{};
  mission::LpsLiftoffDetector lps_liftoff_{};
  mission::PressureApexDetector apex_{};
  sensors::DifferentialPressureFilter differential_pressure_filter_{
      flight_config::kSscZeroOffsetPa,
      flight_config::kDifferentialPressureNegativeTolerancePa,
      flight_config::kDifferentialPressureMovingAverageSamples};
  control::RollController roll_controller_{
      flight_config::kRollGainSchedule,
      flight_config::kRollControlTorqueLimitNm};
  control::FlightControlSession control_session_{
      flight_config::kGyroIntegrationMaximumGapUs,
      flight_config::kAirspeedPermanentStopMps};

  QueueHandle_t fin_command_queue_{};
  QueueHandle_t para_command_queue_{};
  QueueHandle_t result_queue_{};

  std::atomic<bool> fin_command_pending_{};
  std::atomic<bool> para_command_pending_{};

  std::atomic<bool> imu_valid_{};
  std::atomic<uint64_t> imu_sample_us_{};
  std::atomic<double> gyro_roll_rate_dps_{};
  std::atomic<bool> imu_liftoff_detected_{};

  std::atomic<bool> lps_valid_{};
  std::atomic<uint64_t> lps_sample_us_{};
  std::atomic<double> lps_pressure_hpa_{};
  std::atomic<double> lps_temperature_c_{};
  std::atomic<bool> lps_liftoff_detected_{};

  std::atomic<bool> ssc_valid_{};
  std::atomic<uint64_t> ssc_sample_us_{};
  std::atomic<double> differential_pressure_pa_{};
  std::atomic<double> ssc_temperature_c_{};

  std::atomic<bool> airspeed_valid_{};
  std::atomic<uint64_t> airspeed_sample_us_{};
  std::atomic<double> airspeed_mps_{};

  std::atomic<bool> control_active_{};
  std::atomic<bool> control_permanently_disabled_{};
  std::atomic<bool> control_reference_valid_{};
  std::atomic<double> control_roll_deviation_rad_{};
  std::atomic<double> requested_control_torque_nm_{};
  std::atomic<uint8_t> reference_capture_event_sequence_{};

  std::atomic<bool> logger_started_{};
  std::atomic<bool> logger_finished_{};

  protocol::CommandCache command_cache_{};
};

} // namespace runtime
