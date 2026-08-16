#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "diagnostics/device_health.hpp"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"

namespace storage {

struct LogSample {
  uint64_t monotonic_us{};
  uint64_t flight_elapsed_us{};
  uint32_t generation{};
  uint8_t phase{};
  uint8_t flags{};
  uint8_t fin_mode{};
  uint8_t para_mode{};
  std::array<int16_t, 3> acceleration_mg{};
  std::array<int16_t, 3> gyro_decidps{};
  int16_t fin_angle_cdeg{};
  int16_t fin_rate_cdeg_s{};
  int32_t pressure_pa{};
  int16_t para_angle_decideg{};
};

struct LogRecord {
  uint32_t magic{0x4C39394D};
  uint16_t version{1};
  uint16_t size{64};
  uint64_t monotonic_us{};
  uint64_t flight_elapsed_us{};
  uint32_t generation{};
  uint32_t dropped_before{};
  uint8_t phase{};
  uint8_t flags{};
  uint8_t fin_mode{};
  uint8_t para_mode{};
  int16_t ax{};
  int16_t ay{};
  int16_t az{};
  int16_t gx{};
  int16_t gy{};
  int16_t gz{};
  int16_t fin_angle_cdeg{};
  int16_t fin_rate_cdeg_s{};
  int32_t pressure_pa{};
  int16_t para_angle_decideg{};
  uint16_t crc16{};
  uint32_t reserved{};
};
static_assert(sizeof(LogRecord) == 64U, "LogRecord wire size must stay fixed");

class FlightLogger {
public:
  ~FlightLogger();
  [[nodiscard]] esp_err_t prepare();
  [[nodiscard]] esp_err_t startFlight(uint32_t generation);
  [[nodiscard]] esp_err_t append(const LogSample &sample);
  void finishFlight();

  [[nodiscard]] diagnostics::DeviceState sdState() const {
    return sd_health_.state();
  }
  [[nodiscard]] std::size_t psramBytes() const { return psram_bytes_; }
  [[nodiscard]] std::size_t capacityRecords() const { return capacity_records_; }
  [[nodiscard]] uint32_t droppedRecords() const {
    return dropped_.load(std::memory_order_relaxed);
  }
  [[nodiscard]] uint32_t highWaterRecords() const {
    return high_water_.load(std::memory_order_relaxed);
  }

private:
  static constexpr std::size_t kBatchBytes = 8192U;
  static constexpr std::size_t kBatchRecords = kBatchBytes / sizeof(LogRecord);
  static constexpr std::size_t kWriterStackWords = 4096U;
  static constexpr UBaseType_t kWriterPriority = 5U;

  [[nodiscard]] static uint16_t crc16(const uint8_t *data, std::size_t size);
  [[nodiscard]] static LogRecord serialize(const LogSample &sample,
                                           uint32_t dropped_before);
  static void writerEntry(void *context);
  void writerLoop();
  [[nodiscard]] esp_err_t allocatePsram();
  void releasePsram();
  [[nodiscard]] esp_err_t mountSd();
  void closeSd();
  [[nodiscard]] esp_err_t ensureFile();
  [[nodiscard]] bool drainBatch(bool allow_partial);
  [[nodiscard]] esp_err_t writeBatch(std::size_t count);
  void updateHighWater(uint32_t value);

  LogRecord *records_{};
  std::size_t psram_bytes_{};
  std::size_t capacity_records_{};
  std::atomic<uint64_t> write_index_{};
  std::atomic<uint64_t> read_index_{};
  std::atomic<uint32_t> dropped_{};
  std::atomic<uint32_t> high_water_{};

  std::atomic<bool> flight_active_{};
  std::atomic<uint32_t> generation_{};
  std::atomic<bool> truncate_pending_{};
  std::atomic<bool> flush_requested_{};

  diagnostics::DeviceHealth sd_health_{};
  sdmmc_card_t *card_{};
  bool mounted_{};
  int fd_{-1};
  uint64_t file_bytes_{};
  uint64_t next_sd_retry_us_{};

  alignas(LogRecord) std::array<uint8_t, kBatchBytes> batch_{};
  StaticTask_t writer_control_{};
  std::array<StackType_t, kWriterStackWords> writer_stack_{};
  TaskHandle_t writer_task_{};
};

} // namespace storage
