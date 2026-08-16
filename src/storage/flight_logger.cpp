#include "storage/flight_logger.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <sys/stat.h>
#include <unistd.h>

#include "config/board.hpp"
#include "config/flight.hpp"
#include "driver/sdmmc_host.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"

namespace storage {
namespace {
constexpr char kMountPoint[] = "/sdcard";
constexpr char kFlightPath[] = "/sdcard/avi_99l_latest.bin";
constexpr uint32_t kPsramCaps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
}

FlightLogger::~FlightLogger() {
  flight_active_.store(false, std::memory_order_release);
  closeSd();
  releasePsram();
}

uint16_t FlightLogger::crc16(const uint8_t *data, std::size_t size) {
  uint16_t crc = 0xFFFFU;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= static_cast<uint16_t>(data[index]) << 8U;
    for (unsigned bit = 0; bit < 8U; ++bit)
      crc = (crc & 0x8000U) != 0U ? static_cast<uint16_t>((crc << 1U) ^ 0x1021U)
                                  : static_cast<uint16_t>(crc << 1U);
  }
  return crc;
}

LogRecord FlightLogger::serialize(const LogSample &sample,
                                  uint32_t dropped_before) {
  LogRecord record{};
  record.monotonic_us = sample.monotonic_us;
  record.flight_elapsed_us = sample.flight_elapsed_us;
  record.generation = sample.generation;
  record.dropped_before = dropped_before;
  record.phase = sample.phase;
  record.flags = sample.flags;
  record.fin_mode = sample.fin_mode;
  record.para_mode = sample.para_mode;
  record.ax = sample.acceleration_mg[0];
  record.ay = sample.acceleration_mg[1];
  record.az = sample.acceleration_mg[2];
  record.gx = sample.gyro_decidps[0];
  record.gy = sample.gyro_decidps[1];
  record.gz = sample.gyro_decidps[2];
  record.fin_angle_cdeg = sample.fin_angle_cdeg;
  record.fin_rate_cdeg_s = sample.fin_rate_cdeg_s;
  record.pressure_pa = sample.pressure_pa;
  record.para_angle_decideg = sample.para_angle_decideg;
  record.crc16 = 0;
  record.crc16 =
      crc16(reinterpret_cast<const uint8_t *>(&record), sizeof(record));
  return record;
}

esp_err_t FlightLogger::allocatePsram() {
  if (records_ != nullptr)
    return ESP_OK;
  const std::size_t largest = heap_caps_get_largest_free_block(kPsramCaps);
  if (largest <= flight_config::kPsramReserveBytes + kBatchBytes)
    return ESP_ERR_NO_MEM;
  std::size_t target = std::min(flight_config::kPsramMaxBytes,
                                largest - flight_config::kPsramReserveBytes);
  target -= target % sizeof(LogRecord);
  if (target < kBatchBytes)
    return ESP_ERR_NO_MEM;
  records_ = static_cast<LogRecord *>(heap_caps_malloc(target, kPsramCaps));
  if (records_ == nullptr)
    return ESP_ERR_NO_MEM;
  psram_bytes_ = target;
  capacity_records_ = target / sizeof(LogRecord);
  return ESP_OK;
}

void FlightLogger::releasePsram() {
  if (records_ != nullptr)
    heap_caps_free(records_);
  records_ = nullptr;
  psram_bytes_ = 0;
  capacity_records_ = 0;
}

esp_err_t FlightLogger::prepare() {
  const esp_err_t result = allocatePsram();
  if (result != ESP_OK)
    return result;
  if (writer_task_ == nullptr) {
    writer_task_ = xTaskCreateStatic(writerEntry, "SdWriter", kWriterStackWords,
                                     this, kWriterPriority,
                                     writer_stack_.data(), &writer_control_);
    if (writer_task_ == nullptr)
      return ESP_ERR_NO_MEM;
  }
  return ESP_OK;
}

esp_err_t FlightLogger::startFlight(uint32_t generation) {
  if (records_ == nullptr || capacity_records_ == 0U || generation == 0U)
    return ESP_ERR_INVALID_STATE;
  flight_active_.store(false, std::memory_order_release);
  const bool first_attempt = generation_.load(std::memory_order_acquire) == 0U;
  if (first_attempt) {
    write_index_.store(0, std::memory_order_release);
    read_index_.store(0, std::memory_order_release);
    dropped_.store(0, std::memory_order_release);
    high_water_.store(0, std::memory_order_release);
    truncate_pending_.store(true, std::memory_order_release);
  }
  generation_.store(generation, std::memory_order_release);
  flush_requested_.store(false, std::memory_order_release);
  flight_active_.store(true, std::memory_order_release);
  if (writer_task_ != nullptr)
    xTaskNotifyGive(writer_task_);
  return ESP_OK;
}

void FlightLogger::updateHighWater(uint32_t value) {
  uint32_t current = high_water_.load(std::memory_order_relaxed);
  while (current < value &&
         !high_water_.compare_exchange_weak(current, value,
                                            std::memory_order_relaxed,
                                            std::memory_order_relaxed)) {
  }
}

esp_err_t FlightLogger::append(const LogSample &sample) {
  if (!flight_active_.load(std::memory_order_acquire))
    return ESP_ERR_INVALID_STATE;
  if (sample.generation != generation_.load(std::memory_order_acquire))
    return ESP_ERR_INVALID_STATE;
  const uint64_t write = write_index_.load(std::memory_order_relaxed);
  const uint64_t read = read_index_.load(std::memory_order_acquire);
  if (write < read)
    return ESP_ERR_INVALID_STATE;
  const uint64_t occupancy = write - read;
  if (occupancy >= capacity_records_) {
    dropped_.fetch_add(1U, std::memory_order_relaxed);
    return ESP_ERR_NO_MEM;
  }
  records_[write % capacity_records_] =
      serialize(sample, dropped_.load(std::memory_order_relaxed));
  write_index_.store(write + 1U, std::memory_order_release);
  updateHighWater(static_cast<uint32_t>(std::min<uint64_t>(
      occupancy + 1U, std::numeric_limits<uint32_t>::max())));
  if (writer_task_ != nullptr && occupancy + 1U >= kBatchRecords)
    xTaskNotifyGive(writer_task_);
  return ESP_OK;
}

void FlightLogger::finishFlight() {
  flight_active_.store(false, std::memory_order_release);
  flush_requested_.store(true, std::memory_order_release);
  if (writer_task_ != nullptr)
    xTaskNotifyGive(writer_task_);
}

void FlightLogger::writerEntry(void *context) {
  static_cast<FlightLogger *>(context)->writerLoop();
}

esp_err_t FlightLogger::mountSd() {
  if (mounted_) {
    sd_health_.markHealthy();
    return ESP_OK;
  }

  sd_health_.markRecovering();
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
  slot.width = 4;
  slot.clk = board::kSdClk;
  slot.cmd = board::kSdCmd;
  slot.d0 = board::kSdDat0;
  slot.d1 = board::kSdDat1;
  slot.d2 = board::kSdDat2;
  slot.d3 = board::kSdDat3;
  esp_vfs_fat_sdmmc_mount_config_t mount{};
  mount.format_if_mount_failed = false;
  mount.max_files = 2;
  mount.allocation_unit_size = 16U * 1024U;
  const esp_err_t result =
      esp_vfs_fat_sdmmc_mount(kMountPoint, &host, &slot, &mount, &card_);
  mounted_ = result == ESP_OK;
  if (mounted_)
    sd_health_.markHealthy();
  else
    sd_health_.markFailed();
  return result;
}

void FlightLogger::closeSd() {
  if (fd_ >= 0) {
    (void)::close(fd_);
    fd_ = -1;
  }
  if (mounted_) {
    (void)esp_vfs_fat_sdcard_unmount(kMountPoint, card_);
    card_ = nullptr;
    mounted_ = false;
  }
}

esp_err_t FlightLogger::ensureFile() {
  if (fd_ >= 0)
    return ESP_OK;
  const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
  if (now < next_sd_retry_us_)
    return ESP_ERR_TIMEOUT;
  const esp_err_t mounted = mountSd();
  if (mounted != ESP_OK) {
    next_sd_retry_us_ = now + 1'000'000ULL;
    return mounted;
  }
  const bool truncate = truncate_pending_.load(std::memory_order_acquire);
  const int flags = O_RDWR | O_CREAT | (truncate ? O_TRUNC : 0);
  fd_ = ::open(kFlightPath, flags, 0664);
  if (fd_ < 0) {
    closeSd();
    sd_health_.markFailed();
    next_sd_retry_us_ = now + 1'000'000ULL;
    return ESP_FAIL;
  }
  if (!truncate && ::lseek(fd_, 0, SEEK_END) < 0) {
    closeSd();
    sd_health_.markFailed();
    next_sd_retry_us_ = now + 1'000'000ULL;
    return ESP_FAIL;
  }
  const off_t pos = ::lseek(fd_, 0, SEEK_CUR);
  file_bytes_ = pos >= 0 ? static_cast<uint64_t>(pos) : 0;
  truncate_pending_.store(false, std::memory_order_release);
  sd_health_.markHealthy();
  return ESP_OK;
}

esp_err_t FlightLogger::writeBatch(std::size_t count) {
  if (fd_ < 0 || count == 0U || count > kBatchRecords)
    return ESP_ERR_INVALID_ARG;
  const std::size_t bytes = count * sizeof(LogRecord);
  const off_t start = ::lseek(fd_, 0, SEEK_CUR);
  if (start < 0)
    return ESP_FAIL;
  std::size_t offset = 0;
  while (offset < bytes) {
    const ssize_t written =
        ::write(fd_, batch_.data() + offset, bytes - offset);
    if (written > 0) {
      offset += static_cast<std::size_t>(written);
      continue;
    }
    if (written < 0 && errno == EINTR)
      continue;
    (void)::ftruncate(fd_, start);
    (void)::lseek(fd_, start, SEEK_SET);
    return ESP_FAIL;
  }
  file_bytes_ += bytes;
  return ESP_OK;
}

bool FlightLogger::drainBatch(bool allow_partial) {
  const uint64_t read = read_index_.load(std::memory_order_relaxed);
  const uint64_t write = write_index_.load(std::memory_order_acquire);
  if (write < read)
    return false;
  const uint64_t available = write - read;
  if (available == 0U || (!allow_partial && available < kBatchRecords))
    return false;
  if (ensureFile() != ESP_OK)
    return false;

  const std::size_t count = static_cast<std::size_t>(
      std::min<uint64_t>(available, kBatchRecords));
  for (std::size_t index = 0; index < count; ++index) {
    const LogRecord &record = records_[(read + index) % capacity_records_];
    std::memcpy(batch_.data() + index * sizeof(LogRecord), &record,
                sizeof(LogRecord));
  }

  const esp_err_t result = writeBatch(count);
  if (result != ESP_OK) {
    closeSd();
    sd_health_.markFailed();
    next_sd_retry_us_ =
        static_cast<uint64_t>(esp_timer_get_time()) + 1'000'000ULL;
    return false;
  }

  sd_health_.markHealthy();
  read_index_.store(read + count, std::memory_order_release);
  return true;
}

void FlightLogger::writerLoop() {
  uint64_t next_sd_status_us = 0;

  for (;;) {
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
    const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());

    // 飛行前からhealthを出せるよう、SD probe/retryは低優先度writerだけが行う。
    if (!mounted_ && now >= next_sd_retry_us_) {
      const esp_err_t result = mountSd();
      if (result != ESP_OK)
        next_sd_retry_us_ = now + 1'000'000ULL;
    }

    if (mounted_ && card_ != nullptr && now >= next_sd_status_us) {
      if (sdmmc_get_status(card_) == ESP_OK) {
        sd_health_.markHealthy();
      } else {
        closeSd();
        sd_health_.markFailed();
        next_sd_retry_us_ = now + 1'000'000ULL;
      }
      next_sd_status_us = now + 1'000'000ULL;
    }

    const bool flush = flush_requested_.load(std::memory_order_acquire);
    while (drainBatch(flush)) {
    }

    if (flush && read_index_.load(std::memory_order_acquire) ==
                     write_index_.load(std::memory_order_acquire)) {
      if (fd_ >= 0) {
        if (::fsync(fd_) == 0) {
          sd_health_.markHealthy();
        } else {
          closeSd();
          sd_health_.markFailed();
          next_sd_retry_us_ = now + 1'000'000ULL;
        }
      }
      flush_requested_.store(false, std::memory_order_release);
    }
  }
}

} // namespace storage
