#pragma once

#include <atomic>
#include <cstdint>
#include <optional>

#include "STS3215.h"
#include "STSCREATE.h"
#include "esp_err.h"
#include "protocol/wire.hpp"

namespace actuators {

enum class ParaOperation : uint8_t { none, open, close, deployment };

struct ParaCompletion {
  uint8_t transaction_id{};
  uint8_t command{};
  protocol::CommandReason reason{protocol::CommandReason::none};
};

struct ParaTelemetry {
  protocol::ParaMode mode{protocol::ParaMode::powered_off};
  bool position_valid{};
  double position_deg{};
  bool ready{};
};

class ParachuteActuator {
public:
  [[nodiscard]] esp_err_t initialize();
  [[nodiscard]] esp_err_t startCommand(uint8_t transaction_id,
                                       protocol::CommandCode command);
  [[nodiscard]] esp_err_t startDeployment(uint32_t generation);
  [[nodiscard]] std::optional<ParaCompletion> update(uint64_t now_us);
  void forcePowerOff();
  void allowPower();

  [[nodiscard]] bool busy() const { return operation_ != ParaOperation::none; }
  [[nodiscard]] bool ready() const { return ready_.load(std::memory_order_acquire); }
  [[nodiscard]] ParaTelemetry telemetry() const;

private:
  [[nodiscard]] esp_err_t ensureReady();
  [[nodiscard]] esp_err_t holdCurrent();
  [[nodiscard]] esp_err_t startRelative(float delta_deg, ParaOperation operation,
                                        uint8_t transaction_id,
                                        uint8_t command,
                                        uint32_t generation);
  void endTransport();

  STSCREATE bus_{};
  STS3215 servo_{};
  std::atomic<bool> ready_{};
  std::atomic<bool> position_valid_{};
  std::atomic<double> position_deg_{};
  std::atomic<protocol::ParaMode> mode_{protocol::ParaMode::powered_off};
  std::atomic<bool> power_cutoff_{};

  ParaOperation operation_{ParaOperation::none};
  uint8_t transaction_id_{};
  uint8_t command_{};
  uint32_t generation_{};
  uint64_t deadline_us_{};
  bool command_issued_{};
  uint64_t next_reconnect_us_{};
};

} // namespace actuators
