#include "actuators/safe_outputs.hpp"

#include <atomic>

#include "config/board.hpp"
#include "driver/gpio.h"

namespace actuators::safe_outputs {
namespace {
std::atomic<bool> ready{false};

esp_err_t outputLow(gpio_num_t pin) {
  gpio_config_t config{};
  config.pin_bit_mask = uint64_t{1} << static_cast<unsigned>(pin);
  config.mode = GPIO_MODE_OUTPUT;
  config.pull_up_en = GPIO_PULLUP_DISABLE;
  config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  config.intr_type = GPIO_INTR_DISABLE;
  esp_err_t result = gpio_config(&config);
  if (result == ESP_OK)
    result = gpio_set_level(pin, 0);
  return result;
}
} // namespace

esp_err_t initialize() {
  esp_err_t result = outputLow(board::kMotorIn1);
  if (result == ESP_OK)
    result = outputLow(board::kMotorIn2);
  if (result == ESP_OK)
    result = outputLow(board::kAux5vEnable);
  if (result == ESP_OK)
    result = outputLow(board::kParaEnable);
  if (result == ESP_OK)
    result = outputLow(board::kLed1);
  if (result == ESP_OK)
    result = outputLow(board::kLed2);
  ready.store(result == ESP_OK, std::memory_order_release);
  return result;
}

esp_err_t motorCoast() {
  if (!ready.load(std::memory_order_acquire))
    return ESP_ERR_INVALID_STATE;
  esp_err_t result = gpio_set_direction(board::kMotorIn1, GPIO_MODE_OUTPUT);
  if (result == ESP_OK)
    result = gpio_set_direction(board::kMotorIn2, GPIO_MODE_OUTPUT);
  if (result == ESP_OK)
    result = gpio_set_level(board::kMotorIn1, 0);
  if (result == ESP_OK)
    result = gpio_set_level(board::kMotorIn2, 0);
  return result;
}

esp_err_t setAux5v(bool enabled) {
  if (!ready.load(std::memory_order_acquire))
    return ESP_ERR_INVALID_STATE;
  return gpio_set_level(board::kAux5vEnable, enabled ? 1 : 0);
}

esp_err_t setParaPower(bool enabled) {
  if (!ready.load(std::memory_order_acquire))
    return ESP_ERR_INVALID_STATE;
  return gpio_set_level(board::kParaEnable, enabled ? 1 : 0);
}

bool initialized() { return ready.load(std::memory_order_acquire); }

} // namespace actuators::safe_outputs
