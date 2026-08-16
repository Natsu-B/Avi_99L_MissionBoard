#include <cstdio>

#include "actuators/safe_outputs.hpp"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "runtime/runtime.hpp"

extern "C" void app_main() {
  const esp_err_t safe = actuators::safe_outputs::initialize();
  if (safe != ESP_OK) {
    while (true)
      vTaskDelay(pdMS_TO_TICKS(1000));
  }

  std::printf("\n99L Mission Board minimal runtime\n");
  static runtime::Runtime runtime;
  const esp_err_t result = runtime.start();
  std::printf("runtime start: %s\n", esp_err_to_name(result));
  if (result != ESP_OK) {
    (void)actuators::safe_outputs::motorCoast();
    (void)actuators::safe_outputs::setAux5v(false);
    (void)actuators::safe_outputs::setParaPower(false);
    while (true)
      vTaskDelay(pdMS_TO_TICKS(1000));
  }
  vTaskDelete(nullptr);
}
