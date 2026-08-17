#pragma once

#include <cstdint>

#include "driver/gpio.h"
#include "driver/i2c_types.h"
#include "driver/spi_master.h"
#include "driver/uart.h"

namespace board {

constexpr spi_host_device_t kEncoderSpiHost = SPI2_HOST;
constexpr gpio_num_t kEncoderMosi = GPIO_NUM_4;
constexpr gpio_num_t kEncoderMiso = GPIO_NUM_5;
constexpr gpio_num_t kEncoderSclk = GPIO_NUM_6;
constexpr gpio_num_t kEncoderCs = GPIO_NUM_7;
constexpr uint32_t kEncoderSpiFrequencyHz = 8'000'000;

constexpr spi_host_device_t kImuSpiHost = SPI3_HOST;
constexpr gpio_num_t kImuCs = GPIO_NUM_8;
constexpr gpio_num_t kImuInterrupt = GPIO_NUM_15;
constexpr gpio_num_t kImuMiso = GPIO_NUM_16;
constexpr gpio_num_t kImuMosi = GPIO_NUM_17;
constexpr gpio_num_t kImuSclk = GPIO_NUM_18;
constexpr uint32_t kImuSpiFrequencyHz = 8'000'000;

constexpr gpio_num_t kSdDat1 = GPIO_NUM_3;
constexpr gpio_num_t kSdDat0 = GPIO_NUM_9;
constexpr gpio_num_t kSdClk = GPIO_NUM_10;
constexpr gpio_num_t kSdCmd = GPIO_NUM_11;
constexpr gpio_num_t kSdDat3 = GPIO_NUM_12;
constexpr gpio_num_t kSdDat2 = GPIO_NUM_13;

constexpr gpio_num_t kCanRx = GPIO_NUM_14;
constexpr gpio_num_t kCanTx = GPIO_NUM_21;
constexpr uint32_t kCanBitrate = 125'000;

constexpr i2c_port_t kAirDataI2cPort = I2C_NUM_0;
constexpr gpio_num_t kAirDataSda = GPIO_NUM_47;
constexpr gpio_num_t kAirDataScl = GPIO_NUM_48;
constexpr uint32_t kAirDataI2cFrequencyHz = 300'000;

constexpr gpio_num_t kMotorIn2 = GPIO_NUM_38;
constexpr gpio_num_t kMotorIn1 = GPIO_NUM_39;
// FIN0003/FIN0004実機characterizationと同じ30 kHzに固定する。
constexpr uint32_t kMotorPwmFrequencyHz = 30'000;

constexpr gpio_num_t kAux5vEnable = GPIO_NUM_40;
constexpr gpio_num_t kParaRx = GPIO_NUM_41;
constexpr gpio_num_t kParaTx = GPIO_NUM_42;
constexpr gpio_num_t kParaEnable = GPIO_NUM_44;
constexpr uart_port_t kParaUart = UART_NUM_1;
constexpr uint32_t kParaBaudrate = 1'000'000;
constexpr uint8_t kParaServoId = 1;

constexpr gpio_num_t kLed1 = GPIO_NUM_45;
constexpr gpio_num_t kLed2 = GPIO_NUM_46;

} // namespace board
