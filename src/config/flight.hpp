#pragma once

#include <cstddef>
#include <cstdint>

namespace flight_config {

constexpr uint64_t kOneSecondUs = 1'000'000ULL;
constexpr uint64_t kControlStartUs = 8ULL * kOneSecondUs;
constexpr uint64_t kPressureApexEnableUs = 10ULL * kOneSecondUs;
constexpr uint64_t kDeploymentFallbackUs = 17ULL * kOneSecondUs;
constexpr uint64_t kAbsolutePowerCutoffUs = 25ULL * kOneSecondUs;

constexpr float kParaOpenDeltaDeg = -130.0F;  // 反時計回り
constexpr float kParaCloseDeltaDeg = 130.0F;  // 時計回り
constexpr float kParaSpeedDegS = 180.0F;
constexpr float kParaAccelerationDegS2 = 360.0F;
constexpr float kParaTorqueLimitPercent = 20.0F;
constexpr float kParaHoldTorquePercent = 20.0F;
constexpr uint32_t kParaPowerStabilizationMs = 100;
constexpr uint32_t kParaMotionTimeoutMs = 2'000;
constexpr uint32_t kParaReconnectMs = 1'000;

// 動翼ZeroHoldの暫定係数。旧MissionBoardで使用していた係数を1モーター用に流用する。
// TODO(HW_TEST): 実機で確認し、必要なら置換する。
constexpr double kFinZeroHoldKpNmPerRad = 2.32;
constexpr double kFinZeroHoldKdNmPerRadS = 0.296;
constexpr double kFinZeroHoldTorqueLimitNm = 0.80;
constexpr double kMotorResistanceOhm = 3.48;
constexpr double kMotorTorqueConstantNmPerA = 0.00855;
constexpr double kMotorSpeedConstantRpmPerV = 1120.0;
constexpr double kTotalGearRatio = 176.175;
constexpr double kDrivetrainEfficiency = 0.60;
constexpr double kMotorBusVoltageV = 9.0;
constexpr double kMotorMaxCurrentA = 2.0;
constexpr double kMotorMaximumDuty = 1.0;
constexpr bool kPositiveTorqueUsesIn1 = true;

constexpr uint32_t kLogRateHz = 1'000;
constexpr uint32_t kLogPeriodUs = 1'000;
constexpr std::size_t kPsramReserveBytes = 512U * 1024U;
constexpr std::size_t kPsramMaxBytes = 8U * 1024U * 1024U;
constexpr std::size_t kSdBatchBytes = 8U * 1024U;

} // namespace flight_config
