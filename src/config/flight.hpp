#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "control/roll_control.hpp"

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

// moving_fin_system_identificationで実機成立したZero Holdを、
// command-domainのまま本番経路で再現する。単位はTB67H450FNGへ与える±1024 command。
constexpr double kFinZeroHoldKpCommandPerDeg = 500.0;
constexpr double kFinZeroHoldKiCommandPerDegS = 35.0;
constexpr double kFinZeroHoldKdCommandPerDegPerS = 25.0;
constexpr double kFinZeroHoldIntegralLimitDegS = 2.0;
constexpr double kFinZeroHoldVelocityFilterTauS = 0.020;
constexpr double kFinZeroHoldDeadbandDeg = 0.050;
constexpr double kFinZeroHoldDeadbandRateDegS = 0.5;
constexpr double kFinZeroHoldMinimumActiveErrorDeg = 0.080;
constexpr int16_t kFinZeroHoldMinimumCommand = 70;
constexpr int16_t kFinZeroHoldControlCommandLimit = 800;
constexpr double kMotorCommandFullScale = 1024.0;

// 旧Spica PD/host-testとRoll Control側のNm換算互換用。
// Zero Hold本番経路ではこのNm-equivalent変換を使用しない。
constexpr double kFinZeroHoldNmPerCommand = 0.0022825744628906255;
constexpr double kFinZeroHoldKpNmPerRad = 65.390941574;
constexpr double kFinZeroHoldKiNmPerRadS = 4.577365910;
constexpr double kFinZeroHoldKdNmPerRadS = 3.269547079;
constexpr double kFinZeroHoldCharacterizationCommandLimit = 600.0;
constexpr double kFinZeroHoldTorqueLimitNm =
    kFinZeroHoldCharacterizationCommandLimit * kFinZeroHoldNmPerCommand;
constexpr double kFinZeroHoldRateDeadZoneDegS = 1.0;

constexpr double kMotorResistanceOhm = 3.48;
constexpr double kMotorTorqueConstantNmPerA = 0.00855;
constexpr double kMotorSpeedConstantRpmPerV = 1120.0;
constexpr double kTotalGearRatio = 176.175;
constexpr double kDrivetrainEfficiency = 0.60;
constexpr double kMotorBusVoltageV = 9.0;
// TB67の実装上のhardware current settingに合わせる。
constexpr double kMotorMaxCurrentA = 2.2;
constexpr double kMotorMaximumDuty = 1.0;
constexpr bool kPositiveTorqueUsesIn1 = true;
constexpr double kFinOutwardCommandLimitDeg = 15.0;

// Control用の個体固定値。runtime calibration/NVSでは変更しない。
// TODO(HW_TEST): 飛行個体のcharacterization値へ置換する。
constexpr double kGyroRollBiasDps = 0.0;
constexpr double kSscZeroOffsetPa = 0.0;
constexpr double kDifferentialPressureNegativeTolerancePa = 5.0;
constexpr std::size_t kDifferentialPressureMovingAverageSamples = 8;
constexpr double kPitotPressureCorrectionCoefficient = 0.92;

constexpr uint64_t kImuFreshUs = 5'000ULL;
constexpr uint64_t kFinFreshUs = 5'000ULL;
constexpr uint64_t kLpsFreshUs = 120'000ULL;
constexpr uint64_t kSscFreshUs = 15'000ULL;
constexpr uint64_t kAirspeedFreshUs = 15'000ULL;
constexpr uint64_t kGyroIntegrationMaximumGapUs = 5'000ULL;
constexpr double kAirspeedPermanentStopMps = 60.0;

// TODO(SIMULATION): Spicaで確定したgain tableへ置換する。
// 現値は旧MissionBoardの暫定候補で、flight qualification済みではない。
inline constexpr std::array<control::GainPoint, 7> kRollGainSchedule{{
    {60.0, {0.08, 2.32, 0.04, 0.296}},
    {80.0, {0.08, 2.32, 0.04, 0.296}},
    {100.0, {0.08, 2.32, 0.04, 0.296}},
    {120.0, {0.08, 2.32, 0.04, 0.296}},
    {140.0, {0.08, 2.32, 0.04, 0.296}},
    {160.0, {0.08, 2.32, 0.04, 0.296}},
    {180.0, {0.08, 2.32, 0.04, 0.296}},
}};
// TODO(SIMULATION/HW_TEST): 最終Control authorityへ置換する。
constexpr double kRollControlTorqueLimitNm = 1.21208;

constexpr uint32_t kLogRateHz = 1'000;
constexpr uint32_t kLogPeriodUs = 1'000;
constexpr std::size_t kPsramReserveBytes = 512U * 1024U;
constexpr std::size_t kPsramMaxBytes = 8U * 1024U * 1024U;
constexpr std::size_t kSdBatchBytes = 8U * 1024U;

} // namespace flight_config
