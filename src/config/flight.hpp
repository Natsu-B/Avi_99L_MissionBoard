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
constexpr float kParaTorqueLimitPercent = 100.0F;
constexpr float kParaHoldTorquePercent = 100.0F;
constexpr uint32_t kParaPowerStabilizationMs = 100;
constexpr uint32_t kParaMotionTimeoutMs = 2'000;
constexpr uint32_t kParaReconnectMs = 1'000;

// feat/zero-hold-nm-pidで実機成立したcommand-domain PIDをそのまま使う。
constexpr double kFinZeroHoldKpCommandPerDeg = 500.0;
constexpr double kFinZeroHoldKiCommandPerDegS = 35.0;
constexpr double kFinZeroHoldKdCommandPerDegPerS = 25.0;
constexpr double kFinZeroHoldNmPerCommand = 0.0022825744628906255;
constexpr double kFinZeroHoldIntegralLimitDegS = 2.0;
constexpr double kFinZeroHoldRateFilterTauS = 0.020;
constexpr double kFinZeroHoldAngleDeadbandDeg = 0.05;
constexpr double kFinZeroHoldRateDeadbandDegS = 0.5;
constexpr double kFinZeroHoldMinimumActiveErrorDeg = 0.08;
constexpr double kFinZeroHoldIntegralDecay = 0.95;
constexpr double kFinZeroHoldIntegralZeroThresholdDegS = 0.001;
constexpr double kFinZeroHoldMaximumDtS = 0.005;
constexpr int16_t kFinZeroHoldControlCommandLimit = 800;
constexpr int16_t kFinZeroHoldMinimumCommand = 70;
constexpr double kFinZeroHoldAchievedAngleDeg = 1.0;
constexpr double kFinZeroHoldAchievedRateDegS = 2.0;
constexpr uint64_t kFinZeroHoldAchievedDurationUs = 200'000ULL;
// FIN0006/FIN0007 FIT-derived nonlinear plantでは50/50 caseが成立した。
// ただしactual current/torqueが未計測のため飛行選定済みとはしない。
constexpr bool kFinZeroHoldSimulationGatePassed = true;
constexpr bool kFinZeroHoldProductionSelectable = false;
constexpr double kMotorResistanceOhm = 3.48;
constexpr double kMotorTorqueConstantNmPerA = 0.00855;
constexpr double kMotorSpeedConstantRpmPerV = 1120.0;
constexpr double kTotalGearRatio = 176.175;
constexpr double kDrivetrainEfficiency = 0.60;
constexpr double kMotorBusVoltageV = 9.0;
// TB67の実装上のhardware current settingに合わせる。
constexpr double kMotorMaxCurrentA = 2.2;
constexpr double kMotorHardSpeedRpm = 9'800.0;
constexpr double kGearboxContinuousSpeedRpm = 6'000.0;
constexpr double kMotorMaximumDuty = 1.0;
constexpr uint16_t kMotorCommandFullScale = 1024;
constexpr uint16_t kMotorMinimumActiveCommand = 70;
constexpr bool kPositiveTorqueUsesIn1 = true;
constexpr double kFinOutwardCommandLimitDeg = 15.0;

// Control用の個体固定値。runtime calibration/NVSでは変更しない。
// TODO(HW_TEST): 飛行個体のcharacterization値へ置換する。
constexpr double kGyroRollBiasDps = 0.0;
// 2026-08-17の飛行個体実測値。+5V投入後5秒待機し、
// 500 sample x 3 runのtrimmed mean平均から固定する。
constexpr double kSscZeroOffsetPa = 86.877;
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

// FIN0007 drive/brake FIT-only effective plantから
// Q=diag([200,50,0.05,0.5]), R=1の共通policyで生成した7点候補。
// FIN0009 strict holdoutはrecursive rateを再現したがtarget segment angle
// RMSE=5.530028 degが事前固定1 deg gateを満たさない。FIN0010の
// 追加確認もRMSE=11.53965 deg、drift=+20.26027 degでFAIL。ユーザ指定で
// Brake候補を固定するが、飛行選定値ではない。
inline constexpr char kRollGainCandidateTopology[] = "drive_brake";
inline constexpr char kRollGainFitRun[] = "FIN0007";
inline constexpr char kRollGainStrictHoldoutRun[] = "FIN0009";
inline constexpr char kRollGainSelectionStatus[] =
    "PROVISIONAL_BRAKE_FIXED_BY_USER_DIRECTION_VALIDATION_GATE_NOT_MET";
inline constexpr char kRollGainSourceArtifactSha256[] =
    "2eb8ac6f6b94fb99b22417546ef437c557b676c56545e0721e4b15c94dff25b1";
inline constexpr char kRollGainSourceConsumedMarkerSha256[] =
    "4cedffcdb8f389116bfe10c8a6126ca34e64bff2b6cc04898889b79f73fa09c2";
inline constexpr char kRollGainConfirmationRun[] = "FIN0010";
inline constexpr char kRollGainConfirmationCsvSha256[] =
    "cc8ac1b0bd3f7cf2af08074d313e4f194e3daacc84d9254f888c5a3342df0254";
inline constexpr char kRollGainConfirmationMetadataSha256[] =
    "7d7c41622715cc91fe334aa674fc809169731fd719521320f4d6030d1c7db177";
constexpr bool kRollGainStrictHoldoutPassed = false;
constexpr bool kRollGainConfirmationPassed = false;
// 以下はvalidation metadataであり、runtimeのControl gateではない。
constexpr bool kRollGainValidationNoGo = true;
constexpr bool kRollGainProductionSelectable = false;
inline constexpr std::array<control::GainPoint, 7> kRollGainSchedule{{
    {60.0,
     {14.142135623750997, 17.848362242491291, 2.7408752200006115,
      0.5733297870290188}},
    {80.0,
     {14.142135623728176, 21.122267798503998, 2.1934392351804677,
      0.6100282346806772}},
    {100.0,
     {14.142135623726388, 24.290707004734234, 1.8456407122409573,
      0.6444682755503087}},
    {120.0,
     {14.142135623726164, 27.363474239193827, 1.6035209983579835,
      0.6769427908755344}},
    {140.0,
     {14.14213562373258, 30.362532251737875, 1.423327710512041,
      0.7078278812966763}},
    {160.0,
     {14.142135623732253, 33.3812090635748, 1.276489149551201,
      0.7381654653819836}},
    {180.0,
     {14.142135623730802, 36.368817904885404, 1.1577597738460124,
      0.7675013930465681}},
}};
constexpr uint32_t kLogRateHz = 1'000;
constexpr uint32_t kLogPeriodUs = 1'000;
constexpr std::size_t kPsramReserveBytes = 512U * 1024U;
constexpr std::size_t kPsramMaxBytes = 8U * 1024U * 1024U;
constexpr std::size_t kSdBatchBytes = 8U * 1024U;

} // namespace flight_config
