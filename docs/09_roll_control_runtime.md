# RollControl Runtime Implementation

## Status

RollControl本体は実装済み。内部MissionStateは増やさず、`Flight`中のFin出力modeとして動作する。

## Runtime flow

```text
Flight < +8 s
  -> ZeroHold

+8 s gate
  ICM roll rate unavailable OR Fin zero invalid
    -> permanent disable
  otherwise
    -> eligible

8 s .. Descent
  valid airspeed <= 60 m/s
    -> permanent disable

  required inputs unavailable
    -> ZeroHold

  first fully usable tick
    -> roll deviation = 0
    -> capture reference event
    -> RollControl

  later temporary LPS/SSC/airspeed/Fin input loss
    -> ZeroHold
    -> recover to same reference when inputs return

  gyro integration gap > configured limit
    -> permanent disable
```

## State vector

Controller input is:

1. Control開始時からのunwrapped roll deviation [rad]
2. Fin zeroからのFin angle [rad]
3. roll rate [rad/s]
4. Fin rate [rad/s]

Airspeedでgain scheduleを補間し、output torque [N m]を計算する。

## Reference

Control用gyro historyは持たない。

最初にRollControlを実際に出力できたtickで`roll_deviation = 0`とし、その時刻以降だけgyroを台形積分する。ControlがSSC/LPS欠落でZeroHoldへ落ちてもgyroが有効なら積分は継続し、復帰時にreferenceを再captureしない。

## AirData

- LPS25HB: 25 Hz
- SSC: AirData taskで反復取得
- SSC zero: compile-time `kSscZeroOffsetPa`
- moving average: `kDifferentialPressureMovingAverageSamples`
- Pitot pressure correction coefficient: `kPitotPressureCorrectionCoefficient`
- airspeed: Saint-Venant式

`airspeed unavailable`と`valid airspeed <= 60 m/s`を区別する。

## Fin actuator

Fin actuator modeは以下。

- `free`
- `zero_hold`
- `roll_control`

ZeroHoldとRollControlは同じAS5047D sample、Fin rate、motor electric model、PWM driverを共有する。RollControllerはGPIO/PWMを直接操作せず、Fin actuatorへoutput torqueだけを渡す。

±15 deg境界より外向きのtorqueは0へ抑制し、中心へ戻すtorqueは許可する。

## CAN compatibility

- FlightでControl inactive: `EngineBurn(2)`
- FlightでControl active: `Control(3)`
- 0x100: control-relative roll deviation / roll rate / Fin angle / Fin rate
- 0x101: requested Control torque / flight elapsed
- 0x109: airspeed
- 0x10A: schema v2、reference=0、roll deviation、Control active、reference capture event

内部state machineには`Control` stateを追加しない。

## Constants still requiring qualification

### Simulation

- 60..180 m/s gain schedule
- RollControl torque authority

### Hardware

- gyro roll bias
- SSC zero offset
- Fin motor polarity
- motor resistance / torque constant / speed constant
- drivetrain efficiency
- ZeroHold Kp/Kd/torque limit

これらはruntime command/NVSで変更せず、flight前にsource constantsへ固定する。
