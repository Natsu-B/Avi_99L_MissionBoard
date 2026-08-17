# RollControl Runtime Implementation

## Status

RollControl本体は実装済み。内部MissionStateは増やさず、`Flight`中のFin出力modeとして動作する。

## Runtime flow

```text
Flight < +8 s
  -> ZeroHold

+8 s gate
  attitude/Fin/LPS/SSC/airspeed unavailable OR ZeroHold未成立
    -> permanent disable
  otherwise
    -> eligible

8 s .. Descent
  valid airspeed <= 60 m/s
    -> permanent disable

  first fully usable tick
    -> roll deviation = 0
    -> capture reference event
    -> RollControl

  later attitude/Fin/LPS/SSC/airspeed input loss
    -> ZeroHold
    -> permanent disable / no re-entry

  gyro integration gap > configured limit
    -> permanent disable
```

## State vector

Controller input is:

1. Control開始時からのunwrapped roll deviation [rad]
2. Fin zeroからのFin angle [rad]
3. roll rate [rad/s]
4. Fin rate [rad/s]

Airspeedでgain scheduleを補間し、requested/effective output shaft torque座標 [N m]を計算する。この座標はactual shaft torqueの実測値ではない。

## Reference

Control用gyro historyは持たない。

最初にRollControlを実際に出力できたtickで`roll_deviation = 0`とし、その時刻以降だけgyroを台形積分する。Control entry後はattitude/Fin/LPS/SSC/airspeedのどれかがinvalid/staleになったtickでpermanent-disable latchを立て、ZeroHold（zero reference無効時はcoast）へ移る。同一flight内でreferenceを再captureしない。

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

RollControlだけがmainの30 kHz torque mapperとdrive/brakeを使用する。RollControllerはGPIO/PWMを直接操作せず、Fin actuatorへrequested torqueだけを渡す。mapperはcurrent、bus voltage、minimum 70 command補償、`±1024 / ±100 %`、gearbox 6000 rpmとmotor hard 9800 rpmの条件を適用する。requested torque、mapper上のeffective torque、計算current、duty、limit状態は内部Fin telemetryで分離する。これらは実測torque/currentではなく、CAN/SDへの外部露出は未実装である。

ZeroHoldは`feat/zero-hold-nm-pid`と同じ20 kHz command-domain PID／drive-coast経路であり、torque mapperを通らない。mode切替時は一度両入力をLOWにしてPWM周波数を切り替える。Roll gainのFIT run FIN0007はdrive/brakeでtopologyは一致するが、FIN0009/FIN0010 validation gateが不成立のため現行gainをflight-qualifiedとは扱わない。

±15 deg境界より外向きのtorqueは0へ抑制し、中心へ戻すtorqueは許可する。9800 rpm以上でも同方向加速torqueだけを0へ抑制し、逆方向brakingは許可する。bus voltage内でcurrent制約を実現できない駆動候補は計算currentをclampして隠さず、`current_limit_unrealizable`を立ててdriveを0へ落とす。minimum 70 command補償がback-EMF下でrequested torqueと逆向きのcurrentを作る場合は補償前dutyへ戻し、最終dutyでもrequested torque方向を実現できなければ`torque_direction_unrealizable`としてdriveを0へ落とす。

## Gain schedule

FIN0007 drive/brake FIT-only effective plantへ全速度共通の
`Q=diag([200,50,0.05,0.5])`、`R=1`を適用して生成したsimulation候補を使用する。
表記丸め前のexact値は`src/config/flight.hpp`をsource of truthとする。

| airspeed [m/s] | K roll angle | K fin angle | K roll rate | K fin rate |
|---:|---:|---:|---:|---:|
| 60 | 14.1421356238 | 17.8483622425 | 2.7408752200 | 0.5733297870 |
| 80 | 14.1421356237 | 21.1222677985 | 2.1934392352 | 0.6100282347 |
| 100 | 14.1421356237 | 24.2907070047 | 1.8456407122 | 0.6444682756 |
| 120 | 14.1421356237 | 27.3634742392 | 1.6035209984 | 0.6769427909 |
| 140 | 14.1421356237 | 30.3625322517 | 1.4233277105 | 0.7078278813 |
| 160 | 14.1421356237 | 33.3812090636 | 1.2764891496 | 0.7381654654 |
| 180 | 14.1421356237 | 36.3688179049 | 1.1577597738 | 0.7675013930 |

60--180 m/sでは線形補間し、有限入力が60 m/s未満ならK60、180 m/s超ならK180へclampする。FIN0009 strict holdoutはtarget-segment angle RMSE `5.530028 deg`で1 deg gateを満たさない。FIN0010確認はacceleration R² `0.430368`、recursive rate fit `56.2689 %`、target RMSE `11.53965 deg`、whole-run drift `+20.26027 deg`であり、これもFAILである。strict holdout/confirmationの結果からgainをretuneせず、ユーザ指定で`PROVISIONAL_BRAKE_FIXED_BY_USER_DIRECTION_VALIDATION_GATE_NOT_MET`として固定する。`NO_GO`、`production_selectable=false`を維持する。

FIN0010 revision artifact SHA-256は
`2eb8ac6f6b94fb99b22417546ef437c557b676c56545e0721e4b15c94dff25b1`、
read-once consumed marker SHA-256は
`4cedffcdb8f389116bfe10c8a6126ca34e64bff2b6cc04898889b79f73fa09c2`である。

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

- FIN0004で顕在化したactuator model errorの解消
- provisional 7点gain scheduleの実機qualification

### Hardware

- gyro roll bias
- SSC zero offset
- Fin motor polarity
- motor resistance / torque constant / speed constant
- drivetrain efficiency
- actual motor current / shaft torque / drivetrain efficiency

これらはruntime command/NVSで変更せず、flight前にsource constantsへ固定する。
