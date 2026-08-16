# Roll Control Specification

## 1. 目的

Controlの目的は、**実際にRollControlを開始した瞬間の機体ロール角を、その後できるだけ一定に保つこと**である。

離床時roll=0を保持目標にはしない。

## 2. MissionStateとの関係

ControlはMissionStateではない。

内部phaseが`Flight`である間、そのtickのControl条件に応じてFin出力modeを決める。

```text
Flight
  |
  +-- control条件成立 --> RollControl
  |
  +-- control条件不成立 --> ZeroHold
```

RollControlとZeroHoldの切替でMission phaseを変更しない。

## 3. +8秒のControl資格判定

離床+8秒で、Control系そのものが成立可能か一度判定する。

### 永久にControlを禁止する条件

- ICM42688から有効なroll rateを取得できず、Control用積分を開始できない。
- Fin zeroがinvalidで、論理Fin 0度が不明。

どちらか成立した場合:

```text
control_permanently_disabled = true
```

として、そのflightではDescentまでZeroHold/safeを継続する。

### +8秒時点では永久停止にしない条件

以下は一時的availabilityとして扱う。

- LPS unavailable
- SSC unavailable
- airspeed unavailable
- 一時的なControl input freshness不足

これらだけを理由に、そのflight全体のControl資格を失わせない。

## 4. Control開始時roll reference

最初に実際のRollControl条件がすべて成立したtickをControl開始時刻とする。

その瞬間:

```text
roll_deviation = 0
control_reference_started = true
```

とする。

制御用に離床時からのabsolute roll履歴を保持する必要はない。

Control開始後はroll rateを積分して、Control開始時からのroll偏差を連続値として更新する。

```text
roll_deviation(t) = integral(roll_rate dt) from control start
```

360度を超える回転もwrapせず、unwrappedな偏差として扱う。

## 5. gyro history

Control目的だけであれば、CommandReceiveや離床1秒前からのgyro historyは不要とする。

Control開始時点をroll偏差0として積分開始する。

将来GUI用3D姿勢表示などで離床前姿勢履歴が必要になった場合は、Control estimatorとは別moduleとして実装する。

表示用姿勢推定の故障をControl/Paraへ波及させない。

## 6. 8秒からDescentまでの出力mode

Control資格がある場合でも、毎tick必要入力を確認する。

```text
if control_permanently_disabled:
    ZeroHold
else if required control inputs are usable:
    RollControl
else:
    ZeroHold
```

一時的に必要sensorが欠けた場合はZeroHoldへ切り替える。

sensorが復旧し、roll estimatorの連続性が維持されている場合はRollControlへ戻ってよい。

この復帰をMissionState再entryとして扱わない。

## 7. required control inputs

RollControlを実行するtickでは最低限、以下がvalid/freshであること。

- ICM42688 roll rate
- Control開始後のroll deviation estimator
- AS5047D Fin angle
- Fin rate
- LPS static pressure
- SSC differential pressure
- 計算済みairspeed
- controller計算に使用する全数値がfinite

Control追加時に具体的freshness閾値をconfigとして実装する。

## 8. 一時的sensor欠落

LPS/SSC/airspeed等が一時的に利用不能の場合:

```text
RollControl -> ZeroHold
```

とする。

利用可能に戻った場合、Control estimatorがvalidなら:

```text
ZeroHold -> RollControl
```

としてよい。

Control referenceを再captureしてはならない。

## 9. gyro積分の連続性喪失

単発sample欠落など、補間またはtimestamp上の連続性を保証できる範囲は実装上許容してよい。

一方、長時間欠落、timestamp不整合、FIFO faultなどでroll偏差積分の連続性を保証できなくなった場合:

```text
control_permanently_disabled = true
```

とし、そのflightではRollControlへ戻らない。

適当な角度を再構成して再開しない。

## 10. 60 m/s停止条件

validなairspeedが一度でも60 m/s以下になった場合、そのflightではRollControlを永久停止する。

```text
if airspeed_valid && airspeed_mps <= 60.0:
    control_permanently_disabled = true
```

重要:

- `airspeed unavailable` は永久停止ではない。
- `valid airspeed <= 60 m/s` は永久停止。

停止後はDescentまでZeroHold/safe。

この条件は通信やLPS apexとは独立したControl safety conditionである。

## 11. Calibration

runtime PreflightCalibrationは実装しない。

Control追加時に必要なgyro bias、SSC zeroなどはcompile-time定数として管理する。

値は実機characterizationで決定し、NVSから飛行前に変更する仕組みを設けない。

## 12. controller parameter

現時点でRollControl gain schedule、motor model、Control authorityは最終確定していない。

これらはsimulation/HW test後に`src/config/`へ固定値として追加する。

未確定値をreadiness/NVSでoperatorが現場変更する方式には戻さない。
