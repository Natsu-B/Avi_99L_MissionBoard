# Flight Sequence

## 1. 全体

```text
boot
  |
  v
CommandReceive
  |
  | StartSequence
  v
LiftoffDetection
  |
  | liftoff detected
  v
Flight
  |\
  | \ pressure apex after +10 s
  |  \
  |   +17 s timer fallback
  v
Descent
  |
  +25 s absolute actuator power cutoff
```

## 2. 離床検知

LiftoffDetectionではICM42688とLPS25HBを独立に評価する。

### 2.1 ICM42688

- 1 kHz評価
- 各軸直近20 sampleの算術平均
- 平均加速度vectorのnormが2 Gを超える状態を50 ms連続

数学的には:

```text
mean_ax^2 + mean_ay^2 + mean_az^2 > 4.0 [G^2]
```

を50回連続で満たす。

invalid sampleでは連続counterをresetする。

### 2.2 LPS25HB

- 25 Hz
- 5 sample算術平均
- 前回平均より0.05 hPa以上低下した状態を5回連続

ICMまたはLPSのどちらか一方で離床成立すればよい。

一方のsensor故障を理由に他方の離床判定を止めない。

## 3. 離床時刻

検知には遅延があるため、Mission上の離床時刻は検知時刻の1秒前とする。

```text
liftoff_us = detected_us - 1 s
```

検知時刻が1秒未満の場合だけ0へclampする。

この`liftoff_us`を+8/+10/+17/+25秒の共通基準とする。

## 4. LiftoffDetectionEmergencyStop

Flight中、Descent commit前まで使用可能。

Emergency受理後はLiftoffDetectionへ戻り、以前のliftoff timerを無効化する。

再離床検知後は新しいgenerationと新しいliftoff時刻を使用する。

旧generationのtimer/eventは新flightへ適用しない。

## 5. +8秒 Control判定

Control追加後、離床+8秒をControl使用資格の最初の判定時刻とする。

+8秒時点で永久にControl不能とする条件は次の2つ。

1. ICM42688から有効なroll軸角速度を取得できず、Control用gyro積分を開始できない。
2. CommandReceiveで設定したFin zeroが失われており、論理Fin 0度が分からない。

LPS/SSC/airspeedが+8秒時点で一時的に利用不能であることだけではControlを永久禁止しない。

詳細は`04_control_spec.md`を参照する。

## 6. pressure apex

離床+10秒以降、LPS25HBのpressure apex detectorを有効にする。

- 5 sample平均
- 平均圧力が前回より上昇する状態を1秒連続

成立した場合はDescentへcommitする。

LPSが利用不能な場合はpressure apexを成立させず、+17秒fallbackへ任せる。

## 7. +17秒 timer fallback

離床+17秒で、他の条件に関係なくDescentへcommitする。

このfallbackは以下へ依存させない。

- CAN
- LoRa
- 通信基板
- GUI
- LPS
- SSC
- SD
- Control
- Fin actuator

Missionのmonotonic timerとliftoff時刻が生きている限り進行することを要求する。

## 8. Descent

Descentは不可逆state。

entry時:

1. `deployment_started=true`
2. Para Openを要求
3. Finは可能なら0度保持、不能ならsafe

Para Open完了をMissionState遷移の前提にしない。

## 9. +25秒 absolute cutoff

離床+25秒で以下を無条件に実行する。

- Para power OFF
- `+5V_short` / auxiliary 5 V OFF
- Fin motor Hi-Z
- `power_cutoff=true`

Open成功、Open失敗、servo moving、Control状態、SD状態に関係なく実行する。

このcutoffはSafety経路が所有し、blocking UART/SD/CAN operationを待たない。

## 10. 開傘時刻の変更

現基準は+17秒fallbackとする。

15秒等へ変更する場合は単なる定数調整として扱わず、Vault、simulation、審査書との整合確認を伴う仕様変更として実施する。
