# Commands and Actuators

## 1. CommandReceiveで許可する操作

公開する通常commandは最小限とする。

| Command | Code | 動作 |
|---|---:|---|
| StartSequence | `0x01` | LiftoffDetectionへ遷移 |
| FinFree | `0x10` | Fin motorをHi-Z、zero invalid |
| FinHoldCurrent | `0x13` | 現在のFin位置を0度としてZeroHold |
| ParaOpen | `0x25` | 反時計回り130度の相対move後Hold |
| ParaClose | `0x26` | 時計回り130度の相対move後Hold |

その他の旧commandは新runtimeへ持ち込まない。

特に以下は廃止する。

- ForceStartSequence
- SetFinZero
- FinMoveRelative
- ParaFree
- ParaHold command
- ParaMoveRelative
- SetParaOpen / SetParaClose
- RunPreflightCalibration
- ActuatorEmergencyStop
- NVS設定command

`LiftoffDetectionEmergencyStop`だけは専用CAN ID `0x002`として維持する。

## 2. Command replay

相対moveを二重実行しないため、transaction IDとcommandの結果をcacheする。

同じtransaction ID・同じcommandが再送された場合は、既に完了した結果を返し、actuator side effectを再実行しない。

同じtransaction IDを異なるcommandへ使い回すことはprotocol errorとして扱う。

## 3. Fin

### 3.1 Hardware

- encoder: AS5047D
- motor: 1系統のみ
- motor driver入力: GPIO39 / GPIO38
- PWM: 30 kHz

旧2-motor profileやSpareMotorBは使用しない。

### 3.2 FinHoldCurrent

CommandReceiveで`FinHoldCurrent`を受理したとき:

1. AS5047Dの現在角を取得する。
2. その角度を`zero_rad`としてRAMへ保存する。
3. 以後の論理Fin角を`current - zero_rad`として扱う。
4. ZeroHoldを開始する。
5. `zero_valid=true`とする。

NVSへzeroを保存しない。

### 3.3 FinFree

`FinFree`受理時:

- motor出力をHi-Zにする。
- `zero_valid=false`とする。

Free中にFinを物理的に動かせるため、以前のzero referenceを保持しない。

### 3.4 ZeroHold

ZeroHoldは論理Fin角0度を保持する。

現実装の暫定controller:

```text
torque = -Kp * fin_angle - Kd * fin_rate
```

暫定値:

- `Kp = 2.32 N m/rad`
- `Kd = 0.296 N m/(rad/s)`
- torque limit = `0.80 N m`

これらは確定飛行値ではなく`TODO(HW_TEST)`とする。

encoderを利用できず現在角が分からない場合はZeroHoldを継続しようとせずmotorをsafe/Hi-Zへ落とす。

## 4. Parachute / STS3215

### 4.1 基本方針

パラシュートはabsolute Open/Close endpointを保存しない。

- Open = 現在位置から反時計回り130度
- Close = 現在位置から時計回り130度

現符号定義:

```text
Open  = -130 deg
Close = +130 deg
```

`Direction::normal`前提であり、物理機構で方向を実機確認する。

### 4.2 boot後

Para電源投入後、STS3215へ接続できた場合は現在位置Holdを成立させる。

通常flight pathではFree/disableTorqueを使用せず、+25秒cutoffまでHold/Torque ONを基本とする。

### 4.3 Open / Close

command開始時:

1. servo transportを確立する。
2. 現在位置Holdを成立させる。
3. step/relative modeで固定130度moveを1回だけ発行する。
4. `moving=false`をbounded pollingする。
5. 完了後、到達した現在位置をHoldする。

暫定parameter:

- speed = 180 deg/s
- acceleration = 360 deg/s^2
- torque limit = 20 %
- hold torque = 20 %
- motion timeout = 2 s

### 4.4 曖昧な結果の扱い

130度相対moveを送信したか不明なtransport failureでは、同じmoveを自動再送しない。

理由は、実際にはservoがmoveを受理済みだった場合に再送すると260度動くためである。

自動deploymentでは、まだrelative moveを一度も発行していないことが確実な間だけSTS接続を再試行してよい。

### 4.5 power cutoff

離床+25秒でPara電源を物理OFFする。

cutoff後に通常taskが再度Para電源をONにしてはならない。

## 5. Descent entry

Descent commit後:

- Fin zeroがvalidかつencoder利用可能ならZeroHoldを要求する。
- Finを利用できない場合はmotor safe。
- Para Openを直ちに要求する。

Finが0度へ到達することをPara Openの前提条件にしない。
