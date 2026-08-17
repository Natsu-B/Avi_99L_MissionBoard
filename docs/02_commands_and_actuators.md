# Commands and Actuators

## 1. CommandReceiveで許可する操作

公開する通常commandは最小限とする。

| Command | Code | 動作 |
|---|---:|---|
| StartSequence | `0x01` | LiftoffDetectionへ遷移 |
| FinFree | `0x10` | Fin motorをHi-Z。AS5047D unwrapとFin zeroは保持 |
| FinZero | `0x11` | 現在のmulti-turn encoder位置を論理Fin 0度としてRAMへcapture |
| FinHold | `0x13` | capture済みの論理Fin 0度をZeroHold |
| ParaOpen | `0x25` | 反時計回り130度の相対move後Hold |
| ParaClose | `0x26` | 時計回り130度の相対move後Hold |

その他の旧commandは新runtimeへ持ち込まない。

特に以下は廃止する。

- ForceStartSequence
- FinMoveRelative
- StartFinZeroHold
- ParaFree
- ParaHold command
- ParaMoveRelative
- SetParaOpen / SetParaClose
- RunPreflightCalibration
- ActuatorEmergencyStop
- NVS設定command

`LiftoffDetectionEmergencyStop`だけは専用CAN ID `0x002`として維持する。

## 2. Command replay

同じtransaction ID・同じcommandが再送された場合は、既に完了した結果を返し、actuator side effectを再実行しない。

同じtransaction IDを異なるcommandへ使い回すことはprotocol errorとして扱う。

## 3. Fin

### 3.1 Hardware / coordinate

- encoder: AS5047D
- motor: 1系統のみ
- motor driver入力: GPIO39 / GPIO38
- PWM: ZeroHold 20 kHz / RollControl 30 kHz
- total gear ratio: `176.175`

AS5047Dは1回転内の絶対角しか返さない。一方、Finの機構ではgear ratioによりAS5047D側が複数回転するため、単純な`current - zero`のwrap値をFin角として扱ってはならない。

CommandReceiveから飛行終了まで、validなAS5047D sampleごとに前sampleとの差を`[-pi,+pi]`へwrapし、その差分をRAM上の`encoder_unwrapped_rad`へ積算する。

```text
encoder_delta =
    remainder(raw_current - raw_previous, 2*pi)

encoder_unwrapped += encoder_delta

fin_angle =
    (encoder_unwrapped - zero_encoder_unwrapped)
    / total_gear_ratio
```

`remainder`を使用するのは隣接sample間のencoder差分だけであり、Zeroからの累積角そのものを1回転へwrapしてはならない。

隣接valid sample間でAS5047D実回転が180 deg未満であることをunwrapの前提条件とする。productionではAS5047Dを継続取得し、Free中もproducerを止めない。

### 3.2 FinZero

CommandReceiveで`FinZero`を受理したとき:

1. AS5047D trackingが開始済みかつ最新sampleがvalidであることを確認する。
2. 現在の`encoder_unwrapped_rad`を`zero_encoder_unwrapped_rad`としてRAMへ保存する。
3. 論理Fin角を0 degとする。
4. `zero_reference_valid=true`とする。
5. `zero_hold_achieved=false`へ戻す。
6. motor modeは変更しない。

FinZeroは駆動commandではない。Free中ならFreeのまま、ZeroHold中なら新しい0 degをその場で保持する。

### 3.3 FinFree

`FinFree`受理時:

- motor出力をHi-Zにする。
- ZeroHold / RollControl torque requestを解除する。
- `zero_reference_valid`を維持する。
- `zero_hold_achieved`は解除する。
- AS5047D samplingとmulti-turn unwrapを継続する。

これにより、審査時にFinZero/FinHoldを成立させた後、待機中だけFinFreeとして消費電力を下げ、打上げ前に同じ0 degへFinHoldを戻せる。

### 3.4 FinHold

`FinHold`は新しいZeroをcaptureしない。

- `zero_reference_valid=true`
- motor driver利用可能

を満たす場合のみ、既存の論理Fin 0 degをZeroHoldする。

FinFree中にFinを手で動かした場合も、AS5047D unwrapが連続していれば審査時にcaptureした0 degへ戻る。

### 3.5 ZeroHold

ZeroHoldはmotor-side/output-equivalentの論理Fin角0度を保持する。physical
left/right finの厳密な空力0度を保証するfieldではない。

現実装のcontroller:

```text
error = -fin_angle
command = Kp * error + Ki * integral(error) - Kd * filtered_fin_rate
```

値:

- `Kp = 500 command/deg`
- `Ki = 35 command/(deg s)`
- `Kd = 25 command/(deg/s)`
- integral limit `±2 deg s`
- velocity LPF `tau = 20 ms`
- hold deadband `0.05 deg / 0.5 deg/s`
- minimum active error `0.08 deg`

AS5047Dのencoder軸角・encoder軸角速度は`total gear ratio`で割ってFin出力軸角・角速度へ変換した後でcontrollerへ渡す。

commandは`±800`へ制限し、minimum active error以上で絶対値70未満なら70 commandへ補償する。20 kHz、10 bitのdrive/coastで直接駆動し、±15 deg境界では外向きcommandだけを0にする。ZeroHoldはRollControl用torque mapperを使用しない。

`zero_hold_achieved`は`|angle| <= 1 deg`かつ`|rate| <= 2 deg/s`が200 ms連続した場合だけ成立する。valid sample gapはFin freshness上限`5 ms`以下でなければならず、timestampが0、非単調または過大gapなら200 ms判定を最初からやり直す。Control gateは`zero_reference_valid`でなくこのfieldを使用する。

encoderを利用できず現在角が分からない間はmotorをsafe/Hi-Zへ落とす。sample復帰後もmulti-turn unwrapは最後のvalid sampleから継続するため、飛行前試験で想定最大sample gapに対して周回誤認が起きないことを確認する。

### 3.6 RAM-only Zero / reboot

Fin zeroをNVSへ保存しない。

理由はAS5047Dから再起動後に取得できるのが1回転内角度だけであり、gear ratioによりencoder側が複数回転するため、再起動前の`encoder_unwrapped_rad`の周回数を復元できないためである。

reboot後は`zero_reference_valid=false`、`zero_hold_achieved=false`から開始し、CommandReceiveで物理Finを基準位置へ合わせて`FinZero`を再実行する。

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
