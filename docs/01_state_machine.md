# Mission State Machine

## 1. 内部phase

```text
CommandReceive
    |
    | StartSequence
    v
LiftoffDetection
    |
    | liftoff detected
    v
Flight
    |
    | deployment condition
    v
Descent
```

この4状態以外を内部Mission phaseとして追加しない。

## 2. CommandReceive

boot後の初期phase。

受理する通常command:

- `StartSequence (0x01)`
- `FinFree (0x10)`
- `FinZero (0x11)`
- `FinHold (0x13)`
- `ParaOpen (0x25)`
- `ParaClose (0x26)`

Fin操作の意味は以下とする。

- `FinFree`: motorをHi-Zにする。Fin zeroとAS5047D multi-turn trackingは維持する。
- `FinZero`: 現在の連続AS5047D位置を論理Fin 0度としてRAMへcaptureする。motor modeは変更しない。
- `FinHold`: capture済みの論理Fin 0度を保持する。現在位置を新しい0度にはしない。

`StartSequence`はPreflight readinessやCalibration resultを確認せず、phaseがCommandReceiveであればLiftoffDetectionへ遷移する。

Startを拒否するための7 bit / 5 bit readiness mask、ForceStart、Calibration gateは持たない。

## 3. LiftoffDetection

StartSequence受理後の離床待機phase。

このphaseでは:

- ICM42688による離床検知
- LPS25HBによる離床検知
- Fin zeroがvalidならZeroHold
- ParaはHold

を継続する。

AS5047D trackingはCommandReceiveから停止せず、Freeであっても1回転角のunwrapを継続する。

離床検知が成立した時点で、検知時刻の1秒前を`liftoff_us`とする。

```text
liftoff_us = max(detected_us - 1 s, 0)
```

離床確定ごとに`generation`を更新する。

## 4. Flight

離床後からDescent commit前までのphase。

内部ではEngineBurn/Controlを分離しない。

Flight中の主な時間条件:

- +8 s: Control資格判定
- +10 s: pressure apex判定を有効化
- +17 s: 無条件timer fallback deployment
- +25 s: absolute actuator power cutoff

Flight中のRollControl有無はMission phaseではなく出力modeとして扱う。

## 5. LiftoffDetectionEmergencyStop

CAN ID `0x002` を使用する。

受理可能条件:

- 現在phaseが`Flight`
- まだ`Descent`へcommitしていない

受理時:

- `Flight -> LiftoffDetection`
- generationを更新
- liftoff時刻を無効化
- deployment関連stateをclear
- Control開始判定、roll reference、control permanent stop latchをclearする
- 離床検知counterとapex detectorをreset

維持するもの:

- CommandReceiveで設定したFin zero
- AS5047D multi-turn tracking
- Paraの現在Hold状態
- compile-time parameter
- device driver初期化

Descent commit後はEmergency rollbackを受理しない。

## 6. Descent commit

Descentへの遷移は不可逆とする。

遷移条件は以下のOR:

1. 離床+10秒以降にpressure apex判定成立
2. 離床+17秒timer fallback

Descent commit時には`deployment_started=true`をlatchする。

古いgenerationから到着したtimer/eventは無視する。

## 7. Emergencyとdeploymentの競合

LiftoffDetectionEmergencyStopとdeploymentが同時に競合しても、どちらか一方だけがstate transitionに成功しなければならない。

- Emergencyが先にFlightをLiftoffDetectionへ変更した場合、旧generationのdeploymentは失敗する。
- deploymentが先にDescentへcommitした場合、Emergency rollbackは失敗する。

この競合を理由にDescentからLiftoffDetectionへ戻る経路を作らない。

## 8. Wire MissionState

既存地上系との互換のため、内部phaseからCAN上のstateを派生させる。

| Internal | Wire |
|---|---|
| CommandReceive | CommandReceive = 0 |
| LiftoffDetection | LiftoffDetection = 1 |
| Flight + RollControl inactive | EngineBurn = 2 |
| Flight + RollControl active | Control = 3 |
| Descent | Descent = 4 |

`EngineBurn`と`Control`はwire表示であり、内部state machineを分岐させる根拠にしない。
