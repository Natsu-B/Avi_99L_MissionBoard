# 全体アーキテクチャ

## 1. 目的

99L Mission Boardは、必要な飛行機能を小さいstate machineと独立したSafety経路で実装する。

旧MissionBoardの巨大な高位runtime、Preflight readiness、Calibration、複雑なNVS設定、dual motor profileなどは持ち込まない。

設計の中心は次の2点である。

1. Missionの状態遷移を最小化する。
2. パラシュート開放をControlや通信の故障から分離する。

## 2. 内部Mission phase

内部phaseは以下の4つだけとする。

- `CommandReceive`
- `LiftoffDetection`
- `Flight`
- `Descent`

`EngineBurn`と`Control`は内部stateにしない。Flight中のそのtickで制御条件を満たすかどうかを出力modeとして扱う。

既存地上系とのwire互換のため、CAN上ではFlightを通常`EngineBurn`として表現し、RollControl実行中だけ`Control`を派生表示してよい。

## 3. 責務分離

### Mission state machine

担当するもの:

- StartSequence
- 離床確定
- LiftoffDetectionEmergencyStopによるrollback
- Flight generation
- Descentへの不可逆commit
- power cutoff latch

担当しないもの:

- servoの具体的UART transaction
- motor PWMの具体的制御
- SD write
- LoRa
- GUI

### Fin actuator

- AS5047Dの読取り
- CommandReceiveでの現在位置0度定義
- Free / ZeroHold
- 次段のRollControl出力
- encoderが利用不能な場合のmotor safe化

### Parachute actuator

- STS3215 UART所有
- boot後の現在位置Hold
- Open/Closeの固定相対130度move
- 自動deployment
- +25秒physical power cutoff後の再投入禁止

### Safety path

Safety処理は、CAN、SD、LPS、SSC、Controlなどの結果を待たない。

最低限、離床時刻が確定した後は以下を独立して進める。

- +17秒: timer fallbackによるdeployment commit
- +25秒: Para電源OFF、補助5V OFF、Fin motor Hi-Z

### Storage

flight-critical producerはSDへ直接writeしない。

- producer -> PSRAM ring
- 低優先度writer -> SD

SD faultはMission state、Control、Para、Safetyを停止させない。

## 4. Dependency方針

低位device driverとして`Avi_ESP_Libs`を利用する。

Mission側ではdevice APIを薄く包み、旧MissionBoardの高位runtime型を依存にしない。

対象device:

- ICM42688
- AS5047D
- LPS25HB
- SSCDRRN005PD2A5（Control追加時）
- STS3215 / STSCREATE
- CANCREATE

## 5. NVS方針

現最小実装ではNVSを使用しない。

以下は永続保存しない。

- Fin zero
- Para Open/Close位置
- gyro bias calibration結果
- SSC zero calibration結果
- Preflight readiness

Fin zeroは毎boot後にCommandReceiveで`FinHoldCurrent`を行った位置を0度とする。

Paraは絶対endpointを保存せず、Open/Closeを固定相対130度として実行する。

飛行中reset復元は別仕様として明示的に追加するまで保証しない。

## 6. 他基板との境界

通信基板・地上受信基板・GUIはほぼ完成済みのため、原則として内部ロジックを凍結する。

Mission Board側が既存CAN ID、packet layout、command/result semanticsを維持することで互換を取る。

Mission内部stateの簡略化を理由に、他基板へ同じstate machine変更を波及させない。
