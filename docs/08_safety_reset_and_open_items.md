# Safety, Reset, and Open Items

## 1. Safety優先順位

Mission Boardの安全優先順位は次の通り。

1. +25秒absolute actuator power cutoff
2. +17秒timer fallbackによるdeployment
3. Descent commit後のPara Open
4. Fin safe / ZeroHold
5. RollControl
6. telemetry / CAN
7. logging / SD

下位機能のfaultやstallが上位Safety処理を待たせてはならない。

## 2. パラシュート独立性

離床後、timerさえ正常に進行していれば+17秒でDescentへ入れることを要求する。

以下の故障はtimer fallbackを禁止しない。

- LoRa fault
- CAN fault
- 通信基板fault
- LPS fault
- SSC fault
- SD fault
- Control fault
- Fin fault

LPS apexは早期deployment条件として利用するが、LPS unavailable時は単に+17秒fallbackへ任せる。

## 3. +25秒cutoff

+25秒ではServoの状態確認を待たず物理powerをOFFする。

同時に:

- Fin motor Hi-Z
- auxiliary 5 V OFF
- Para power OFF

を行う。

UART transactionがstallしていてもSafety task側で実行できる構造にする。

## 4. Reset

### 4.1 現最小実装

現段階ではflight中resetからのMission復元を保証しない。

NVS partitionを持たず、Fin zero、AS5047D multi-turn周回数、flight elapsedを永続化していない。

AS5047Dは1回転内絶対角のみを返す。gear ratioによりencoder側が複数回転するため、reboot後に何周目だったかを一意に復元できない。このためFin zeroをNVSへ保存して再利用する設計は禁止する。

reboot後は`zero_valid=false`とし、CommandReceiveで物理Finを基準位置へ合わせて`FinZero`を再実行する。

この状態は**本番投入前の未完事項**として扱う。

### 4.2 復元を追加する場合の原則

復元情報は最小限とする。

保存候補:

- flight started / liftoff committed
- deployment started
- power cutoff done
- RTCで復元可能なliftoff time

保存しないもの:

- Fin zero
- AS5047D multi-turn周回数
- Control reference
- roll integration state
- Para absolute endpoint
- Calibration result
- Preflight readiness

reset後はRollControlへ復帰しない。

飛行中だったことが確実で経過時刻を復元できる場合は、Para timerとcutoffだけを継続する方向を優先する。

完全なPower-on resetで時刻を復元できない場合のPara fallbackは、別途安全方針を確定する。

## 5. 実機確認必須項目

### Fin

- `FinZero`でその時点の物理Fin位置が論理0度になること
- `FinZero`がmotorを駆動せず現在modeを維持すること
- `FinFree`でmotorがHi-Zになり、zero referenceとAS5047D trackingが維持されること
- `FinFree`中に動かした後、`FinHold`で元の論理0度へ戻って保持すること
- AS5047D 0/360度境界を跨いで連続unwrapできること
- encoder側が複数回転してもFin角が`total gear ratio`で正しく変換されること
- 想定最大Fin速度・sample gapで隣接valid sampleのencoder回転が180度を超えず、周回誤認しないこと
- reboot後にzeroが無効化され、古い周回数を再利用しないこと
- motor正負極性
- 本番固定ZeroHold値がbuildへ入っていること
- motor電気定数
- 最大current/duty
- encoder/rate invalid時にmotorがHi-Zへ移ること

### Para

- boot後に現在位置Holdできること
- Openが物理的に反時計回り130度であること
- Closeが物理的に時計回り130度であること
- 130度move後にHoldすること
- STS timeoutが2秒で十分であること
- move command結果が曖昧な場合に二重130度を発行しないこと
- +25秒で物理電源がOFFになること

### Liftoff

- ICM条件の実機再現
- LPS条件の実機再現
- 片系sensor fault時に他方が継続すること

### Safety

- CAN切断中も+17秒fallbackが進むこと
- LPS/SSCを停止しても+17秒fallbackが進むこと
- SDをstallさせても+17/+25秒Safety処理が進むこと
- LiftoffDetectionEmergencyStopと+17秒deploymentの競合で二重遷移しないこと

### Logging

- PSRAM allocation量
- 1 kHz producerでdropしないこと
- SDの数秒stallをPSRAMで吸収すること
- PSRAM full時に未書込old dataを上書きしないこと

## 6. Control用に確定する値

以下を飛行前に固定する。

- gyro bias
- SSC zero
- SSC/LPS freshness
- airspeed算出parameter
- RollControl gain
- Control output torque/current limit
- gyro欠落をどこまで補間可能とするか
- AS5047D multi-turn unwrapが成立する最大sample gap / 実機Fin速度

これらはruntime CalibrationやNVS設定としては実装しない。

## 7. 60 m/s停止

validなairspeedが60 m/s以下となった時点で、そのflightのRollControlを永久停止する。

`airspeed unavailable`を60 m/s以下とみなしてはならない。

- unavailable -> 一時ZeroHold
- valid <= 60 m/s -> permanent Control stop

## 8. 開傘fallback時刻

現仕様は+17秒。

15秒等への変更は未決定。

変更する場合は:

- Vault
- flight simulation
- 審査書
- 開傘荷重・頂点条件

を確認して確定する。

## 9. 仕様上削除した機能

新MissionBoardへ再導入しないもの:

- dual motor switching
- ForceStartSequence
- ActuatorEmergencyStop
- PreflightCalibration
- Para absolute endpoint NVS
- Fin zero NVS
- 7/5 bit readiness gate
- Control/EngineBurn内部state往復
- Control再entry禁止用の複数latch

再導入が必要になった場合は、既存旧実装をそのまま戻さず、必要性とSafety影響をdocsへ先に記述する。
