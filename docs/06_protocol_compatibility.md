# Protocol Compatibility

## 1. 方針

通信基板、地上側受信firmware、GroundFirmwareはほぼ完成済みのため、MissionBoard再実装を理由にそれらの内部ロジックを変更しない。

互換はMissionBoard側で維持する。

## 2. 凍結対象

原則として以下を凍結する。

- CAN identifier
- packet length
- field packing
- quantization
- command transaction ID
- command resultのAccepted / Completed / Rejected / Failed semantics
- LoRa packet構造
- 通信基板のfallback logic
- 地上局GUIの既存telemetry解釈

致命的なprotocol bugを除き、Mission内部stateの変更だけを理由にwire formatを変えない。

## 3. 使用CAN ID

新MissionBoardが現時点で直接必要とする主要ID:

| CAN ID | 用途 |
|---:|---|
| `0x002` | LiftoffDetectionEmergencyStop |
| `0x010` | GenericCommandRequest |
| `0x011` | CommandResult |
| `0x020` | MissionEvent |
| `0x100` | KinematicsTelemetry |
| `0x101` | ControlTelemetry |
| `0x102` | MissionStatusTelemetry |
| `0x103` | PowerTimeTelemetry |
| `0x104` | DescentCoreTelemetry |
| `0x107` | AttitudeTiltTelemetry |
| `0x108` | LpsTelemetry |
| `0x109` | AirspeedTelemetry |
| `0x10A` | ControlRollTelemetryV2 |

全てのmessageを最初から実データで埋める必要はない。まだ実装されていないsourceは既存error/unavailable表現を使用する。

## 4. Generic command

公開command:

| code | command | semantics |
|---:|---|---|
| `0x01` | StartSequence | LiftoffDetectionへ遷移 |
| `0x10` | FinFree | Fin motor Hi-Z。Fin zeroは保持 |
| `0x11` | FinZero | 現在のmulti-turn encoder位置を論理Fin 0 degとしてcapture |
| `0x13` | FinHold | capture済みFin 0 degを保持。Zero再captureなし |
| `0x25` | ParaOpen | -130 deg relative |
| `0x26` | ParaClose | +130 deg relative |

argsは現在の最小commandでは全0を要求する。

`0x13`は旧`FinHoldCurrent`の「現在位置を再zero化する」意味では使用しない。wire code自体は維持し、現在の0 deg referenceを保持する`FinHold`へ意味を固定する。

未対応旧commandは、誤って別動作へaliasせず`Rejected / NotSupported`とする。

## 5. Command result

CommandResultのwire enumは既存値を維持する。

phase:

- Accepted = 0
- Completed = 1
- Rejected = 2
- Failed = 3

reasonも既存wire値を維持する。

相対Para commandでは再送安全性が重要なため、同じtransaction IDの同一commandは結果cacheを返してside effectを再実行しない。

FinZeroもreplay時に再captureしない。同じtransaction IDのreplayは最初の結果を返す。

## 6. Wire MissionState

既存5-state表現を維持する。

```text
0 CommandReceive
1 LiftoffDetection
2 EngineBurn
3 Control
4 Descent
```

Mission内部では4 phaseだが、wireでは以下のように派生する。

- internal CommandReceive -> 0
- internal LiftoffDetection -> 1
- internal Flight, Control inactive -> 2
- internal Flight, Control active -> 3
- internal Descent -> 4

地上側へ新しい`Flight` enumを追加しない。

## 7. Telemetry source unavailable

Mission再実装の初期段階では未実装または一時Unavailableなsourceが存在し得る。

その場合、0やもっともらしい値を偽装せず、既存protocolのerror/unavailable rawを送る。

sourceが実装された時点で同じpacket schemaのままnumeric値へ切り替える。

## 8. 他repositoryの扱い

原則変更しない対象:

- `Natsu-B/99L_comboard`
- `CREATE-ROCKET/Avi_tenkatenn_board` の地上受信firmware
- `CREATE-ROCKET/Avi_99L_GroundFirmware`

ただしGround側のcommand名・UIが旧`FinHoldCurrent` semanticsを仮定している場合は、wire code `0x13`を変えず表示/説明だけを`FinHold`へ更新し、`FinZero (0x11)`を送れる経路を用意する。

## 9. Vault

CAN/LoRa contractに変更が必要になった場合は、MissionBoard codeだけを先行変更しない。

1. Vaultの通信仕様を更新
2. compatibility影響を確認
3. MissionBoard
4. 必要な場合だけ通信基板/地上側

の順で反映する。
