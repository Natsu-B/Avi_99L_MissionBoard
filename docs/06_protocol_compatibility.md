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

| code | command |
|---:|---|
| `0x01` | StartSequence |
| `0x10` | FinFree |
| `0x13` | FinHoldCurrent |
| `0x25` | ParaOpen |
| `0x26` | ParaClose |

argsは現在の最小commandでは全0を要求する。

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

Mission再実装の初期段階では、Control、airspeed、3D attitudeなど未実装sourceが存在する。

その場合、0やもっともらしい値を偽装せず、既存protocolのerror/unavailable rawを送る。

sourceが実装された時点で同じpacket schemaのままnumeric値へ切り替える。

## 8. 他repositoryの扱い

原則変更しない対象:

- `Natsu-B/99L_comboard`
- `CREATE-ROCKET/Avi_tenkatenn_board` の地上受信firmware
- `CREATE-ROCKET/Avi_99L_GroundFirmware`

変更が必要なのは、wire contract自体を変更すると決定した場合だけとする。

Mission内部実装の簡略化、task分割、NVS削除、Control algorithm変更は他repository変更理由にしない。

## 9. Vault

CAN/LoRa contractに変更が必要になった場合は、MissionBoard codeだけを先行変更しない。

1. Vaultの通信仕様を更新
2. compatibility影響を確認
3. MissionBoard
4. 必要な場合だけ通信基板/地上側

の順で反映する。
