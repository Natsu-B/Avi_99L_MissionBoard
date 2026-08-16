# 99L Mission Board

99L用Mission Board firmwareの最小実装です。旧MissionBoardの巨大なproduction runtimeを移植せず、飛行に必要な経路を小さいstate machineから作り直しています。

## 仕様文書

実装・今後追加するControl・Safetyの基準仕様は[`docs/README.md`](docs/README.md)から参照できます。

主な文書:

- [`docs/00_architecture.md`](docs/00_architecture.md): 全体設計と責務分離
- [`docs/01_state_machine.md`](docs/01_state_machine.md): 4-state Mission phase
- [`docs/02_commands_and_actuators.md`](docs/02_commands_and_actuators.md): Fin/Para command
- [`docs/03_flight_sequence.md`](docs/03_flight_sequence.md): 離床から開傘・cutoffまで
- [`docs/04_control_spec.md`](docs/04_control_spec.md): +8秒以降のRollControl仕様
- [`docs/05_logging_and_storage.md`](docs/05_logging_and_storage.md): PSRAM -> SD logging
- [`docs/06_protocol_compatibility.md`](docs/06_protocol_compatibility.md): 通信基板・地上側との互換方針
- [`docs/07_hardware_and_constants.md`](docs/07_hardware_and_constants.md): ESP32-S3、pin、定数
- [`docs/08_safety_reset_and_open_items.md`](docs/08_safety_reset_and_open_items.md): Safety、reset、未決事項

## 現在の内部state

- `CommandReceive`
- `LiftoffDetection`
- `Flight`
- `Descent`

CAN wire上は既存地上系との互換性のため、`Flight`を通常は`EngineBurn(2)`として送信します。将来RollControlを追加した際は、制御中だけ`Control(3)`を派生表示します。

## CommandReceive

- `StartSequence (0x01)`: 無条件で`LiftoffDetection`へ遷移
- `FinFree (0x10)`: 動翼motorをHi-Zにしてzeroを無効化
- `FinHoldCurrent (0x13)`: 現在のAS5047D角度を0 degとしてZeroHold開始
- `ParaOpen (0x25)`: STS3215を反時計回りに130 deg相対移動し、その位置をHold
- `ParaClose (0x26)`: STS3215を時計回りに130 deg相対移動し、その位置をHold

同じtransaction ID・同じcommandの再送はcacheされた結果を返し、相対130 degを二重実行しません。

## LiftoffDetection / Flight

- ICM42688: 20 sample平均の加速度normが2 Gを50 ms連続で超えると離床
- LPS25HB: 既存99L判定と同じ圧力低下条件でも離床
- 離床時刻は検知時刻の1秒前
- `LiftoffDetectionEmergencyStop (CAN 0x002)`は`Flight`から`LiftoffDetection`へ戻す
- `Descent`へcommitした後はEmergency rollbackしない
- 離床+10秒以降はLPSの頂点判定を使用
- 圧力条件が成立しなくても離床+17秒で`Descent`
- 離床+25秒でPara電源、`+5V_short`、動翼motorをSafety taskが物理的にOFF/Hi-Zへ移行

+17秒fallbackと+25秒cutoffはCANやLPS taskの処理結果を待ちません。

## Logging

flight log producerは1 kHzです。SDへ直接書かず、まずPSRAM ring bufferへ64 byte recordとして保存します。

- PSRAMは最大8 MiB使用
- 他用途向けに最低512 KiBを残す
- SD mount/open/writeは低優先度`SdWriter` taskだけが行う
- SDが無い、mountに失敗する、writeがstallする場合もMission/Safety taskは待たない
- SDへ完全に書けたrecordだけPSRAMから解放
- PSRAMが満杯になった場合は古いrecordを上書きせず、新しいrecordをdropしてcounterを増やす
- 64 byte/recordなので約7.5 MiB確保できた場合は約120秒分を1 kHzで保持可能

ログ中の加速度はmg、gyroは0.1 deg/s、fin角は0.01 deg単位として格納します。

## ESP Libs

`Avi_ESP_Libs`は現在の実装で次のrevisionを使用します。

`122b4bebdfc89eaef364ff59b3bcd18010f83d5e`

## Build

```sh
pio run -e avi_99l_missionboard
```

## Host test

```sh
cmake -S host_test -B host_test/build
cmake --build host_test/build
ctest --test-dir host_test/build --output-on-failure
```

## 次段で追加するもの

RollControlとSSC airspeedを`Flight` state内部の出力modeとして追加します。MissionStateは増やしません。制御開始時点をroll基準0とし、必要sensorが一時的に欠けた場合はZeroHold、validな対気速度が60 m/s以下になった場合はそのflightでRollControlを永久停止します。
