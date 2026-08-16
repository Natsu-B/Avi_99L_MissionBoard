# 99L Mission Board 仕様書

このdirectoryは `Natsu-B/Avi_99L_MissionBoard` のMission側仕様をまとめる。

## 文書一覧

- [00_architecture.md](00_architecture.md): 全体設計、責務分離、設計原則
- [01_state_machine.md](01_state_machine.md): Mission内部stateと遷移
- [02_commands_and_actuators.md](02_commands_and_actuators.md): CommandReceive、動翼、STS3215
- [03_flight_sequence.md](03_flight_sequence.md): 離床検知、Flight、開傘、Descent
- [04_control_spec.md](04_control_spec.md): 動翼制御の確定方針
- [05_logging_and_storage.md](05_logging_and_storage.md): PSRAM、SD、log format
- [06_protocol_compatibility.md](06_protocol_compatibility.md): CAN wire互換と他基板の凍結方針
- [07_hardware_and_constants.md](07_hardware_and_constants.md): ESP32-S3、pin、固定値、暫定値
- [08_safety_reset_and_open_items.md](08_safety_reset_and_open_items.md): Safety、reset、未確定事項、実機確認

## Source of truth

本repositoryは99L Mission Boardの実装仕様を記述する。飛行安全要求と通信仕様については `Natsu-B/Vault` および提出審査書との整合を維持する。

仕様の優先順位は次の通りとする。

1. 打上げ審査で承認された安全要求
2. `Natsu-B/Vault` の99L共通・通信仕様
3. 本repositoryの `docs/`
4. 実装中のコメント・TODO

上位仕様と本docsに差異を見つけた場合は、暗黙に解釈せずdocsと実装の両方を更新する。

## Scope

本firmwareはMission Board上で以下を担当する。

- CommandReceiveでの最小actuator操作
- 離床検知
- Flight timer管理
- 動翼0度保持、および次段で追加するロール制御
- 頂点判定とtimer fallbackによるパラシュート開放
- Descent中のactuator安全化
- CAN telemetry / command compatibility
- flight logのPSRAM stagingとSD保存

通信基板、地上側受信基板、GUIの内部実装は原則として本repositoryの責務外とする。
