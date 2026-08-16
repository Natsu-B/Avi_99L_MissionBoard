# 99L Mission Board

99L用Mission Board firmwareの最小実装です。旧MissionBoardの巨大なproduction runtimeを移植せず、飛行に必要な経路を小さいstate machineから作り直しています。

## 現在の内部state

- `CommandReceive`
- `LiftoffDetection`
- `Flight`
- `Descent`

`Control`は内部stateではありません。`Flight`中にRollControlが実際に出力されている間だけ、既存地上系とのwire互換のためCAN上を`Control(3)`として送信し、それ以外の`Flight`は`EngineBurn(2)`として送信します。

詳細仕様は [`docs/README.md`](docs/README.md) を参照してください。

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

+17秒fallbackと+25秒cutoffはCAN、SSC、LPS、SDの処理結果を待ちません。

## RollControl

RollControl本体は`Flight`内部の出力modeとして実装しています。

- 離床+8秒でICM roll rateとFin zeroを一度だけ確認
- +8秒でどちらかが成立しなければ、そのflightのRollControlを永久停止
- LPS/SSC/airspeedの一時欠落は永久停止にせずZeroHold
- 最初に実際のRollControl条件が成立した瞬間をroll偏差0としてgyro積分開始
- sensorが復帰しgyro積分の連続性が維持されていれば同じreferenceへControl復帰
- gyro gapが許容上限を超えた場合はそのflightのRollControlを永久停止
- validな対気速度が60 m/s以下になった場合はそのflightのRollControlを永久停止
- `airspeed unavailable`は60 m/s以下として扱わない
- Descent移行時はZeroHoldへ戻る

SSCは同じAirData I2C busで取得し、固定zero offset、moving average、LPS静圧、SSC温度からSaint-Venant式でairspeedを計算します。runtime calibration/NVSは使用しません。

**Roll gain、gyro bias、SSC zero、motor/ZeroHold定数は最終flight値ではありません。** `src/config/flight.hpp`の`TODO(SIMULATION)` / `TODO(HW_TEST)`を飛行前に確定してください。

## Logging

flight log producerは1 kHzです。SDへ直接書かず、まずPSRAM ring bufferへ64 byte recordとして保存します。

- PSRAMは最大8 MiB使用
- 他用途向けに最低512 KiBを残す
- SD mount/open/writeは低優先度`SdWriter` taskだけが行う
- SDが無い、mountに失敗する、writeがstallする場合もMission/Safety taskは待たない
- SDへ完全に書けたrecordだけPSRAMから解放
- PSRAMが満杯になった場合は古いrecordを上書きせず、新しいrecordをdropしてcounterを増やす
- 64 byte/recordなので約7.5 MiB確保できた場合は約120秒分を1 kHzで保持可能

log flagsにはIMU/LPS/SSC/airspeed validity、Control active、Control永久停止、Control reference validを記録します。

## ESP Libs

`Avi_ESP_Libs`は次のrevisionを使用します。

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

## 飛行前に残るもの

- SpicaでRollControl gain / authorityを確定
- 実機でgyro bias / SSC zeroを確定
- Fin motor極性・電気定数・ZeroHold gainを実機確認
- Para ±130 deg、Hold、2 s timeoutを実機確認
- flight中reset時のPara timer復元
- CAN/LPS/SSC/SD fault injection下で+17/+25秒Safety pathを実機確認
