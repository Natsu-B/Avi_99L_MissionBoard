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
- `FinFree (0x10)`: 動翼motorをHi-Z。Fin zeroとAS5047D multi-turn trackingは保持
- `FinZero (0x11)`: 現在のmulti-turn AS5047D位置を論理Fin 0 degとしてRAMへcapture。motor modeは変更しない
- `FinHold (0x13)`: capture済みのFin 0 degをZeroHold。現在位置を再zero化しない
- `ParaOpen (0x25)`: STS3215を反時計回りに130 deg相対移動し、その位置をHold
- `ParaClose (0x26)`: STS3215を時計回りに130 deg相対移動し、その位置をHold

FinはAS5047Dの1回転角をsampleごとにunwrapしてRAM上のmulti-turn encoder角を維持し、`kTotalGearRatio`でFin出力軸角へ変換します。Free中もAS5047D trackingを止めません。

Fin zeroはNVSへ保存しません。再起動後はencoderの周回数を復元できないため`zero_valid=false`から開始し、CommandReceiveで`FinZero`をやり直します。

同じtransaction ID・同じcommandの再送はcacheされた結果を返し、FinZeroの再captureやPara相対130 degの二重実行を行いません。

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

## ZeroHold

本飛行ではSpicaのfin装着characterizationを固定値として採用します。

- `Kp = 65.390941574 N m/rad`
- `Kd = 3.269547079 N m s/rad`
- characterization command limit `= ±600`
- requested torque limit equivalent `= ±1.369544677734375 N m`
- rate continuous dead-zone `= 1.0 deg/s`
- requested-torque dead-zone/hysteresis `= none`

演算順序はSpicaの`mission_zero_hold_step`と同じく、angle/rateへcontinuous dead-zoneを適用してPD要求torqueを計算し、最後にSpica実機試験の`±600 command`相当へclampします。AS5047D angleまたはFin rateがinvalidなtickではZeroHold出力を生成せずmotorをHi-Zへ落とし、validなrateが復帰したtickから保持を再開します。

`±1.369544677734375 N m`はSpica artifactの`0.0022825744628906255 N m/command`換算を使ったrequested-torque相当値です。fin装着試験ではactual current/torqueを直接計測していないため、実測済みなのは`±600 command`までのcommand-domain authorityであるという区別を残します。

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

Fin angle/rateはAS5047D encoder軸のwrapped angle/rateではなく、連続unwrapしたencoder角をtotal gear ratioで変換したFin出力軸値を使用します。

SSCは同じAirData I2C busで取得し、固定zero offset、moving average、LPS静圧、SSC温度からSaint-Venant式でairspeedを計算します。runtime calibration/NVSは使用しません。

**ZeroHold定数は本飛行固定値です。Roll gain、gyro bias、SSC zeroはまだ別途確定対象です。** 固定値は`src/config/flight.hpp`で管理します。

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
- AS5047Dの連続unwrapが実機最大Fin速度・想定sample gapで周回誤認しないことを確認
- Fin motor極性・電気定数が実機と固定configで一致することを確認
- Para ±130 deg、Hold、2 s timeoutを実機確認
- flight中reset時のPara timer復元
- CAN/LPS/SSC/SD fault injection下で+17/+25秒Safety pathを実機確認
