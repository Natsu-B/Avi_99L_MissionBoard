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

Fin zeroはNVSへ保存しません。再起動後はencoderの周回数を復元できないため`zero_reference_valid=false`、`zero_hold_achieved=false`から開始し、CommandReceiveで`FinZero`をやり直します。

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

FIN0003/FIN0004を取得した30 kHz、10 bit、1 kHz実機controllerを、
Spicaと同じTorqueMapper基準のrequested torque座標へ換算しています。

- `Kp = 65.390941574 N m/rad`
- `Ki = 4.577365910 N m/(rad s)`
- `Kd = 3.269547079 N m s/rad`
- integral limit `= ±0.034906585 rad s`
- velocity LPF `tau = 20 ms`
- hold deadband `= 0.05 deg / 0.5 deg/s`
- minimum active error `= 0.08 deg`

ZeroHoldとRollControlは同じactuator mapperを通ります。mapper後にmotionを要求し、raw commandが0より大きく70未満なら、実機で使用した70 commandへ補償します。その後にcurrent制約を再適用し、PWMを`±1024 / ±100 %`へ制限します。commandからLEDC countへの変換はFIN0003と同じ整数floorで、70 commandは69 count、1024 commandは1023 countです。back-EMF下で70 command補償がrequested torqueと逆向きの計算currentを作る場合は補償前dutyへ戻します。旧characterizationの`±600 command`と旧RollControl `1.21208 N m`はsoftware authority limitとして使用しません。±15 degより外向きのrequested torqueは禁止します。bus/back-EMF条件でcurrent制約を実現不能なら計算値をclampして隠さずdriveを0へ落とし、9800 rpm以上では同方向加速torqueを禁止します。gearbox 6000 rpm超過も内部Fin telemetryの独立limited値として保持します。`zero_hold_achieved`とmapperのeffective/current/duty/limitをCAN/SDへ外部出力するprotocol拡張は未実装です。

`zero_reference_valid`はmotor-side zero capture済み、`zero_hold_achieved`は`|angle| <= 1 deg`かつ`|rate| <= 2 deg/s`が、5 msを超えるsample gapなしで200 ms連続したことを表します。後者がControl gate条件です。どちらも左右physical finの厳密な空力0度を保証しません。invalid/stale sample、mode解除、zero recaptureではPID stateをresetし、current/duty/角度/速度制約中は積分をfreezeします。

requested/effective torqueとcurrentはmapper計算座標であり、actual shaft torque/currentの実測値ではありません。

FIN0006/FIN0007 FIT-derived nonlinear plantのZeroHold再検証ではこの
baselineが50/50 caseで成立したため値を維持します。ただし実測したのは
commandとoutput-equivalent angle/rateであり、actual current/torqueが未計測のため
ZeroHoldも`production_selectable=false`です。

## RollControl

RollControl本体は`Flight`内部の出力modeとして実装しています。

- 離床+8秒でICM、Fin、LPS、SSC、airspeedのhealth/freshnessと`zero_hold_achieved`を一度だけ確認
- +8秒で必須条件が成立しなければ、そのflightのRollControlを永久停止
- 最初に実際のRollControl条件が成立した瞬間をroll偏差0としてgyro積分開始
- Control entry後にattitude、Fin angle/rate、LPS、SSC、airspeedのどれかがinvalid/staleになるか、validな対気速度が60 m/s以下になれば即時exit
- exit後はZeroHold（zero referenceが無効ならcoast）とし、同一flight内で再entryしない
- Descent移行時はZeroHoldへ戻る

Fin angle/rateはAS5047D encoder軸のwrapped angle/rateではなく、連続unwrapしたencoder角をtotal gear ratioで変換したFin出力軸値を使用します。

SSCは同じAirData I2C busで取得し、固定zero offset、moving average、LPS静圧、SSC温度からSaint-Venant式でairspeedを計算します。runtime calibration/NVSは使用しません。

Roll gain scheduleは100 % dutyを取得したFIN0007 drive/brake FIT-only
effective plantに、全速度共通の`Q=diag([200,50,0.05,0.5])`、`R=1`を
適用した7点候補です。現行Roll出力もdrive/brakeのためtopologyは一致します。
ただしFIN0009 strict holdoutはtarget-segment angle RMSE `5.530028 deg`で
事前固定した1 deg gateを満たさずFAILです。さらにFIN0010確認はacceleration
R² `0.430368`、recursive rate fit `56.2689 %`、target RMSE `11.53965 deg`、
whole-run drift `+20.26027 deg`であり、これもFAILです。FIN0010のsource SHA-256は
CSV `cc8ac1b0bd3f7cf2af08074d313e4f194e3daacc84d9254f888c5a3342df0254`、metadata
`7d7c41622715cc91fe334aa674fc809169731fd719521320f4d6030d1c7db177`です。

**statusは`PROVISIONAL_BRAKE_FIXED_BY_USER_DIRECTION_VALIDATION_GATE_NOT_MET / NO_GO`、
`production_selectable=false`です。** ユーザ指定でBrake候補をfirmwareへ固定しますが、
strict holdoutまたはFIN0010からgain/modelをretuneしておらず、flight qualificationとは
扱いません。Spica FIN0010 revision artifact SHA-256は
`2eb8ac6f6b94fb99b22417546ef437c557b676c56545e0721e4b15c94dff25b1`、
consumed marker SHA-256は
`4cedffcdb8f389116bfe10c8a6126ca34e64bff2b6cc04898889b79f73fa09c2`、
exact値は`src/config/flight.hpp`で管理します。

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

`2562ef05d3f7673a5681d8a3739c874f95811c73`

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

- FIN0009/FIN0010で顕在化したactuator model angle/drift errorを解消し、
  RollControl scheduleを実機qualification
- 実機でgyro bias / SSC zeroを確定
- AS5047Dの連続unwrapが実機最大Fin速度・想定sample gapで周回誤認しないことを確認
- Fin motor極性・電気定数が実機と固定configで一致することを確認
- Para ±130 deg、Hold、2 s timeoutを実機確認
- flight中reset時のPara timer復元
- CAN/LPS/SSC/SD fault injection下で+17/+25秒Safety pathを実機確認
