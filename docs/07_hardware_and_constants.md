# Hardware and Constants

## 1. MCU

対象MCUはESP32-S3-WROOM-1-N16R8とする。

- Flash: 16 MiB
- PSRAM: 8 MiB
- CPU: 240 MHz
- framework: ESP-IDF
- PlatformIO board variant: `esp32_s3r8n16`

この構成を99L Mission Boardの標準とする。

## 2. Pin assignment

### AS5047D

| signal | GPIO |
|---|---:|
| MOSI | 4 |
| MISO | 5 |
| SCLK | 6 |
| CS | 7 |

- SPI host: SPI2
- frequency: 8 MHz
- 1回転内の絶対角を返す
- multi-turn位置はRAM上でsample間差分をunwrapして保持する
- encoder軸角を`total gear ratio`で割ってFin出力軸角へ変換する

AS5047Dのmulti-turn周回数はsensor単体からreboot後に復元できないためNVSへ保存しない。

### ICM42688

| signal | GPIO |
|---|---:|
| CS | 8 |
| INT | 15 |
| MISO | 16 |
| MOSI | 17 |
| SCLK | 18 |

- SPI host: SPI3
- frequency: 8 MHz

### SDMMC

| signal | GPIO |
|---|---:|
| DAT1 | 3 |
| DAT0 | 9 |
| CLK | 10 |
| CMD | 11 |
| DAT3 | 12 |
| DAT2 | 13 |

### CAN

| signal | GPIO |
|---|---:|
| RX | 14 |
| TX | 21 |

- bitrate: 125 kbit/s

### AirData I2C

| signal | GPIO |
|---|---:|
| SDA | 47 |
| SCL | 48 |

- I2C0
- bus frequency: 300 kHz

### Fin motor

| signal | GPIO |
|---|---:|
| IN1 | 39 |
| IN2 | 38 |

- PWM: 30 kHz
- 1 motor only

### Parachute STS3215

| signal | GPIO |
|---|---:|
| RX | 41 |
| TX | 42 |
| power enable | 44 |

- UART1
- baudrate: 1 Mbit/s
- servo ID: 1

### Auxiliary / LEDs

| function | GPIO |
|---|---:|
| +5V_short enable | 40 |
| LED1 | 45 |
| LED2 | 46 |

## 3. Flight timing constants

| item | value | status |
|---|---:|---|
| Control eligibility | +8 s | 確定方針 |
| pressure apex enable | +10 s | 現行仕様 |
| deployment fallback | +17 s | 現行仕様・審査整合対象 |
| absolute power cutoff | +25 s | Safety要求 |

15秒等へ変更する場合はVault/審査/simulation確認を伴う。

## 4. Para constants

| item | value | status |
|---|---:|---|
| Open delta | -130 deg | 方向実機確認必要 |
| Close delta | +130 deg | 方向実機確認必要 |
| speed | 180 deg/s | 暫定 |
| acceleration | 360 deg/s^2 | 暫定 |
| torque limit | 20 % | 暫定 |
| hold torque | 20 % | 暫定 |
| power stabilization | 100 ms | 暫定 |
| motion timeout | 2 s | TODO(HW_TEST) |
| reconnect interval | 1 s | 暫定 |

Open負方向を「反時計回り」、Close正方向を「時計回り」と定義する。実機機構で必ず確認する。

## 5. Fin ZeroHold constants

30 kHz、10 bit、1 kHzで取得したFIN0003/FIN0004の実機controllerを、Spica TorqueMapper基準のrequested torque座標へ換算した値をbaselineとする。FIN0006/FIN0007 FIT-derived nonlinear plantの再検証で50/50 caseが成立したため値を維持するが、actual current/torqueが未計測のため`production_selectable=false`を維持する。

PID、20 ms速度LPF、deadband、minimum active errorは実機controllerと同じ構造を使う。characterization用`±600 command`は本番software authority limitへ流用せず、ZeroHold/RollControlとも共通mapper後のcurrent、bus voltage、`±1024 / ±100 %`、±15 deg outward blockで制限する。

| item | value | status |
|---|---:|---|
| Kp | 65.390941574 N m/rad | baseline |
| Ki | 4.577365910 N m/(rad s) | baseline |
| Kd | 3.269547079 N m/(rad/s) | baseline |
| integral limit | ±0.034906585 rad s | 実機controller換算 |
| velocity LPF tau | 20 ms | 実機controller |
| hold deadband | 0.05 deg / 0.5 deg/s | 実機controller |
| minimum active error | 0.08 deg | 実機controller |
| characterization command limit | ±600 | 取得時のみ、本番limitではない |
| N m per command | 0.0022825744628906255 | TorqueMapper換算 |
| minimum active command | 70 | mapper後PWM補償 |
| command full scale | 1024 | 10 bit command座標 |
| PWM frequency | 30 kHz | FIN0003/FIN0004と本番で固定 |
| motor resistance | 3.48 ohm | TorqueMapper固定値 |
| torque constant | 0.00855 N m/A | TorqueMapper固定値 |
| speed constant | 1120 rpm/V | TorqueMapper固定値 |
| gear ratio | 176.175 | 機構固定値 |
| drivetrain efficiency | 0.60 | TorqueMapper換算仮定 |
| bus voltage | 9.0 V | fin装着換算条件 |
| max current | 2.2 A | TB67 hardware setting |
| gearbox continuous speed | 6000 rpm | 超過をtelemetry/limited判定 |
| motor hard speed | 9800 rpm | 同方向加速torqueを禁止 |
| PWM max duty | 1.0 | software上限 |
| positive torque polarity | IN1 | 実機方向確認対象 |

`gear ratio = 176.175`はencoder/motor側角度をFin出力軸角へ変換するために使用する。ZeroHold/RollControlへ渡すFin angle/rateはgear ratio変換後の値とする。

Spicaのfin装着characterizationで直接観測したのはcommandとoutput-equivalent angle/rateである。requested/effective torque、current、drivetrain efficiencyはmapper計算値であり、actual motor currentまたはactual shaft torqueの実測値ではない。`estimated_motor_current`は駆動候補dutyからの計算値を数値上clampせず保持し、bus voltage範囲で2.2 A制約を実現できない場合は`current_limit_unrealizable`としてdriveを0へ落とす。runtime command/NVSから値を変更する仕組みは設けない。

AS5047D angle/rateがinvalidまたはstale、dtが非正または上限外のtickでは要求torqueを生成せずmotorをHi-Zとし、PID stateと`zero_hold_achieved`をresetする。current/duty/角度/速度制約を検出したtickは、そのtickで追加したintegral成分だけをrollbackして速度LPF stateを維持する。integralは常に±0.034906585 rad s内へ制限する。back-EMFにより70 command補償後の計算currentがrequested torqueと逆向きになる場合は補償前dutyへ戻し、最終dutyでもtorque方向を実現できなければdriveを0へ落とす。

ZeroHold PID、成立時間、encoder unwrapのnon-atomic stateは1 kHz realtime taskだけが更新する。Recovery taskはatomic reset requestとinvalid flagを発行し、motorをcoastしてからtransportを復旧する。realtime側はencoder mutexを待たずに取得できたtickだけsensor readからdrive反映までを完結し、取得できないtickはstateをresetしてdriveを生成しない。FinFree、force-safe、Recovery、Safety cutoffも同じmutexでmotor writerを直列化する。Safety taskはatomic inhibitを先にlatchし、進行中writerの終了後にmotorをcoastするため、古いrealtime snapshotがcutoff後に非0 driveを再開できない。

command-to-countはFIN0003の整数floorを再現し、70 commandを69 count、1024 commandを1023 countへ変換する。ZeroHold出力はFIN0003と同じdrive/coastである。Roll出力はdrive/brakeであり、gain候補のFIT run FIN0007とtopologyは一致する。ただしFIN0009 strict holdoutとFIN0010確認が共に事前固定gateを満たさないため、`PROVISIONAL_BRAKE_FIXED_BY_USER_DIRECTION_VALIDATION_GATE_NOT_MET / NO_GO`、`production_selectable=false`とする。

FIN0010 revision artifact SHA-256は`2eb8ac6f6b94fb99b22417546ef437c557b676c56545e0721e4b15c94dff25b1`、read-once consumed marker SHA-256は`4cedffcdb8f389116bfe10c8a6126ca34e64bff2b6cc04898889b79f73fa09c2`である。

±15 degの外向きcommand禁止はZeroHold authority変更後も維持する。outward判定と9800 rpmでの加速禁止はPWM電圧符号でなくrequested torque方向で行い、back-EMFでPWM符号が反転するbrakingを誤って禁止しない。gearbox 6000 rpm超過は独立してtelemetryへ残す。

`zero_reference_valid`はzero capture transactionの成立、`zero_hold_achieved`はmotor-side/output-equivalentで`|angle| <= 1 deg`かつ`|rate| <= 2 deg/s`が200 ms連続したことを表す。physical left/right finの厳密な0 degを意味しない。

## 6. Logging constants

| item | value |
|---|---:|
| producer rate | 1 kHz |
| period | 1 ms |
| record size | 64 B |
| PSRAM max use | 8 MiB |
| PSRAM reserve | 512 KiB |
| SD batch | 8 KiB |

## 7. Partition table

現最小構成ではNVS partitionを持たない。

現在のpartition tableはfactory applicationのみ。

```text
factory, app, factory, 0x10000, 4M
```

Fin zeroについてはpartitionを追加しても解決しない。AS5047Dの1回転絶対角からreboot前のmulti-turn周回数を復元できないため、Fin zeroとunwrap stateはRAM-onlyとし、reboot後に`FinZero`を再実行する。

NVSや専用flight-log flash partitionは、他用途で必要性が確定するまで追加しない。

## 8. Launcher angle / display attitude

射角は水平から70度とする。

ただし現Control仕様は「Control開始時点をroll偏差0」とするため、射角70度をControl計算へ使用しない。

将来3D姿勢表示を追加する場合は、表示用estimatorをControlと分離し、固定launcher elevation 70度を初期条件として使う。表示用姿勢推定の失敗をControl/Paraへ波及させない。
