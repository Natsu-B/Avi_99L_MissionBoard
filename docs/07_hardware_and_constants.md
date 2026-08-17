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

- PWM: ZeroHold 20 kHz / RollControl 30 kHz
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

`feat/zero-hold-nm-pid`で実機成立した20 kHz、10 bit、1 kHzのcommand-domain controllerをbaselineとする。ZeroHoldはこの経路を直接使用し、RollControl用TorqueMapperを通さない。

| item | value | status |
|---|---:|---|
| Kp | 500 command/deg | 実機controller |
| Ki | 35 command/(deg s) | 実機controller |
| Kd | 25 command/(deg/s) | 実機controller |
| integral limit | ±2 deg s | 実機controller |
| velocity LPF tau | 20 ms | 実機controller |
| hold deadband | 0.05 deg / 0.5 deg/s | 実機controller |
| minimum active error | 0.08 deg | 実機controller |
| ZeroHold command limit | ±800 | 実機controller |
| N m per command | 0.0022825744628906255 | telemetry互換換算 |
| minimum active command | 70 | ZeroHold PWM補償 |
| command full scale | 1024 | 10 bit command座標 |
| ZeroHold PWM frequency | 20 kHz | command-domain drive/coast |
| RollControl PWM frequency | 30 kHz | torque mapper drive/brake |
| motor resistance | 3.48 ohm | RollControl TorqueMapper固定値 |
| torque constant | 0.00855 N m/A | RollControl TorqueMapper固定値 |
| speed constant | 1120 rpm/V | RollControl TorqueMapper固定値 |
| gear ratio | 176.175 | 機構固定値 |
| drivetrain efficiency | 0.60 | TorqueMapper換算仮定 |
| bus voltage | 9.0 V | fin装着換算条件 |
| max current | 2.2 A | TB67 hardware setting |
| gearbox continuous speed | 6000 rpm | 超過をtelemetry/limited判定 |
| motor hard speed | 9800 rpm | 同方向加速torqueを禁止 |
| PWM max duty | 1.0 | software上限 |
| positive torque polarity | IN1 | 実機方向確認対象 |

`gear ratio = 176.175`はencoder/motor側角度をFin出力軸角へ変換するために使用する。ZeroHold/RollControlへ渡すFin angle/rateはgear ratio変換後の値とする。

Spicaのfin装着characterizationで直接観測したのはcommandとoutput-equivalent angle/rateである。RollControlのrequested/effective torque、current、drivetrain efficiencyはmapper計算値であり、actual motor currentまたはactual shaft torqueの実測値ではない。runtime command/NVSから値を変更する仕組みは設けない。

AS5047D angle/rateがinvalidまたはstale、dtが非正または上限外のtickではcommandを生成せずmotorをHi-Zとし、PID stateと`zero_hold_achieved`をresetする。積分は常に±2 deg s内へ制限する。

ZeroHold PID、成立時間、encoder unwrapのnon-atomic stateは1 kHz realtime taskだけが更新する。Recovery taskはatomic reset requestとinvalid flagを発行し、motorをcoastしてからtransportを復旧する。realtime側はencoder mutexを待たずに取得できたtickだけsensor readからdrive反映までを完結し、取得できないtickはstateをresetしてdriveを生成しない。FinFree、force-safe、Recovery、Safety cutoffも同じmutexでmotor writerを直列化する。Safety taskはatomic inhibitを先にlatchし、進行中writerの終了後にmotorをcoastするため、古いrealtime snapshotがcutoff後に非0 driveを再開できない。

command-to-countは整数floorを使用し、70 commandを69 count、1024 commandを1023 countへ変換する。ZeroHold出力は20 kHz drive/coast、Roll出力は30 kHz drive/brakeであり、mode切替前に両入力をLOWへ落としてtimer周波数を変更する。Roll gain候補のFIT run FIN0007はdrive/brakeでtopologyは一致する。ただしFIN0009 strict holdoutとFIN0010確認が共に事前固定gateを満たさないため、`PROVISIONAL_BRAKE_FIXED_BY_USER_DIRECTION_VALIDATION_GATE_NOT_MET / NO_GO`、`production_selectable=false`とする。

FIN0010 revision artifact SHA-256は`2eb8ac6f6b94fb99b22417546ef437c557b676c56545e0721e4b15c94dff25b1`、read-once consumed marker SHA-256は`4cedffcdb8f389116bfe10c8a6126ca34e64bff2b6cc04898889b79f73fa09c2`である。

±15 degの外向きcommand禁止は両modeで維持する。RollControlの9800 rpm加速禁止はrequested torque方向で判定し、逆向きbrakingを許可する。

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
