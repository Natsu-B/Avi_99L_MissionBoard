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

本飛行ではSpica `f7c477bff52c5a404ba25f7adbbe92aac68c819a`で記録されたfin装着characterizationの換算値をZeroHold固定値として採用する。Spica artifact上の`production_selectable=false`は元解析時点のstatusだが、本Mission firmwareでは運用判断によりflight-fixedとして扱う。

制御構造はSpica revision 3のselected variantに合わせ、angle dead-zoneなし、rate continuous dead-zone 1.0 deg/s、requested-torque dead-zone/hysteresisなしとする。authorityはfin装着実機試験で直接使用した`±600 command`まで戻し、既存TorqueMapper換算ではrequested torque `±1.369544677734375 N m`相当とする。

| item | value | status |
|---|---:|---|
| Kp | 65.390941574 N m/rad | 本飛行固定 |
| Kd | 3.269547079 N m/(rad/s) | 本飛行固定 |
| rate continuous dead-zone | 1.0 deg/s | 本飛行固定 |
| characterization command limit | ±600 | 実機試験済み範囲 |
| N m per command | 0.0022825744628906255 | TorqueMapper換算 |
| requested torque limit equivalent | 1.369544677734375 N m | ±600 command換算 |
| requested torque conditioner | none | 本飛行固定 |
| motor resistance | 3.48 ohm | TorqueMapper固定値 |
| torque constant | 0.00855 N m/A | TorqueMapper固定値 |
| speed constant | 1120 rpm/V | TorqueMapper固定値 |
| gear ratio | 176.175 | 機構固定値 |
| drivetrain efficiency | 0.60 | TorqueMapper換算仮定 |
| bus voltage | 9.0 V | fin装着換算条件 |
| max current | 2.2 A | TB67 hardware setting |
| PWM max duty | 1.0 | software上限 |
| positive torque polarity | IN1 | 実機方向確認対象 |

`gear ratio = 176.175`はencoder/motor側角度をFin出力軸角へ変換するために使用する。ZeroHold/RollControlへ渡すFin angle/rateはgear ratio変換後の値とする。

Spicaのfin装着characterizationはcommand-domainの応答を実測しており、上表Kp/Kdおよびrequested torque limitのN m換算には既存TorqueMapperを使用している。actual motor current、actual shaft torque、Vbusはそのcaptureで直接計測していない。このため「±600 commandまで実機で使用した」ことと「±1.369544677734375 N mを実測した」ことは同義ではない。runtime command/NVSから値を変更する仕組みは設けない。

AS5047D angleまたはFin rateがinvalidなtickではZeroHold要求torqueを生成せず、motorをHi-Zとする。従来のようにrate invalidを0 rad/sへ置換してP項だけで保持しない。

±15 degの外向きcommand禁止はZeroHold authority変更後も維持する。

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
