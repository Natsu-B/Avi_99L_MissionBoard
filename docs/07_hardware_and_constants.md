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

現実装は旧候補値を1 motor用へ流用している。

| item | value | status |
|---|---:|---|
| Kp | 2.32 N m/rad | TODO(HW_TEST) |
| Kd | 0.296 N m/(rad/s) | TODO(HW_TEST) |
| torque limit | 0.80 N m | TODO(HW_TEST) |
| motor resistance | 3.48 ohm | TODO(HW_TEST) |
| torque constant | 0.00855 N m/A | TODO(HW_TEST) |
| speed constant | 1120 rpm/V | TODO(HW_TEST) |
| gear ratio | 176.175 | 機構確認 |
| drivetrain efficiency | 0.60 | 暫定 |
| bus voltage | 9.0 V | 暫定 |
| max current | 2.0 A | TODO(HW_TEST) |
| PWM max duty | 1.0 | TODO(HW_TEST) |
| positive torque polarity | IN1 | TODO(HW_TEST) |

これらをruntime commandやNVSで変更する仕組みは設けない。

実機characterization後にsource constantを更新する。

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

NVSや専用flight-log flash partitionは、必要性が確定するまで追加しない。

## 8. Launcher angle / display attitude

射角は水平から70度とする。

ただし現Control仕様は「Control開始時点をroll偏差0」とするため、射角70度をControl計算へ使用しない。

将来3D姿勢表示を追加する場合は、表示用estimatorをControlと分離し、固定launcher elevation 70度を初期条件として使う。表示用姿勢推定の失敗をControl/Paraへ波及させない。
