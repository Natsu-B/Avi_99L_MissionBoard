# Logging and Storage

## 1. 目的

flight log保存はMission/Safety/Para/Controlをblockしてはならない。

SDへ直接writeせず、flight producerからPSRAMへ先に退避する。

## 2. Data flow

```text
1 kHz producer
    |
    v
PSRAM ring buffer
    |
    v
low priority SdWriter
    |
    v
SD card
```

producerはSD mount、filesystem、write latencyを待たない。

## 3. PSRAM

対象ESP32-S3は8 MiB PSRAMを搭載する。

flight loggerは:

- 最大8 MiBを利用可能
- 他用途向けに最低512 KiBを残す
- 実際にはその時点で取得可能な連続PSRAM領域に合わせてcapacityを決める

現LogRecordは64 bytes固定。

約7.5 MiB確保できた場合:

```text
7.5 MiB / 64 B = 122880 records
```

1 kHzでは約122.9秒分を保持できる。

## 4. Log rate

flight log producerは1 kHz。

`StartSequence`だけでなく、実装上flight log開始タイミングを明確にし、少なくとも離床以降の全flight eventを欠落させない。

現実装ではgenerationをrecordへ保存する。

## 5. LogRecord v1

固定長64 bytes。

主な内容:

- magic
- schema version
- record size
- monotonic timestamp
- flight elapsed
- flight generation
- それ以前のdrop count
- Mission phase
- flags
- Fin mode
- Para mode
- acceleration x/y/z [mg]
- gyro x/y/z [0.1 deg/s]
- Fin angle [0.01 deg]
- Fin rate [0.01 deg/s]
- LPS pressure [Pa]
- Para angle [0.1 deg]
- CRC16

schema変更時はrecord size/versionを更新し、既存recordを暗黙に別形式として読まない。

## 6. SD writer

SD writeは低優先度taskだけが行う。

- batch target: 8 KiB
- full batchを優先してwrite
- flight終了/flush時だけpartial batchを許可
- SDへの完全write成功後だけPSRAM read indexを進める

write中にstallしてもproducerが同じ未書込slotを再利用しない。

## 7. SD unavailable

以下はflight faultへ昇格させない。

- SD未挿入
- mount failure
- file open failure
- write stall
- 一時write failure

SD mount/openは低優先度taskから再試行してよい。

PSRAMに空きがある限りflight producerはappendを継続する。

## 8. PSRAM full

PSRAM ringが満杯の場合、古い未書込recordを上書きしない。

新しいrecordをdropし、drop counterを増やす。

理由:

- 過去のflight sequenceを無断で破壊しない
- SD stall中にproducer/consumer indexを競合させない
- データ欠落を明示的に観測できる

## 9. Flight criticality

logging故障は以下へ影響させない。

- StartSequence
- 離床検知
- LiftoffDetectionEmergencyStop
- +17秒deployment
- +25秒cutoff
- Fin ZeroHold
- RollControl

logging taskからMission stateを変更する経路を作らない。

## 10. NVSとの関係

flight log用途にNVSを使用しない。

PSRAMはpower lossで消えるため、完全電源断直前の未flush recordは失われ得る。これはSafety動作より低優先度とする。

飛行中reset復元を将来追加する場合も、log persistenceとMission recovery stateを同じNVS blobへ混在させない。
