# main.py – Functional (no-class) port of the ESP-IDF SCD41 example
from machine import I2C, Pin
import time
import sys

# ──────────────────────────
# ハードウェア設定
# ──────────────────────────
I2C_ID        = 0          # ESP32 の I2C バス番号
I2C_SCL_PIN   = 33         # SCL → GPIO33
I2C_SDA_PIN   = 32         # SDA → GPIO32
I2C_FREQ      = 100_000    # 100 kHz

SCD41_ADDR    = 0x62       # 7-bit I²C アドレス

# Sensirion CRC-8 定数
CRC8_POLYNOMIAL = 0x31
CRC8_INIT       = 0xFF

# ──────────────────────────
# CRC-8 (Sensirion) ルーチン
# ──────────────────────────
def crc8(data: bytes) -> int:
    crc = CRC8_INIT
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ CRC8_POLYNOMIAL) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc

def crc_ok(buf: bytes) -> bool:
    """バッファ内の各 2-byte ワード + CRC を確認 (word|crc, …)"""
    for i in range(0, len(buf), 3):
        if crc8(buf[i:i+2]) != buf[i+2]:
            return False
    return True

# ──────────────────────────
# 低レベル I²C 操作
# ──────────────────────────
def cmd(i2c: I2C, addr: int, value: int) -> None:
    """16-bit コマンド送信（MSB→LSB）"""
    i2c.writeto(addr, value.to_bytes(2, "big"))

def cmd_read(i2c: I2C, addr: int, value: int, nbytes: int, delay_ms: int = 1) -> bytes:
    """
    コマンド送信後に Repeated-START で読み込みを行う。
    stop=False でストップ条件を抑制し、START→CMD→REP-START→READ を実現。
    """
    buf = bytearray(nbytes)
    i2c.writeto(addr, value.to_bytes(2, "big"), stop=False)
    if delay_ms:
        time.sleep_ms(delay_ms)
    i2c.readfrom_into(addr, buf)
    return bytes(buf)

# ──────────────────────────
# SCD41 基本コマンド
# ──────────────────────────
def stop_periodic_measurements(i2c, addr):
    cmd(i2c, addr, 0x3F86)
    time.sleep_ms(1)

def start_periodic_measurements(i2c, addr):
    cmd(i2c, addr, 0x21B1)
    # 初回データは ≤5 s で準備完了（データシートより）
    time.sleep_ms(10)

def get_serial_number(i2c, addr) -> int:
    raw = cmd_read(i2c, addr, 0x3682, 9)
    if not crc_ok(raw):
        raise ValueError("CRC エラー（シリアル）")
    w1 = int.from_bytes(raw[0:2], "big")
    w2 = int.from_bytes(raw[3:5], "big")
    w3 = int.from_bytes(raw[6:8], "big")
    return (w1 << 32) | (w2 << 16) | w3

def data_ready(i2c, addr) -> bool:
    raw = cmd_read(i2c, addr, 0xE4B8, 3)
    if not crc_ok(raw):
        return False
    status = int.from_bytes(raw[0:2], "big")
    return bool(status & 0x07FF)

def read_measurement(i2c, addr):
    raw = cmd_read(i2c, addr, 0xEC05, 9)
    if not crc_ok(raw):
        raise ValueError("CRC エラー（測定値）")

    co2      = int.from_bytes(raw[0:2], "big")
    raw_temp = int.from_bytes(raw[3:5], "big")
    raw_hum  = int.from_bytes(raw[6:8], "big")

    temperature = -45 + 175 * (raw_temp / 65535.0)
    humidity    = 100 * (raw_hum  / 65535.0)

    return co2, temperature, humidity

# ──────────────────────────
# アプリケーション
# ──────────────────────────
def main() -> None:
    i2c = I2C(
        I2C_ID,
        scl=Pin(I2C_SCL_PIN),
        sda=Pin(I2C_SDA_PIN),
        freq=I2C_FREQ,
    )

    if SCD41_ADDR not in i2c.scan():
        print("SCD41 が検出できません")
        sys.exit(1)

    # C 版と同じ初期化シーケンス
    stop_periodic_measurements(i2c, SCD41_ADDR)
    time.sleep_ms(500)

    serial = get_serial_number(i2c, SCD41_ADDR)
    print("SCD41 Serial:", hex(serial))

    time.sleep_ms(500)
    start_periodic_measurements(i2c, SCD41_ADDR)
    time.sleep_ms(500)

    print("計測ループ開始（1 s 周期）")
    while True:
        if data_ready(i2c, SCD41_ADDR):
            co2, temp, rh = read_measurement(i2c, SCD41_ADDR)
            print(
                "CO₂: {:5d} ppm | 温度: {:6.2f} °C | 相対湿度: {:6.2f} %RH".format(
                    co2, temp, rh
                )
            )
        else:
            # データ未準備の場合はドットを出力
            print(".", end="")
        time.sleep(1)

# ──────────────────────────
# エントリポイント
# ──────────────────────────
if __name__ == "__main__":
    main()
