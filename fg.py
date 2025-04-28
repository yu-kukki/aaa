# main.py  – MicroPython port of the ESP-IDF SCD41 example
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


# ──────────────────────────
# SCD41 ドライバクラス
# ──────────────────────────
class SCD41:
    def __init__(self, i2c: I2C, addr=SCD41_ADDR):
        self.i2c  = i2c
        self.addr = addr

        if self.addr not in self.i2c.scan():
            raise OSError("SCD41 が検出できません")

    # ――― 低レベル I²C ―――
    def _cmd(self, cmd: int) -> None:
        """16 bit コマンド送信（MSB→LSB）"""
        self.i2c.writeto(self.addr, cmd.to_bytes(2, "big"))

    def _cmd_read(self, cmd: int, nbytes: int, delay_ms: int = 1) -> bytes:
        """
        Repeated-START でコマンド送信後に読み込みを行う。
        stop=False によってストップ条件を抑制し、START→CMD→REPEATED-START→READ を実現。
        """
        buf = bytearray(nbytes)
        self.i2c.writeto(self.addr, cmd.to_bytes(2, "big"), stop=False)
        if delay_ms:
            time.sleep_ms(delay_ms)
        self.i2c.readfrom_into(self.addr, buf)
        return bytes(buf)

    # ――― 基本コマンド ―――
    def stop_periodic_measurements(self):
        self._cmd(0x3F86)
        time.sleep_ms(1)

    def start_periodic_measurements(self):
        self._cmd(0x21B1)
        # 初回データは ≤5 s で準備完了（データシートより）
        time.sleep_ms(10)

    def get_serial_number(self) -> int:
        raw = self._cmd_read(0x3682, 9)
        if not self._crc_ok(raw):
            raise ValueError("CRC エラー（シリアル）」")
        w1 = int.from_bytes(raw[0:2], "big")
        w2 = int.from_bytes(raw[3:5], "big")
        w3 = int.from_bytes(raw[6:8], "big")
        return (w1 << 32) | (w2 << 16) | w3

    def _data_ready(self) -> bool:
        raw = self._cmd_read(0xE4B8, 3)
        if not self._crc_ok(raw):
            return False
        status = int.from_bytes(raw[0:2], "big")
        return bool(status & 0x07FF)

    def read_measurement(self):
        raw = self._cmd_read(0xEC05, 9)
        if not self._crc_ok(raw):
            raise ValueError("CRC エラー（測定値）」")

        co2       = int.from_bytes(raw[0:2], "big")
        raw_temp  = int.from_bytes(raw[3:5], "big")
        raw_hum   = int.from_bytes(raw[6:8], "big")

        temperature = -45 + 175 * (raw_temp / 65535.0)
        humidity    = 100 * (raw_hum  / 65535.0)

        return co2, temperature, humidity

    # ――― CRC 検証 ―――
    def _crc_ok(self, buf: bytes) -> bool:
        for i in range(0, len(buf), 3):
            if crc8(buf[i:i+2]) != buf[i+2]:
                return False
        return True


# ──────────────────────────
# メインアプリケーション
# ──────────────────────────
def main() -> None:
    i2c = I2C(
        I2C_ID,
        scl=Pin(I2C_SCL_PIN),
        sda=Pin(I2C_SDA_PIN),
        freq=I2C_FREQ,
    )

    try:
        scd41 = SCD41(i2c)
    except OSError as e:
        print(e)
        sys.exit(1)

    # 初期化シーケンス（オリジナル C と同一）
    scd41.stop_periodic_measurements()
    time.sleep_ms(500)

    serial = scd41.get_serial_number()
    print("SCD41 Serial:", hex(serial))

    time.sleep_ms(500)
    scd41.start_periodic_measurements()
    time.sleep_ms(500)

    print("計測ループ開始（1 s 周期）")
    while True:
        if scd41._data_ready():
            co2, temp, rh = scd41.read_measurement()
            print(
                "CO₂: {:5d} ppm | 温度: {:6.2f} °C | 相対湿度: {:6.2f} %RH".format(
                    co2, temp, rh
                )
            )
        else:
            # データ未準備の場合はドットを出力
            print(".", end="")
        time.sleep(1)


if __name__ == "__main__":
    main()
