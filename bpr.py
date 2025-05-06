# main.py ― RPR-0521RS / BPR0521  indoor-light sample  (functional style)
#
#   Gain ×2・積分時間 100 ms（暗い室内〜一般室内照明向け）
#
#   元コード: ESP-IDF v5 new-style I²C driver 版
#   置換点:
#     • FreeRTOS → 無し（while に time.sleep_ms）
#     • ESP_ERROR_CHECK → 例外処理
#     • i2c_master_* API → machine.I2C
#
# ────────────────────────────────────────────────
from machine import I2C, Pin
import time
import sys

# ==== ユーザ設定 ================================================
PIN_SDA       = 21
PIN_SCL       = 22
I2C_FREQ_HZ   = 400_000          # 400 kHz  (MicroPython 上限はビルド依存)

# ==== センサ定義 (RPR-0521RS) ===================================
BPR0521_ADDR       = 0x38            # 7-bit
REG_SYSTEM_CONTROL = 0x40
REG_MODE_CONTROL   = 0x41
REG_ALS_PS_CONTROL = 0x42
REG_PS_DATA_LSB    = 0x44
REG_ALS_DATA0_LSB  = 0x46
REG_ALS_DATA1_LSB  = 0x48
REG_MANUFACT_ID    = 0x92            # 0xE0 expected

# 屋内向け設定 : Gain ×2 (bits=0b01), ALS時間 100 ms (bits=0b110)
ALS_GAIN_BITS      = 0x05            # ×2
MODE_CONTROL_BITS  = 0xC6            # ALS+PS enable & 100 ms


# ==== I²C ラッパ ===============================================
def i2c_wr(i2c: I2C, reg: int, val: int) -> None:
    """1 byte レジスタ書き込み"""
    i2c.writeto(BPR0521_ADDR, bytes((reg, val)))

def i2c_rd(i2c: I2C, reg: int, nbytes: int) -> bytes:
    """Repeated-START で連続読み出し"""
    # stop=False で STOP 条件を出さずにリスタート
    i2c.writeto(BPR0521_ADDR, bytes((reg,)), stop=False)
    return i2c.readfrom(BPR0521_ADDR, nbytes)


# ==== Lux 変換（ROHM 推奨近似式：DS p.22） ======================
def lux_from_raw(d0: int, d1: int) -> float:
    f0 = d0 / 2.0      # Gain ×2 → /2
    f1 = d1 / 2.0
    if f0 < 1e-3:
        return 0.0

    r = f1 / f0
    if r < 0.595:
        return 1.682 * f0 - 1.877 * f1
    elif r < 1.015:
        return 0.644 * f0 - 0.132 * f1
    elif r < 1.352:
        return 0.756 * f0 - 0.243 * f1
    elif r < 3.053:
        return 0.766 * f0 - 0.250 * f1
    else:
        return 0.0


# ==== I²C / センサ初期化 =======================================
def sensor_init(i2c: I2C) -> None:
    """製造 ID 確認 & 屋内モード設定"""
    manuf_id = i2c_rd(i2c, REG_MANUFACT_ID, 1)[0]
    if manuf_id != 0xE0:
        raise RuntimeError("BPR0521: wrong ID (0x{:02X})".format(manuf_id))

    # 屋内設定を書き込み
    i2c_wr(i2c, REG_ALS_PS_CONTROL, ALS_GAIN_BITS)
    i2c_wr(i2c, REG_MODE_CONTROL,   MODE_CONTROL_BITS)


# ==== メイン ====================================================
def main() -> None:
    # I²C バス生成
    i2c = I2C(
        0,
        scl=Pin(PIN_SCL, Pin.OPEN_DRAIN, value=1),
        sda=Pin(PIN_SDA, Pin.OPEN_DRAIN, value=1),
        freq=I2C_FREQ_HZ,
    )

    # デバイス検出
    if BPR0521_ADDR not in i2c.scan():
        print("BPR0521 が見つかりません (配線 / アドレス確認)", file=sys.stderr)
        sys.exit(1)

    try:
        sensor_init(i2c)
    except Exception as e:
        print("初期化失敗:", e, file=sys.stderr)
        sys.exit(1)

    print("BPR0521 initialised -- start loop (1 s)")
    while True:
        # 6 byte 連続読み取り: [PS_L, PS_H, ALS0_L, ALS0_H, ALS1_L, ALS1_H]
        buf = i2c_rd(i2c, REG_PS_DATA_LSB, 6)

        als0 = (buf[3] << 8) | buf[2]
        als1 = (buf[5] << 8) | buf[4]
        lux  = lux_from_raw(als0, als1)

        print("ALS0:{:5d}  ALS1:{:5d}  → {:7.2f} lx".format(als0, als1, lux))
        time.sleep_ms(1000)


# ==== エントリポイント =========================================
if __name__ == "__main__":
    main()
