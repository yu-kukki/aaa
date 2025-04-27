/*  main/bpr0521_indoor.c  ----------------------------------------
 *  ROHM RPR-0521RS ― ALS/PS センサ（I²C）屋内用サンプル
 *  Gain ×2・積分時間 100 ms ＝ 暗い室内～一般室内照明に最適
 *
 *  ※ ESP-IDF v5 “new-style I2C driver” 使用
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"     // ← 新 API
#include "esp_log.h"
#include "esp_err.h"

/* ==== ユーザ設定ここだけ ====================================== */
#define PIN_SDA       21
#define PIN_SCL       22
#define I2C_HZ        400000           // 400 kHz

/* ==== センサ定義 ============================================== */
#define BPR0521_ADDR           0x38    // 7-bit
#define REG_SYSTEM_CONTROL     0x40
#define REG_MODE_CONTROL       0x41
#define REG_ALS_PS_CONTROL     0x42
#define REG_PS_DATA_LSB        0x44
#define REG_ALS_DATA0_LSB      0x46
#define REG_ALS_DATA1_LSB      0x48
#define REG_MANUFACT_ID        0x92

/* 屋内向け設定 : Gain ×2 (D0/D1=01b) , ALS時間 100 ms (bits=0x06) */
#define ALS_GAIN_BITS          0x05    // ×2
#define MODE_CONTROL_BITS      0xC6    // ALS&PS enable + 100 ms   :contentReference[oaicite:0]{index=0}

/* ==== I²C バス／デバイスハンドル =============================== */
static i2c_master_bus_handle_t  i2c_bus  = NULL;
static i2c_master_dev_handle_t  bpr_dev  = NULL;

/* ==== I²C ラッパ ============================================= */
static esp_err_t i2c_wr(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(bpr_dev, buf, sizeof(buf),
                               pdMS_TO_TICKS(100));
}

static esp_err_t i2c_rd(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(bpr_dev,
                                       &reg, 1,
                                       data, len,
                                       pdMS_TO_TICKS(100));
}

/* ==== Lux 変換（ROHM 近似式：データシート p.22） =============== */
static float lux_from_raw(uint16_t d0, uint16_t d1)
{
    float f0 = d0 / 2.0f;   // Gain ×2 → /2
    float f1 = d1 / 2.0f;
    if (f0 < 1e-3f) return 0;

    float r = f1 / f0;
    if (r < 0.595f)       return 1.682f * f0 - 1.877f * f1;
    else if (r < 1.015f)  return 0.644f * f0 - 0.132f * f1;
    else if (r < 1.352f)  return 0.756f * f0 - 0.243f * f1;
    else if (r < 3.053f)  return 0.766f * f0 - 0.250f * f1;
    else                  return 0;
}

/* ==== I²C / センサ初期化 ====================================== */
static void i2c_init(void)
{
    /* 1) バス生成 ------------------------------------------------ */
    i2c_master_bus_config_t bus_cfg = {
        .clk_source           = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt    = 7,
        .i2c_port             = 0,
        .scl_io_num           = PIN_SCL,
        .sda_io_num           = PIN_SDA,
        .flags.enable_internal_pullup = true
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    /* 2) デバイス検出 ------------------------------------------ */
    ESP_ERROR_CHECK(i2c_master_probe(i2c_bus, BPR0521_ADDR, pdMS_TO_TICKS(100)));

    /* 3) デバイスハンドル作成 ---------------------------------- */
    const i2c_device_config_t dev_cfg = {
        .device_address   = BPR0521_ADDR,
        .dev_addr_length  = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz     = I2C_HZ
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg, &bpr_dev));
}

static esp_err_t sensor_init(void)
{
    /* 製造IDチェック（0xE0 想定）*/
    uint8_t id;
    ESP_ERROR_CHECK(i2c_rd(REG_MANUFACT_ID, &id, 1));
    if (id != 0xE0) return ESP_ERR_INVALID_RESPONSE;

    ESP_ERROR_CHECK(i2c_wr(REG_ALS_PS_CONTROL, ALS_GAIN_BITS));
    ESP_ERROR_CHECK(i2c_wr(REG_MODE_CONTROL,  MODE_CONTROL_BITS));
    return ESP_OK;
}

/* ==== メイン =================================================== */
void app_main(void)
{
    i2c_init();
    if (sensor_init() != ESP_OK) {
        ESP_LOGE("BPR0521", "sensor init failed");
        vTaskDelay(portMAX_DELAY);
    }

    uint8_t buf[6];
    while (true) {
        ESP_ERROR_CHECK(i2c_rd(REG_PS_DATA_LSB, buf, sizeof(buf)));

        uint16_t als0 = (buf[3] << 8) | buf[2];
        uint16_t als1 = (buf[5] << 8) | buf[4];
        float lux = lux_from_raw(als0, als1);

        printf("ALS0:%5u  ALS1:%5u  → %7.2f lx\n", als0, als1, lux);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
/* -------------------------------------------------------------- */
