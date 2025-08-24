#include "dadlib.h"
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "sdkconfig.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#define CONVST_PIN GPIO_NUM_4 // GPIO4 - Start conversion
#define BUSY_PIN GPIO_NUM_27  // GPIO27 - Conversion status
#define CS_PIN GPIO_NUM_15    // GPIO15 - Chip select
#define RESET_PIN GPIO_NUM_25 // GPIO25 - Hardware reset

#define SCK_PIN GPIO_NUM_5   // GPIO5 - SCK
#define MISO_PIN GPIO_NUM_21 // GPIO21 - MISO
#define MOSI_PIN GPIO_NUM_19 // GPIO19 - MOSI (unused)

#define NUM_SAMPLES 2000
static const double LSB_10V = 20.0 / 262144.0;
static const double V_FS_240Volt = LSB_10V * 509.9576;
static const int32_t OFFSET[8] = {10,1,-1,5,-2,3,-6,-1};

#define VOLTAGE_CH 1

static uint8_t rawBytes[18];
static int32_t adcData[NUM_SAMPLES][8];
static double calcAvg[8];
static double calcRms[8];
static double calcPwr[8];

// dadlib config
dadlib_config_t config = {
    .wifi_ssid = "PRINTER24",
    .wifi_pass = "herein3016830168.",

    .mqtt_user = "power",
    .mqtt_pass = "123456",
    .mqtt_broker_url = "mqtts://192.168.1.61:8885",
    .mqtt_topic = "esp32/energymon",
};

static void setup_pin_input(int pin)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

static void setup_pin_output(int pin)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

static void calcRmsAndPower(int ch, int startSample, int endSample)
{
    int64_t sum = 0;
    uint64_t sumRms = 0;
    int64_t sumPwr = 0;
    for (int i = startSample; i < endSample; i++)
    {
        int32_t val = adcData[i][ch];
        sum += val;
        sumRms += val * val;
        sumPwr += val * adcData[i][VOLTAGE_CH];
    }
    uint32_t numSamples = endSample - startSample;
    calcAvg[ch] = ((double)sum) / ((double)numSamples);
    calcRms[ch] = sqrt(((double)sumRms) / ((double)numSamples));
    calcPwr[ch] = ((double)sumPwr) / ((double)numSamples);
}

static bool waitForBusy(int state, uint32_t timeoutMicros)
{
    int64_t then = esp_timer_get_time();
    while (gpio_get_level(BUSY_PIN) != state)
    {
        if (esp_timer_get_time() - then > timeoutMicros)
        {
            return false;
        }
    }
    return true;
}

static void doRead(spi_device_handle_t spi)
{
    if (!waitForBusy(0, 10 * 1000))
        printf("low wait timeout\n");

    gpio_set_level(CONVST_PIN, 0);
    uint32_t then = esp_cpu_get_cycle_count();
    while (esp_cpu_get_cycle_count() - then < 10);
    gpio_set_level(CONVST_PIN, 1);

    if (!waitForBusy(1, 10))
        printf("high wait timeout\n");

    gpio_set_level(CS_PIN, 0);
    spi_transaction_t t = {
        .length = 18 * 8,
        .tx_buffer = NULL,
        .rx_buffer = rawBytes,
        .flags = 0};
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t));
    gpio_set_level(CS_PIN, 1);
}

static void readAllChannels(spi_device_handle_t spi, int32_t *data)
{
    doRead(spi);
    for (int ch = 0; ch < 8; ch++)
    {
        int startBit = ch * 18;
        int startByte = startBit / 8;
        uint32_t buf = 0;
        for (int i = 0; i < 3; i++)
        {
            buf = (buf << 8) | rawBytes[startByte + i];
        }

        int shift = ((startByte + 3) * 8) - (startBit + 18);
        uint32_t val = (uint32_t)((buf >> shift) & 0x3FFFF);

        if (val & 0x20000)
        {
            val |= 0xFFFC0000;
        }
        data[ch] = val - OFFSET[ch];
    }
}

static bool findEndpoints(int *outStartSample, int *outEndSample)
{
    int startSample = 1;
    bool sign = adcData[0][VOLTAGE_CH] >= 0;
    while (startSample < NUM_SAMPLES && ((adcData[startSample][VOLTAGE_CH] >= 0) == sign))
        startSample++;

    if (startSample == NUM_SAMPLES) return false;

    int halfPeriods = 0;
    int endSample = startSample;
    bool endSign = adcData[endSample][VOLTAGE_CH] >= 0;
    for (int currentSample = startSample + 1; currentSample < NUM_SAMPLES; currentSample++)
    {
        bool currentSign = adcData[currentSample][VOLTAGE_CH] >= 0;
        if (currentSign != endSign)
        {
            endSample = currentSample;
            endSign = currentSign;
            halfPeriods++;
        }
    }
    if (!halfPeriods) return false;
    *outStartSample = startSample;
    *outEndSample = endSample;
    return true;
}

static void publishResults()
{
    char buf[512];
    int ret = snprintf(buf, sizeof(buf),
        "{ \"RMS0_volts\": %.6lf, \"RMS1\": %.6lf, \"RMS2\": %.6lf, \"RMS3\": %.6lf,"
        "  \"RMS4\": %.6lf, \"RMS5\": %.6lf, \"RMS6\": %.6lf, \"RMS7\": %.6lf,"
        "  \"PWR1\": %.6lf }",
        // calcRms[0] * V_FS_240Volt,
        calcRms[0] * LSB_10V,
        calcRms[1] * LSB_10V,
        calcRms[2] * LSB_10V,
        calcRms[3] * LSB_10V,
        calcRms[4] * LSB_10V,
        calcRms[5] * LSB_10V,
        calcRms[6] * LSB_10V,
        calcRms[7] * LSB_10V,
        calcPwr[1] * V_FS_240Volt * LSB_10V);

    if (ret < 0 || ret >= sizeof(buf)) {
        dadlib_panic("MQTT JSON buffer too small");
    } else {
        dadlib_mqtt_publish(buf, 0, 1);
    }
}

static void loop(spi_device_handle_t spi)
{
    doRead(spi);
    for (int c = 0; c < NUM_SAMPLES; c++)
        readAllChannels(spi, adcData[c]);

    int startSample, endSample;
    if (!findEndpoints(&startSample, &endSample))
        return;

    for (int ch = 0; ch < 8; ch++)
        calcRmsAndPower(ch, startSample, endSample);

    publishResults();  // 🔹 Send to MQTT
}

void app_main()
{
    printf("Starting energy monitor...\n");

    // init dadlib
    dadlib_init(&config);

    setup_pin_output(CONVST_PIN);
    setup_pin_input(BUSY_PIN);
    setup_pin_output(CS_PIN);
        // setup_pin_input(RESET_PIN);
    setup_pin_output(RESET_PIN);

    gpio_set_level(CONVST_PIN, 1);
    gpio_set_level(CS_PIN, 1);
    gpio_set_level(RESET_PIN, 0);

    gpio_set_level(RESET_PIN, 1);
    esp_rom_delay_us(1);
    gpio_set_level(RESET_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(1));

    spi_bus_config_t buscfg = {
        .mosi_io_num = MOSI_PIN,
        .miso_io_num = MISO_PIN,
        .sclk_io_num = SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 30 * 1000 * 1000,
        .mode = 1,
        .spics_io_num = -1,
        .queue_size = 1};
    spi_device_handle_t spi;
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &spi));

    while (true)
    {
        loop(spi);
        vTaskDelay(pdMS_TO_TICKS(2000)); // send every 2s
    }
}
// #include "dadlib.h"

// #include <stdio.h>

// void app_main()
// {
//     dadlib_config_t config = {
//         .wifi_ssid = "PRINTER24",
//         .wifi_pass = "herein3016830168.",

//         .mqtt_user = "power",
//         .mqtt_pass = "123456",
//         .mqtt_broker_url = "mqtts://192.168.1.61:8885", // SSL connection to VerneMQ
//         .mqtt_topic = "esp32/energymon",
//     };
//     dadlib_init(&config);

//     // demo for dad
//     double c1 = 32049843.43;
//     double c2 = 123456789.0123456789;
//     double c3 = 987654321.0987654321;

//     // char buf[256];
//     // int ret = snprintf(buf, sizeof(buf), "{ "
//     //     "\"dad_coolness_1\": %lf,"
//     //     "\"dad_coolness_2\": %lf,"
//     //     "\"dad_coolness_3\": %lf }",
//     //     c1, c2, c3);

//     // if (ret >= sizeof(buf) - 1) {
//     //     dadlib_panic("dad_coolness json buffer too small");
//     // } else if (ret < 0) {
//     //     dadlib_panic("snprintf failed");
//     // }

//     // dadlib_mqtt_publish(buf, 0, 1);
// }
//     // if (ret >= sizeof(buf) - 1) {
//     //     dadlib_panic("dad_coolness json buffer too small");
//     // } else if (ret < 0) {
//     //     dadlib_panic("snprintf failed");
//     // }

//     // dadlib_mqtt_publish(buf, 0, 1);
// }




