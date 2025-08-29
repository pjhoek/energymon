#include "dadlib.h"
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#define MQTT_OUT_TOPIC "esp32/energymon/measurement"

#define CONVST_PIN GPIO_NUM_4 // GPIO4 - Start conversion
#define BUSY_PIN GPIO_NUM_27  // GPIO27 - Conversion status
#define CS_PIN GPIO_NUM_15    // GPIO15 - Chip select
#define RESET_PIN GPIO_NUM_25 // GPIO25 - Hardware reset

#define SCK_PIN GPIO_NUM_5   // GPIO5 - SCK
#define MISO_PIN GPIO_NUM_21 // GPIO21 - MISO (MASTER IN, SLAVE OUT)
#define MOSI_PIN GPIO_NUM_19 // GPIO19 - MOSI (unused)

#define NUM_SAMPLES 2000

#define VOLTAGE_CH 1

static uint8_t rawBytes[18];
static int32_t adcData[NUM_SAMPLES][8];
static int32_t adcOffset[8]; // offsets for calibration
static double calcAvg[8];
static double calcRms[8];
static double calcPwr[8];

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
            // Timeout
            return false;
        }
    }

    // BUSY is now in state `state`
    return true;
}

static void doRead(spi_device_handle_t spi)
{
    if (!waitForBusy(0, 10 * 1000))
    {
        printf("low wait timeout\n");
    }

    gpio_set_level(CONVST_PIN, 0);
    uint32_t then = esp_cpu_get_cycle_count();
    while (esp_cpu_get_cycle_count() - then < 10)
        ;
    gpio_set_level(CONVST_PIN, 1);

    if (!waitForBusy(1, 10))
    {
        printf("high wait timeout\n");
    }

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
        data[ch] = val - adcOffset[ch];
    }
}

// set true for DC testing, false for AC
#define DC_MODE 1

static bool findEndpoints(int *outStartSample, int *outEndSample)
{
    if (DC_MODE)
    {
        *outStartSample = 0;
        *outEndSample = NUM_SAMPLES - 1;
        return true;
    }

    int startSample = 1;

    bool sign = adcData[0][VOLTAGE_CH] >= 0;

    //  Advance `startSample` to the first sample which differs in sign from the literal zeroth sample (this has to be a zero crossing)
    while (startSample < NUM_SAMPLES && ((adcData[startSample][VOLTAGE_CH] >= 0) == sign))
    {
        startSample++;
    }

    if (startSample == NUM_SAMPLES)
    {
        printf("sample overflow finding zero crossing\n");
        return false;
    }

    int halfPeriods = 0;
    int endSample = startSample;
    bool endSign = adcData[endSample][VOLTAGE_CH] >= 0;

    // Scan over all of the samples after `startSample` until the end
    for (int currentSample = startSample + 1; currentSample < NUM_SAMPLES; currentSample++)
    {
        bool currentSign = adcData[currentSample][VOLTAGE_CH] >= 0;

        // If the current sample now has a different sign to the last saved "end sample", updated
        // the saved "end sample" to this position (we have just found another zero crossing).
        if (currentSign != endSign)
        {
            endSample = currentSample;
            endSign = currentSign;
            halfPeriods++;
        }
    }

    printf("%d\n", halfPeriods);

    if (!halfPeriods)
    {
        printf("didn't find any half periods!?!\n");
        return false;
    }

    // Have we see roughly the expected number of half periods?
    // NOTE: 138 samples/halfperiod is a reasonable estimate for x16 oversampling, and
    // should be suitably multiplied/divided if the oversampling rate is changed.
#define EXPECTED_SAMPLES_PER_HALFPERIOD 136

    if (abs((halfPeriods * EXPECTED_SAMPLES_PER_HALFPERIOD) - (endSample - startSample)) > NUM_SAMPLES / 200)
    {
        printf("half periods and range disagree! halfPeriods=%d: %d vs %d - %d = %d\n", halfPeriods,
               halfPeriods * EXPECTED_SAMPLES_PER_HALFPERIOD,
               endSample, startSample, endSample - startSample);
        return false;
    }

    *outStartSample = startSample;
    *outEndSample = endSample;
    return true;
}

static void publishResults()
{
    char buf[512];
    int ret = snprintf(buf, sizeof(buf), "{\"primaryCh\":%d,"
                                         "\"avg\":[%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf],"
                                         "\"rms\":[%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf],"
                                         "\"pwr\":[%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf]}",
                       VOLTAGE_CH,
                       calcAvg[0], calcAvg[1], calcAvg[2], calcAvg[3], calcAvg[4], calcAvg[5], calcAvg[6], calcAvg[7],
                       calcRms[0], calcRms[1], calcRms[2], calcRms[3], calcRms[4], calcRms[5], calcRms[6], calcRms[7],
                       calcPwr[0], calcPwr[1], calcPwr[2], calcPwr[3], calcPwr[4], calcPwr[5], calcPwr[6], calcPwr[7]);

    if (ret < 0 || ret >= sizeof(buf))
    {
        dadlib_panic("MQTT JSON buffer too small");
    }

    if (dadlib_mqtt_publish(MQTT_OUT_TOPIC, buf, 0, 1) < 0)
    {
        ESP_LOGW("app", "MQTT publish failed");
        printf("%s\n", buf); // Print JSON to terminal every time
    }
}

static void loop(spi_device_handle_t spi)
{
    doRead(spi);
    for (int c = 0; c < NUM_SAMPLES; c++)
    {
        readAllChannels(spi, adcData[c]);
    }

    int startSample, endSample;
    if (!findEndpoints(&startSample, &endSample))
    {
        return;
    }

    for (int ch = 0; ch < 8; ch++)
    {
        calcRmsAndPower(ch, startSample, endSample);
    }

    publishResults(); // Send to MQTT
}

static bool offsets_initialized = false;
static SemaphoreHandle_t offsets_initialized_sem;
static StaticSemaphore_t offsets_initialized_sem_buff;

static void handle_mqtt_set_offset(const char *data, int len) {
    printf("received offset data: %.*s\n", len, data);

    if (offsets_initialized) {
        ESP_LOGE("app", "offsets were already initialized, restarting");
        dadlib_panic("offset fail");
    }

    printf("setting offsets to: ");

    const char *cur = data;
    for (int i = 0; i < 8; i++) {
        char *end;
        long val = strtol(cur, &end, 10);
        if (end == cur) {
            ESP_LOGE("app", "invalid offset data for index %d", i);
            dadlib_panic("offset fail");
        }

        adcOffset[i] = (int32_t) val;
        printf("%ld, ", adcOffset[i]);

        cur = end;
        if (*cur != ',') {
            ESP_LOGE("app", "expected a comma after parsing index %d", i);
            dadlib_panic("offset fail");
        }
        cur++;
    }

    printf("\n");

    offsets_initialized = true;
    xSemaphoreGive(offsets_initialized_sem);
}

void app_main()
{
    printf("Starting energy monitor...\n");
    
    offsets_initialized_sem = xSemaphoreCreateBinaryStatic(&offsets_initialized_sem_buff);

    static dadlib_mqtt_topic_handler_t mqtt_subscribe_topics[] = {
        {
            .topic = "esp32/energymon/offset",
            .handler = handle_mqtt_set_offset,
        },
        { 0 }
    };

    dadlib_config_t config = {
        .wifi_ssid = "PRINTER24",
        .wifi_pass = "herein3016830168.",

        .mqtt_user = "power",
        .mqtt_pass = "123456",
        .mqtt_broker_url = "mqtts://192.168.1.19:8883",
        .mqtt_subscribe_topics = mqtt_subscribe_topics,

        .skip_wait_for_wifi_and_mqtt = false,
    };
    dadlib_init(&config);

    dadlib_setup_pin_output(CONVST_PIN);
    dadlib_setup_pin_input(BUSY_PIN);
    dadlib_setup_pin_output(CS_PIN);
    dadlib_setup_pin_output(RESET_PIN);

    gpio_set_level(CONVST_PIN, 1);
    gpio_set_level(CS_PIN, 1);
    gpio_set_level(RESET_PIN, 0);

    gpio_set_level(RESET_PIN, 1);
    esp_rom_delay_us(1);
    gpio_set_level(RESET_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(500));

    spi_bus_config_t buscfg = {
        .mosi_io_num = MOSI_PIN,
        .miso_io_num = MISO_PIN,
        .sclk_io_num = SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0};
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 30 * 1000 * 1000,
        .mode = 1,
        .spics_io_num = -1,
        .queue_size = 1};
    spi_device_handle_t spi;
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &spi));

    // wait until all adc offsets are initialized
    if (!xSemaphoreTake(offsets_initialized_sem, pdMS_TO_TICKS(5000))) {
        dadlib_panic("took too long to initialize offsets");
    }

    while (true)
    {
        loop(spi);
        vTaskDelay(1);
    }
}