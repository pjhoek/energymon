#include "dadlib.h"
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

static const char *TAG = "energymon";

#define MQTT_OUT_TOPIC "esp32/energymon/measurement"

#define CONVST_PIN GPIO_NUM_4 // GPIO4 - Start conversion
#define BUSY_PIN GPIO_NUM_27  // GPIO27 - Conversion status
#define CS_PIN GPIO_NUM_15    // GPIO15 - Chip select
#define RESET_PIN GPIO_NUM_25 // GPIO25 - Hardware reset

#define SCK_PIN GPIO_NUM_5   // GPIO5 - SCK
#define MISO_PIN GPIO_NUM_21 // GPIO21 - MISO (MASTER IN, SLAVE OUT)
#define MOSI_PIN GPIO_NUM_19 // GPIO19 - MOSI (unused)

#define BLUE_LED GPIO_NUM_32 // GPIO32 - Blue LED
#define RED_LED GPIO_NUM_13 // GPIO13 - Red LED
#define GREEN_LED GPIO_NUM_14 // GPIO14 - Green LED

#define NUM_SAMPLES 2000

static int32_t adcOffset[8]; // offsets for calibration
static bool dcMode; // set true for DC testing, false for AC
static int mainsCurrentCh; // (unimportant) used for blue led net power out/in status
static int mainsVoltageCh; //   (important) used for calculating period of mains to sync periods
                           //               NOTE: we want to be using the voltage for this instead of
                           //               (one of) the current channels, because the voltage is always
                           //               much greater than zero (and the current could sometimes be small).

static SemaphoreHandle_t config_param_lock;
static StaticSemaphore_t config_param_lock_buff;

#define INIT_PARAM_OFFSET  (1 << 0)
#define INIT_PARAM_DC_MODE (1 << 1)
#define INIT_PARAM_MAINS_CURRENT_CH (1 << 2)
#define INIT_PARAM_MAINS_VOLTAGE_CH (1 << 3)
#define INIT_PARAM_ALL (INIT_PARAM_OFFSET | INIT_PARAM_DC_MODE | INIT_PARAM_MAINS_CURRENT_CH | INIT_PARAM_MAINS_VOLTAGE_CH)

static EventGroupHandle_t config_param_is_init;
static StaticEventGroup_t config_param_is_init_buff;

#define ERROR_LED_TIMEOUT_US (5 * 1000 * 1000)

static int64_t lastErrorTimeUs = -ERROR_LED_TIMEOUT_US;
static uint8_t rawBytes[18];
static int32_t adcData[NUM_SAMPLES][8];
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
        sumPwr += val * adcData[i][mainsVoltageCh];
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

static bool findEndpoints(int *outStartSample, int *outEndSample)
{
    if (dcMode)
    {
        *outStartSample = 0;
        *outEndSample = NUM_SAMPLES - 1;
        return true;
    }

    int startSample = 1;

    bool sign = adcData[0][mainsVoltageCh] >= 0;

    //  Advance `startSample` to the first sample which differs in sign from the literal zeroth sample (this has to be a zero crossing)
    while (startSample < NUM_SAMPLES && ((adcData[startSample][mainsVoltageCh] >= 0) == sign))
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
    bool endSign = adcData[endSample][mainsVoltageCh] >= 0;

    // Scan over all of the samples after `startSample` until the end
    for (int currentSample = startSample + 1; currentSample < NUM_SAMPLES; currentSample++)
    {
        bool currentSign = adcData[currentSample][mainsVoltageCh] >= 0;

        // If the current sample now has a different sign to the last saved "end sample", updated
        // the saved "end sample" to this position (we have just found another zero crossing).
        if (currentSign != endSign)
        {
            // Have we see roughly the expected number of half periods?
            // NOTE: 136 samples/halfperiod is a reasonable estimate for x16 oversampling, and
            // should be suitably multiplied/divided if the oversampling rate is changed.
            #define EXPECTED_SAMPLES_PER_HALFPERIOD 136

            if (abs(EXPECTED_SAMPLES_PER_HALFPERIOD - (currentSample - endSample)) > EXPECTED_SAMPLES_PER_HALFPERIOD * 0.03)
            {
                printf("half period has bad length: %d vs %d - %d = %d\n",
                    EXPECTED_SAMPLES_PER_HALFPERIOD,
                    currentSample, endSample, currentSample - endSample);
                return false;
            }

            endSample = currentSample;
            endSign = currentSign;
            halfPeriods++;
        }
    }

    if (!halfPeriods)
    {
        printf("didn't find any half periods!?!\n");
        return false;
    }

    *outStartSample = startSample;
    *outEndSample = endSample;
    return true;
}

static void setLedStatus() {
    gpio_set_level( BLUE_LED, calcPwr[mainsCurrentCh] < -0.5);
    gpio_set_level(  RED_LED, esp_timer_get_time() - lastErrorTimeUs < ERROR_LED_TIMEOUT_US);
    gpio_set_level(GREEN_LED, dadlib_is_connected());
}

static void publishResults()
{
    char buf[512];
    int ret = snprintf(buf, sizeof(buf), "{\"dcMode\":%s,\"mainsVoltageCh\":%d,"
                                         "\"avg\":[%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf],"
                                         "\"rms\":[%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf],"
                                         "\"pwr\":[%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf]}",
                       dcMode ? "true" : "false", mainsVoltageCh,
                       calcAvg[0], calcAvg[1], calcAvg[2], calcAvg[3], calcAvg[4], calcAvg[5], calcAvg[6], calcAvg[7],
                       calcRms[0], calcRms[1], calcRms[2], calcRms[3], calcRms[4], calcRms[5], calcRms[6], calcRms[7],
                       calcPwr[0], calcPwr[1], calcPwr[2], calcPwr[3], calcPwr[4], calcPwr[5], calcPwr[6], calcPwr[7]);

    if (ret < 0 || ret >= sizeof(buf))
    {
        dadlib_panic("MQTT JSON buffer too small");
    }

    if (dadlib_mqtt_publish(MQTT_OUT_TOPIC, buf, 0, 0) < 0)
    {
        ESP_LOGW(TAG, "MQTT publish failed");
        printf("%s\n", buf); // Print JSON to terminal every time
    }
}

#define SCRATCH_BUFF_SIZE 32

static void handle_mqtt_set_offset(const char *data, int len) {
    printf("received offset data: %.*s\n", len, data);

    if (len >= SCRATCH_BUFF_SIZE - 1) {
        dadlib_panic("data too long!");
    }

    char buff[SCRATCH_BUFF_SIZE];
    memcpy(buff, data, len);
    buff[len] = 0;

    printf("setting offsets to: ");

    int32_t newOffsets[8];

    const char *cur = buff;
    for (int i = 0; i < 8; i++) {
        char *end;
        long val = strtol(cur, &end, 10);
        if (end == cur) {
            ESP_LOGE(TAG, "invalid offset data for index %d", i);
            dadlib_panic("offset fail");
        }

        newOffsets[i] = (int32_t) val;
        printf("%ld, ", newOffsets[i]);

        cur = end;
        if (*cur != ',') {
            ESP_LOGE(TAG, "expected a comma after parsing index %d", i);
            dadlib_panic("offset fail");
        }
        cur++;
    }

    printf("\n");

    xSemaphoreTake(config_param_lock, portMAX_DELAY);
    memcpy(adcOffset, newOffsets, sizeof(adcOffset));
    xSemaphoreGive(config_param_lock);

    xEventGroupSetBits(config_param_is_init, INIT_PARAM_OFFSET);
}

static void handle_mqtt_set_dcMode(const char *data, int len) {
    printf("received dcMode data: %.*s\n", len, data);

    if (len != 1) {
        dadlib_panic("dcMode data too long");
    }

    bool newDcMode;
    if (data[0] == '0') {
        newDcMode = false;
    } else if (data[0] == '1') {
        newDcMode = true;
    } else {
        dadlib_panic("invalid dcMode data");
    }

    printf("setting dcMode to: %d\n", newDcMode);

    xSemaphoreTake(config_param_lock, portMAX_DELAY);
    dcMode = newDcMode;
    xSemaphoreGive(config_param_lock);

    xEventGroupSetBits(config_param_is_init, INIT_PARAM_DC_MODE);
}

static void handle_mqtt_set_mainsCurrentCh(const char *data, int len) {
    printf("received mainsCurrentCh data: %.*s\n", len, data);

    if (len >= SCRATCH_BUFF_SIZE - 1) {
        dadlib_panic("data too long!");
    }

    char buff[SCRATCH_BUFF_SIZE];
    memcpy(buff, data, len);
    buff[len] = 0;

    long newCurrentCh = strtol(buff, NULL, 10);

    printf("setting mainsCurrentCh to: %ld\n", newCurrentCh);

    xSemaphoreTake(config_param_lock, portMAX_DELAY);
    mainsCurrentCh = newCurrentCh;
    xSemaphoreGive(config_param_lock);

    xEventGroupSetBits(config_param_is_init, INIT_PARAM_MAINS_CURRENT_CH);
}

static void handle_mqtt_set_mainsVoltageCh(const char *data, int len) {
    printf("received mainsVoltageCh data: %.*s\n", len, data);

    if (len >= SCRATCH_BUFF_SIZE - 1) {
        dadlib_panic("data too long!");
    }

    char buff[SCRATCH_BUFF_SIZE];
    memcpy(buff, data, len);
    buff[len] = 0;

    long newVoltageCh = strtol(buff, NULL, 10);

    printf("setting mainsVoltageCh to: %ld\n", newVoltageCh);

    xSemaphoreTake(config_param_lock, portMAX_DELAY);
    mainsVoltageCh = newVoltageCh;
    xSemaphoreGive(config_param_lock);

    xEventGroupSetBits(config_param_is_init, INIT_PARAM_MAINS_VOLTAGE_CH);
}

static void loop(spi_device_handle_t spi)
{
    // Step 1: read NUM_SAMPLES samples from the ADC (safe to do without holding the config params lock as they are not used here)
    doRead(spi);
    for (int c = 0; c < NUM_SAMPLES; c++)
    {
        readAllChannels(spi, adcData[c]);
    }

    // Step 2: calculate RMS and power for each channel, which uses the config params and so needs to hold the lock
    xSemaphoreTake(config_param_lock, portMAX_DELAY);

    int startSample, endSample;
    if (!findEndpoints(&startSample, &endSample))
    {
        lastErrorTimeUs = esp_timer_get_time();
        goto done;
    }

    for (int ch = 0; ch < 8; ch++)
    {
        calcRmsAndPower(ch, startSample, endSample);
    }

    publishResults(); // Send to MQTT, uses the config params

    done:
    setLedStatus();
    xSemaphoreGive(config_param_lock);
}

void app_main()
{
    printf("Starting energy monitor...\n");
    
    config_param_lock = xSemaphoreCreateBinaryStatic(&config_param_lock_buff);
    config_param_is_init = xEventGroupCreateStatic(&config_param_is_init_buff);

    // Initially the lock is available
    xSemaphoreGive(config_param_lock);

    dadlib_mqtt_topic_handler_t mqtt_subscribe_topics[] = {
        {
            .topic = "esp32/energymon/offset",
            .handler = handle_mqtt_set_offset,
        },
        {
            .topic = "esp32/energymon/dcMode",
            .handler = handle_mqtt_set_dcMode,
        },
        {
            .topic = "esp32/energymon/mainsCurrentCh",
            .handler = handle_mqtt_set_mainsCurrentCh,
        },
        {
            .topic = "esp32/energymon/mainsVoltageCh",
            .handler = handle_mqtt_set_mainsVoltageCh,
        },
        { NULL }
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
    dadlib_setup_pin_output(BLUE_LED);

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

    // wait until all config params are initialized
    EventBits_t ret = xEventGroupWaitBits(config_param_is_init, INIT_PARAM_ALL, pdFALSE, pdTRUE, pdMS_TO_TICKS(5000));
    if (ret != INIT_PARAM_ALL) {
        dadlib_panic("took too long to get init params");
    }

    vTaskPrioritySet(NULL, 20);

    while (true)
    {
        loop(spi);
        vTaskDelay(1);
    }
}