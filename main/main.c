#include "dadlib.h"

#include <stdio.h>

void app_main()
{
    dadlib_config_t config = {
        .wifi_ssid = "PRINTER24",
        .wifi_pass = "herein3016830168.",

        .mqtt_user = "power",
        .mqtt_pass = "123456",
        .mqtt_broker_url = "mqtts://192.168.1.61:8885", // SSL connection to VerneMQ
        .mqtt_topic = "esp32/energymon",
    };
    dadlib_init(&config);

    // demo for dad
    double c1 = 32049843.43;
    double c2 = 123456789.0123456789;
    double c3 = 987654321.0987654321;

    char buf[256];
    int ret = snprintf(buf, sizeof(buf), "{ "
        "\"dad_coolness_1\": %lf,"
        "\"dad_coolness_2\": %lf,"
        "\"dad_coolness_3\": %lf }",
        c1, c2, c3);

    if (ret >= sizeof(buf) - 1) {
        dadlib_panic("dad_coolness json buffer too small");
    } else if (ret < 0) {
        dadlib_panic("snprintf failed");
    }

    dadlib_mqtt_publish(buf, 0, 1);
}




