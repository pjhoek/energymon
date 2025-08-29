#ifndef DADLIB_H
#define DADLIB_H

#include <stdbool.h>

struct dadlib_config {
    const char *wifi_ssid;
    const char *wifi_pass;

    const char *mqtt_user;
    const char *mqtt_pass;
    const char *mqtt_broker_url;
    const char *mqtt_topic;

    bool skip_wait_for_wifi_and_mqtt;
};

typedef struct dadlib_config dadlib_config_t;

void dadlib_init(const dadlib_config_t *cfg);
void dadlib_panic(const char *error) __attribute__((noreturn));
int dadlib_mqtt_publish(const char *data, int qos, int retain);

#endif