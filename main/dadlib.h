#ifndef DADLIB_H
#define DADLIB_H

#include <stdbool.h>

struct dadlib_mqtt_topic_handler {
    const char *topic;
    void (*handler)(const char *data, int len);
};

typedef struct dadlib_mqtt_topic_handler dadlib_mqtt_topic_handler_t;

struct dadlib_config {
    const char *wifi_ssid;
    const char *wifi_pass;

    const char *mqtt_user;
    const char *mqtt_pass;
    const char *mqtt_broker_url;
    dadlib_mqtt_topic_handler_t *mqtt_subscribe_topics;

    bool skip_wait_for_wifi_and_mqtt;
};

typedef struct dadlib_config dadlib_config_t;

void dadlib_init(const dadlib_config_t *cfg);
void dadlib_panic(const char *error) __attribute__((noreturn));
int dadlib_mqtt_publish(const char *topic, const char *data, int qos, int retain);

void dadlib_setup_pin_input(int pin);
void dadlib_setup_pin_output(int pin);

#endif