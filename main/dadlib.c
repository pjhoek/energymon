#include "dadlib.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "esp_tls.h"
#include <string.h>

static const char *TAG = "dadlib";
static bool handler_registered = false;
static esp_mqtt_client_handle_t mqtt_client;

static bool mqtt_started = false;
static SemaphoreHandle_t ready_sem;
static StaticSemaphore_t ready_sem_buff;

static dadlib_config_t config;

// version of `esp_mqtt_client_publish` for dadlib
int dadlib_mqtt_publish(const char *data, int qos, int retain) {
    return esp_mqtt_client_publish(mqtt_client, config.mqtt_topic, data, strlen(data), qos, retain);
}

// Enhanced MQTT event handler for better debugging
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, 
                               int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    switch (event->event_id) {
        case MQTT_EVENT_BEFORE_CONNECT:
            ESP_LOGI(TAG, "MQTT attempting to connect to %s", config.mqtt_broker_url);
            break;
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected!");
            
            xSemaphoreGive(ready_sem);

            // Publish a number upon connection
            {
                int number = 42;
                char payload[10];
                sprintf(payload, "%d", number);
                int msg_id = esp_mqtt_client_publish(mqtt_client, config.mqtt_topic, payload, 0, 1, 0);
                ESP_LOGI(TAG, "Published message with ID: %d", msg_id);
            }
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT Disconnected.");
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT Error: %s", esp_err_to_name(event->error_handle->error_type));
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGI(TAG, "Last error code: 0x%x", event->error_handle->esp_tls_last_esp_err);
                ESP_LOGI(TAG, "TLS stack error code: 0x%x", event->error_handle->esp_tls_stack_err);
                ESP_LOGI(TAG, "SSL error code: %d", event->error_handle->esp_transport_sock_errno);
            }
            break;
        default:
            break;
    }
}

static void start_mqtt() {
    if (mqtt_started) {
        return;
    }
    mqtt_started = true;

    // For insecure skip to work:
    // 1) Leave .broker.verification empty (no CA/PSK/global store)
    // 2) Ensure CONFIG_ESP_TLS_SKIP_SERVER_CERT_VERIFY is enabled (via sdkconfig.defaults)
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = config.mqtt_broker_url,
            .verification = {
                // Only optional CN check (does not disable verification)
                .skip_cert_common_name_check = true
            }
        },
        .credentials = {
            .username = config.mqtt_user,
            .authentication.password = config.mqtt_pass
        }
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}



// Wi-Fi Event Handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    // Event: Wi-Fi interface started
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Wi-Fi started, attempting to connect...");
        esp_wifi_connect();  // Start connecting to the AP
    }
    // Event: Wi-Fi disconnected
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Wi-Fi disconnected, retrying...");
        esp_wifi_connect();  // Retry connecting automatically
    }
    // Event: Got IP address from AP
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Connected! Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        start_mqtt(); // Start MQTT after IP acquisition
    } else {
        ESP_LOGW(TAG, "Unhandled Wi-Fi event: 0x%X (ID: %ld)", 
                 (unsigned int) event_base, event_id);
    }
}

// Register Wi-Fi event handlers (only once)
static void register_wifi_handlers_once(void)
{
    if (!handler_registered) {
        ESP_ERROR_CHECK(esp_event_handler_instance_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            &wifi_event_handler,
            NULL,
            NULL
        ));

        ESP_ERROR_CHECK(esp_event_handler_instance_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            &wifi_event_handler,
            NULL,
            NULL
        ));

        handler_registered = true; // Mark as registered
    }
}

// Loop forever just showing our error message
void dadlib_panic(const char *error) {
    ESP_LOGE(TAG, "%s", error);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Init wifi and mqtt
void dadlib_init(const dadlib_config_t *user_config)
{
    if (!user_config->wifi_ssid || !user_config->wifi_pass) {
        dadlib_panic("missing wifi info");
    }

    if (!user_config->mqtt_user || !user_config->mqtt_pass || !user_config->mqtt_broker_url || !user_config->mqtt_topic) {
        dadlib_panic("missing mqtt info");
    }

    // copy all fields from passed config into our private copy
    config.wifi_ssid = strdup(user_config->wifi_ssid);
    config.wifi_pass = strdup(user_config->wifi_pass);
    config.mqtt_user = strdup(user_config->mqtt_user);
    config.mqtt_pass = strdup(user_config->mqtt_pass);
    config.mqtt_broker_url = strdup(user_config->mqtt_broker_url);
    config.mqtt_topic = strdup(user_config->mqtt_topic);

    ready_sem = xSemaphoreCreateBinaryStatic(&ready_sem_buff);

#if CONFIG_ESP_TLS_SKIP_SERVER_CERT_VERIFY
    ESP_LOGW(TAG, "INSECURE TLS BUILD: server certificate verification is DISABLED.");
#endif

    // Set log level for debugging
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("transport", ESP_LOG_VERBOSE);
    
    // Step 1: Initialize NVS (non-volatile storage)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase()); // Clear NVS if corrupted or outdated
        ret = nvs_flash_init();             // Re-initialize NVS
    }
    ESP_ERROR_CHECK(ret);

    // Step 2: Initialize TCP/IP stack and default event loop
    ESP_ERROR_CHECK(esp_netif_init());                    // Initialize network interfaces
    ESP_ERROR_CHECK(esp_event_loop_create_default());    // Create default event loop

    // Create default Wi-Fi station (STA) network interface
    esp_netif_create_default_wifi_sta();

    // Step 3: Initialize Wi-Fi driver
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));

    // Step 4: Register event handlers
    register_wifi_handlers_once();  // Safe, only registers once

    // Step 5: Configure Wi-Fi connection
    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config_t));
    strcpy((char *) wifi_config.sta.ssid, config.wifi_ssid);
    strcpy((char *) wifi_config.sta.password, config.wifi_pass);

    // Set mode to station (client)
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // Apply Wi-Fi configuration
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));

    // Start Wi-Fi driver (asynchronous)
    ESP_ERROR_CHECK(esp_wifi_start());

    // Log initialization complete; actual connection happens asynchronously
    ESP_LOGI(TAG, "Wi-Fi initialization complete, connecting to %s...", config.wifi_ssid);

    // Wait until connected (blocking)
    xSemaphoreTake(ready_sem, portMAX_DELAY);

    // We are done!
    ESP_LOGI(TAG, "dadlib initialization complete");
}
