// #include "esp_wifi.h"       // Wi-Fi driver
// #include "esp_event.h"      // Event handling system
// #include "esp_log.h"        // Logging functions (ESP_LOGI, ESP_LOGE, etc.)
// #include "esp_system.h"     // System functions (restart, abort, etc.)
// #include "nvs_flash.h"      // Non-volatile storage (NVS) functions
// #include <string.h>         // String handling

// // Wi-Fi credentials
// #define WIFI_SSID "PRINTER24"
// #define WIFI_PASS "herein3016830168."

// // Logging tag used in ESP_LOG macros
// static const char *TAG = "wifi";

// // Flags to track connection state
// static bool got_ip = false;          // True once we've received an IP
// static bool handler_registered = false; // True once event handlers are registered

// // ------------------------
// // Event handler for Wi-Fi events
// // ------------------------
// static void wifi_event_handler(void* arg, esp_event_base_t event_base,
//                                int32_t event_id, void* event_data)
// {
//     // Event: Wi-Fi interface started
//     if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
//         ESP_LOGI(TAG, "Wi-Fi started, attempting to connect...");
//         esp_wifi_connect();  // Start connecting to the AP
//     }
//     // Event: Wi-Fi disconnected
//     else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
//         ESP_LOGI(TAG, "Wi-Fi disconnected, retrying...");
//         got_ip = false;      // Reset got_ip flag so we can log again
//         esp_wifi_connect();  // Retry connecting automatically
//     }
//     // Event: Got IP address from AP
//     else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
//         if (!got_ip) {  // Only log once per connection
//             ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
//             ESP_LOGI(TAG, "Connected! Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
//             got_ip = true;  // Prevent repeated logging
//         }
//     }
// }

// // ------------------------
// // Register Wi-Fi event handlers (only once)
// // ------------------------
// static void register_wifi_handlers_once(void)
// {
//     if (!handler_registered) {
//         ESP_ERROR_CHECK(esp_event_handler_instance_register(
//             WIFI_EVENT,
//             ESP_EVENT_ANY_ID,
//             &wifi_event_handler,
//             NULL,
//             NULL
//         ));

//         ESP_ERROR_CHECK(esp_event_handler_instance_register(
//             IP_EVENT,
//             IP_EVENT_STA_GOT_IP,
//             &wifi_event_handler,
//             NULL,
//             NULL
//         ));

//         handler_registered = true; // Mark as registered
//     }
// }

// // ------------------------
// // Main application entry
// // ------------------------
// void app_main(void)
// {
//     // ------------------------
//     // Step 1: Initialize NVS (non-volatile storage)
//     // Wi-Fi needs NVS to store calibration data and credentials
//     // ------------------------
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase()); // Clear NVS if corrupted or outdated
//         ret = nvs_flash_init();             // Re-initialize NVS
//     }
//     ESP_ERROR_CHECK(ret);

//     // ------------------------
//     // Step 2: Initialize TCP/IP stack and default event loop
//     // Must be done before using Wi-Fi
//     // ------------------------
//     ESP_ERROR_CHECK(esp_netif_init());                    // Initialize network interfaces
//     ESP_ERROR_CHECK(esp_event_loop_create_default());    // Create default event loop

//     // Create default Wi-Fi station (STA) network interface
//     esp_netif_create_default_wifi_sta();

//     // ------------------------
//     // Step 3: Initialize Wi-Fi driver
//     // ------------------------
//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_wifi_init(&cfg));

//     // ------------------------
//     // Step 4: Register event handlers
//     // ------------------------
//     register_wifi_handlers_once();  // Safe, only registers once

//     // ------------------------
//     // Step 5: Configure Wi-Fi connection
//     // ------------------------
//     wifi_config_t wifi_config = {
//         .sta = {
//             .ssid = WIFI_SSID,
//             .password = WIFI_PASS,
//             // Optional: set authentication mode threshold
//             //.threshold.authmode = WIFI_AUTH_WPA2_PSK,
//         },
//     };

//     // Set mode to station (client)
//     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

//     // Apply Wi-Fi configuration
//     ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));

//     // Start Wi-Fi driver (asynchronous)
//     ESP_ERROR_CHECK(esp_wifi_start());

//     // Log initialization complete; actual connection happens asynchronously
//     ESP_LOGI(TAG, "Wi-Fi initialization complete, connecting to %s...", WIFI_SSID);

//     // ------------------------
//     // Optional: wait until connected (blocking)
//     // Uncomment if you want app_main to block until Wi-Fi is connected
//     // ------------------------

//     while (!got_ip) {
//         vTaskDelay(pdMS_TO_TICKS(100)); // Wait 100ms and check again
//     }
//     ESP_LOGI(TAG, "Wi-Fi is now fully connected, continuing application...");
   
// }

//-----NON SSL WORKING-----------------------------





// #include "esp_wifi.h"
// #include "esp_event.h"
// #include "esp_log.h"
// #include "esp_system.h"
// #include "nvs_flash.h"
// #include "mqtt_client.h"
// #include "esp_tls.h"
// #include <string.h>

// // Wi-Fi and MQTT configuration
// #define WIFI_SSID "PRINTER24"
// #define WIFI_PASS "herein3016830168."
// // Try without SSL first
// #define BROKER_URL "mqtt://192.168.1.61:1883" // Standard MQTT port
// #define MQTT_TOPIC "esp32/energymon"
// #define MQTT_USERNAME "power"
// #define MQTT_PASSWORD "123456"

// static const char *TAG = "wifi";
// static bool got_ip = false;
// static bool handler_registered = false;
// static esp_mqtt_client_handle_t mqtt_client;

// // Enhanced MQTT event handler for better debugging
// static void mqtt_event_handler(void *handler_args, esp_event_base_t base, 
//                                int32_t event_id, void *event_data) {
//     esp_mqtt_event_handle_t event = event_data;
//     switch (event->event_id) {
//         case MQTT_EVENT_BEFORE_CONNECT:
//             ESP_LOGI(TAG, "MQTT attempting to connect to %s", BROKER_URL);
//             break;
//         case MQTT_EVENT_CONNECTED:
//             ESP_LOGI(TAG, "MQTT Connected!");
//             // Publish a number upon connection
//             {
//                 int number = 42;
//                 char payload[10];
//                 sprintf(payload, "%d", number);
//                 int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, payload, 0, 1, 0);
//                 ESP_LOGI(TAG, "Published message with ID: %d", msg_id);
//             }
//             break;
//         case MQTT_EVENT_DISCONNECTED:
//             ESP_LOGI(TAG, "MQTT Disconnected.");
//             break;
//         case MQTT_EVENT_ERROR:
//             ESP_LOGI(TAG, "MQTT Error: %s", esp_err_to_name(event->error_handle->error_type));
//             if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
//                 ESP_LOGI(TAG, "Last error code: 0x%x", event->error_handle->esp_tls_last_esp_err);
//                 ESP_LOGI(TAG, "TLS stack error code: 0x%x", event->error_handle->esp_tls_stack_err);
//                 ESP_LOGI(TAG, "SSL error code: %d", event->error_handle->esp_transport_sock_errno);
//             }
//             break;
//         default:
//             break;
//     }
// }

// // Start MQTT Client
// static void start_mqtt(void) {
//     esp_mqtt_client_config_t mqtt_cfg = {
//         .broker = {
//             .address.uri = BROKER_URL,
//         },
//         .credentials = {
//             .username = MQTT_USERNAME,
//             .authentication.password = MQTT_PASSWORD
//         }
//     };
    
//     mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
//     esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
//     esp_mqtt_client_start(mqtt_client);
// }

// // Wi-Fi Event Handler
// static void wifi_event_handler(void* arg, esp_event_base_t event_base,
//                                int32_t event_id, void* event_data)
// {
//     // Event: Wi-Fi interface started
//     if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
//         ESP_LOGI(TAG, "Wi-Fi started, attempting to connect...");
//         esp_wifi_connect();  // Start connecting to the AP
//     }
//     // Event: Wi-Fi disconnected
//     else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
//         ESP_LOGI(TAG, "Wi-Fi disconnected, retrying...");
//         got_ip = false;      // Reset got_ip flag so we can log again
//         esp_wifi_connect();  // Retry connecting automatically
//     }
//     // Event: Got IP address from AP
//     else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
//         if (!got_ip) {  // Only log once per connection
//             ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
//             ESP_LOGI(TAG, "Connected! Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
//             got_ip = true;  // Prevent repeated logging
//             start_mqtt(); // Start MQTT after IP acquisition
//         }
//     }
// }

// // Register Wi-Fi event handlers (only once)
// static void register_wifi_handlers_once(void)
// {
//     if (!handler_registered) {
//         ESP_ERROR_CHECK(esp_event_handler_instance_register(
//             WIFI_EVENT,
//             ESP_EVENT_ANY_ID,
//             &wifi_event_handler,
//             NULL,
//             NULL
//         ));

//         ESP_ERROR_CHECK(esp_event_handler_instance_register(
//             IP_EVENT,
//             IP_EVENT_STA_GOT_IP,
//             &wifi_event_handler,
//             NULL,
//             NULL
//         ));

//         handler_registered = true; // Mark as registered
//     }
// }

// // Main application entry
// void app_main(void)
// {
//     // Set log level for debugging
//     esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
//     esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
//     esp_log_level_set("transport", ESP_LOG_VERBOSE);
    
//     // Step 1: Initialize NVS (non-volatile storage)
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase()); // Clear NVS if corrupted or outdated
//         ret = nvs_flash_init();             // Re-initialize NVS
//     }
//     ESP_ERROR_CHECK(ret);

//     // Step 2: Initialize TCP/IP stack and default event loop
//     ESP_ERROR_CHECK(esp_netif_init());                    // Initialize network interfaces
//     ESP_ERROR_CHECK(esp_event_loop_create_default());    // Create default event loop

//     // Create default Wi-Fi station (STA) network interface
//     esp_netif_create_default_wifi_sta();

//     // Step 3: Initialize Wi-Fi driver
//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_wifi_init(&cfg));

//     // Step 4: Register event handlers
//     register_wifi_handlers_once();  // Safe, only registers once

//     // Step 5: Configure Wi-Fi connection
//     wifi_config_t wifi_config = {
//         .sta = {
//             .ssid = WIFI_SSID,
//             .password = WIFI_PASS,
//         },
//     };

//     // Set mode to station (client)
//     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

//     // Apply Wi-Fi configuration
//     ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));

//     // Start Wi-Fi driver (asynchronous)
//     ESP_ERROR_CHECK(esp_wifi_start());

//     // Log initialization complete; actual connection happens asynchronously
//     ESP_LOGI(TAG, "Wi-Fi initialization complete, connecting to %s...", WIFI_SSID);

//     // Wait until connected (blocking)
//     while (!got_ip) {
//         vTaskDelay(pdMS_TO_TICKS(100)); // Wait 100ms and check again
//     }
//     ESP_LOGI(TAG, "Wi-Fi is now fully connected, continuing application...");
    
//     // Keep the application running
//     while (true) {
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }


//-----------------------------------