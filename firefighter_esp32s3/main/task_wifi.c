// task_wifi.c - WiFi access point and web control panel
//
// Connect to "FireFighter" WiFi then open http://192.168.4.1
// Buttons: START, STOP, RESUME, RESET
// Also lets you tune the turn time and shows current state

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "config.h"
#include "shared.h"

static const char *TAG = "WIFI";

static const char INDEX_HTML[] =
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>FireFighter Robot</title>"
    "<style>body{font-family:sans-serif;text-align:center;padding:20px;background:#1a1a2e;color:#eee}"
    "h1{color:#e94560}button{margin:10px;padding:15px 30px;font-size:1.2em;border:none;"
    "border-radius:8px;cursor:pointer}"
    ".start{background:#27ae60}.stop{background:#c0392b}.resume{background:#2980b9}"
    ".reset{background:#7f8c8d}.setturn{background:#8e44ad}"
    "#status{margin-top:20px;padding:15px;background:#16213e;border-radius:8px;font-size:1.1em}"
    "#tuning{margin-top:20px;padding:15px;background:#16213e;border-radius:8px}"
    "input{padding:10px;font-size:1.1em;width:100px;border-radius:6px;border:none;text-align:center}"
    "</style></head><body>"
    "<h1>FireFighter Robot</h1>"
    "<button class='start'  onclick=\"post('/cmd_start')\">START</button>"
    "<button class='stop'   onclick=\"post('/cmd_stop')\">STOP</button>"
    "<button class='resume' onclick=\"post('/cmd_resume')\">RESUME</button>"
    "<button class='reset'  onclick=\"post('/cmd_reset')\">RESET</button>"
    "<div id='status'>Loading...</div>"
    "<div id='tuning'>"
    "<b>Turn time (ms):</b> "
    "<input id='tms' type='number' value='1800' min='100' max='5000'>"
    "<button class='setturn' onclick=\"setTurn()\">SET</button>"
    "</div>"
    "<script>"
    "function post(url){fetch(url,{method:'POST'}).then(()=>refresh());}"
    "function setTurn(){"
    "  var v=document.getElementById('tms').value;"
    "  fetch('/set_turn?ms='+v,{method:'POST'}).then(()=>refresh());}"
    "function refresh(){"
    "  fetch('/status').then(r=>r.json()).then(d=>{"
    "    document.getElementById('status').innerHTML="
    "      'State: <b>'+d.state+'</b>  |  LiDAR: '+d.lidar_cm+' cm  |  Turn: '+d.turn_ms+'ms';"
    "    document.getElementById('tms').value=d.turn_ms;"
    "  });}"
    "setInterval(refresh,1000);refresh();"
    "</script></body></html>";

static const char *state_name(robot_state_t s)
{
    switch (s) {
        case STATE_IDLE:           return "IDLE";
        case STATE_PATH_FOLLOW:    return "PATH_FOLLOW";
        case STATE_OBSTACLE_AVOID: return "OBSTACLE_AVOID";
        case STATE_FIRE_APPROACH:  return "FIRE_APPROACH";
        case STATE_EXTINGUISH:     return "EXTINGUISH";
        case STATE_DONE:           return "DONE";
        default:                   return "UNKNOWN";
    }
}

static esp_err_t handle_index(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t handle_cmd_start(httpd_req_t *req)
{
    xEventGroupSetBits(eg_commands, CMD_START_BIT);
    httpd_resp_send(req, "OK", 2);
    ESP_LOGI(TAG, "CMD_START");
    return ESP_OK;
}

static esp_err_t handle_cmd_stop(httpd_req_t *req)
{
    xEventGroupSetBits(eg_commands, CMD_STOP_BIT);
    httpd_resp_send(req, "OK", 2);
    ESP_LOGI(TAG, "CMD_STOP");
    return ESP_OK;
}

static esp_err_t handle_cmd_resume(httpd_req_t *req)
{
    xEventGroupSetBits(eg_commands, CMD_RESUME_BIT);
    httpd_resp_send(req, "OK", 2);
    ESP_LOGI(TAG, "CMD_RESUME");
    return ESP_OK;
}

static esp_err_t handle_cmd_reset(httpd_req_t *req)
{
    xEventGroupSetBits(eg_commands, CMD_RESET_BIT);
    httpd_resp_send(req, "OK", 2);
    ESP_LOGI(TAG, "RESET");
    return ESP_OK;
}

static esp_err_t handle_status(httpd_req_t *req)
{
    char buf[160];
    snprintf(buf, sizeof(buf),
             "{\"state\":\"%s\",\"lidar_cm\":%d,\"flame\":%s,\"turn_ms\":%d}",
             state_name(g_state), g_lidar_cm,
             g_flame_detected ? "true" : "false",
             g_turn_ms);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t handle_set_turn(httpd_req_t *req)
{
    char query[32] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[16] = {0};
        if (httpd_query_key_value(query, "ms", val, sizeof(val)) == ESP_OK) {
            int ms = atoi(val);
            if (ms >= TURN_MS_MIN && ms <= TURN_MS_MAX) {
                g_turn_ms = ms;
                ESP_LOGI(TAG, "turn time = %dms", g_turn_ms);
            }
        }
    }
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static void start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port    = WEB_PORT;
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "failed to start HTTP server");
        return;
    }

    static const httpd_uri_t uris[] = {
        { .uri = "/",           .method = HTTP_GET,  .handler = handle_index      },
        { .uri = "/cmd_start",  .method = HTTP_POST, .handler = handle_cmd_start  },
        { .uri = "/cmd_stop",   .method = HTTP_POST, .handler = handle_cmd_stop   },
        { .uri = "/cmd_resume", .method = HTTP_POST, .handler = handle_cmd_resume },
        { .uri = "/cmd_reset",  .method = HTTP_POST, .handler = handle_cmd_reset  },
        { .uri = "/set_turn",   .method = HTTP_POST, .handler = handle_set_turn   },
        { .uri = "/status",     .method = HTTP_GET,  .handler = handle_status     },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++)
        httpd_register_uri_handler(server, &uris[i]);

    ESP_LOGI(TAG, "HTTP server up on port %d", WEB_PORT);
}

void task_wifi_start(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t ap_cfg = {
        .ap = {
            .ssid           = WIFI_SSID,
            .ssid_len       = sizeof(WIFI_SSID) - 1,
            .password       = WIFI_PASS,
            .max_connection = 4,
            .authmode       = WIFI_AUTH_WPA2_PSK,
        },
    };
    if (strlen(WIFI_PASS) == 0)
        ap_cfg.ap.authmode = WIFI_AUTH_OPEN;

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    esp_wifi_start();

    ESP_LOGI(TAG, "AP started: %s  192.168.4.1", WIFI_SSID);
    start_webserver();
}
