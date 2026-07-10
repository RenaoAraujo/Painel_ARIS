#pragma once

#include <stdint.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_ldo_regulator.h"
#include "esp_chip_info.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_app_desc.h"
#include "esp_idf_version.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"

#include "bsp_i2c.h"
#include "bsp_display.h"
#include "bsp_illuminate.h"

#define APP_TAG  "APP"
#define APP_INFO(fmt, ...)  ESP_LOGI(APP_TAG, fmt, ##__VA_ARGS__)
#define APP_WARN(fmt, ...)  ESP_LOGW(APP_TAG, fmt, ##__VA_ARGS__)
#define APP_ERROR(fmt, ...) ESP_LOGE(APP_TAG, fmt, ##__VA_ARGS__)

/**
 * Aba OS do dashboard (separador de remédios). Chamadas seguras de outras tasks
 * se usar lvgl_port_lock no caller ou se o LVGL estiver configurado como thread-safe.
 */
void ui_os_set_ordem(const char *numero_os, const char *nome_cliente);
void ui_os_set_remedios(uint32_t separados, uint32_t total);
/** Regista o tempo (ms) que a última pegada de remédio demorou; alimenta a média e a ETA. */
void ui_os_registar_tempo_pegada_ms(uint32_t duracao_ms);

/**
 * Adiciona uma OS concluída ao histórico (mais recente primeiro). Usar com lvgl_port_lock se noutra task.
 * @param duracao_ms tempo da OS em ms; use 0 para calcular desde o último ui_os_set_ordem (esp_timer).
 */
void ui_historico_registar_os(const char *numero_os, const char *nome_cliente,
                              uint32_t separados, uint32_t total, uint32_t duracao_ms);
