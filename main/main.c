/*
 * Dashboard LVGL - Guition JC1060P470C (ESP32-P4 7" 1024×600 MIPI-DSI).
 * UI: splash -> login -> dashboard (OS, Motor, Historico) + tela Configuracoes.
 */

#include "app_main.h"
#include "esp_random.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>

/* ---------- GPIOs dos relés ---------- */
#define LED_AMARELO_GPIO   GPIO_NUM_1   /* acende durante OS ativa        */
#define LED_VERMELHO_GPIO  GPIO_NUM_2   /* pisca em pausa ou cancelamento  */
#define STEPPER_STEP_GPIO  GPIO_NUM_20  /* pulso STEP do motor de passo   */
#define STEPPER_DIR_GPIO   GPIO_NUM_32  /* direção DIR do motor de passo  */

#ifndef LOGIN_DEMO_USER
#define LOGIN_DEMO_USER  "admin"
#endif
#ifndef LOGIN_DEMO_PASS
#define LOGIN_DEMO_PASS  "123456"
#endif

static esp_ldo_channel_handle_t s_ldo3 = NULL;
static esp_ldo_channel_handle_t s_ldo4 = NULL;

/* ----- Estado partilhado entre touch_log_task e o dashboard ----- */
static volatile uint16_t s_last_touch_x = 0;
static volatile uint16_t s_last_touch_y = 0;
static volatile bool     s_last_touch_pressed = false;

/* ----- Aba OS (ordem de serviço / separador) — widgets ----- */
#define OS_NUM_BUF_SZ     32
#define OS_CLIENT_BUF_SZ  72

static lv_obj_t *s_lbl_os_number      = NULL;
static lv_obj_t *s_lbl_os_client      = NULL;
static lv_obj_t *s_lbl_os_meds        = NULL;
static lv_obj_t *s_bar_os_meds        = NULL;
static lv_obj_t *s_lbl_os_eta         = NULL;
static lv_obj_t *s_lbl_os_eta_detail  = NULL;
static lv_obj_t  *s_settings_dash_scr = NULL;

typedef struct {
    lv_obj_t *mi_dev;
    lv_obj_t *lbl_dev;
    lv_obj_t *mi_telas;
    lv_obj_t *lbl_telas;
    lv_obj_t *panel_dev;
    lv_obj_t *panel_telas;
} settings_nav_t;

static settings_nav_t s_settings_nav;

static char       s_os_num_str[OS_NUM_BUF_SZ]       = "";
static char       s_os_client_str[OS_CLIENT_BUF_SZ] = "";
static uint32_t   s_os_meds_total   = 0;
static uint32_t   s_os_meds_done    = 0;
static uint64_t   s_os_pick_ms_sum  = 0;
static uint32_t   s_os_pick_samples = 0;
static uint64_t   s_os_sess_start_us = 0;
static bool       s_os_active       = false;
static lv_timer_t *s_os_pick_timer  = NULL;

/* Sessao: true enquanto o utilizador esta autenticado (dashboard visivel).
 * Ao sair (logout), a separacao da OS continua a correr em segundo plano. */
static bool       s_logged_in       = false;

/* Conclusao de OS ocorrida enquanto o utilizador estava fora (logout):
 * guarda os dados para mostrar o modal quando voltar a entrar. */
static bool       s_os_concl_pending = false;
static char       s_os_concl_num[OS_NUM_BUF_SZ] = "";
static uint32_t   s_os_concl_total  = 0;

static lv_timer_t *s_dash_timer    = NULL;

/* ----- Histórico de OS (aba Historico) ----- */
#define OS_HIST_MAX        48
#define OS_HIST_CLIENT_SZ  56
#define OS_HIST_MOTIVO_SZ  128

/* ----- Comandos recebidos do gateway (Web → ESP32) ----- */
typedef enum {
    CMD_OS_ASSIGN   = 0,
    CMD_OS_COMPLETE = 1,
    CMD_OS_CANCEL   = 2,
    CMD_OS_PAUSE    = 3,
    CMD_OS_RESUME   = 4,
    CMD_PING        = 5,
    CMD_SET_DEVICE  = 6,
} serial_cmd_type_t;

typedef struct {
    serial_cmd_type_t type;
    char     num[OS_NUM_BUF_SZ];
    char     client[OS_CLIENT_BUF_SZ];
    uint32_t total;
    char     reason[OS_HIST_MOTIVO_SZ];
} serial_cmd_t;

#define SERIAL_CMD_QUEUE_LEN 8

static QueueHandle_t  s_cmd_queue      = NULL;
static lv_timer_t    *s_cmd_timer      = NULL;
/* Quando true, os serial_bridge_* nao emitem eventos (evita eco de CMDs recebidos). */
static volatile bool  s_bridge_suppress = false;
static volatile bool  s_os_paused       = false;
static uint32_t       s_bridge_unit_pending_done = 0;
static bool           s_bridge_unit_pending      = false;

typedef enum {
    OS_HIST_CONCLUIDA = 0,
    OS_HIST_CANCELADA = 1,
} os_hist_estado_t;

typedef struct {
    char     numero[OS_NUM_BUF_SZ];
    char     cliente[OS_HIST_CLIENT_SZ];
    uint32_t separados;
    uint32_t total;
    uint32_t duracao_ms;
    uint8_t  estado;
    char     motivo[OS_HIST_MOTIVO_SZ];
} os_hist_item_t;

typedef struct {
    lv_obj_t *panel_idle;
    lv_obj_t *panel_active;
    lv_obj_t *modal;
    lv_obj_t *ta_num;
    lv_obj_t *ta_client;
    lv_obj_t *ta_total;
    lv_obj_t *ta_motivo;
    lv_obj_t *btn_pause;
    lv_obj_t *lbl_btn_pause;
} os_widgets_t;

static os_widgets_t s_os;

static os_hist_item_t s_os_hist[OS_HIST_MAX];
static uint32_t       s_os_hist_count = 0;
static lv_obj_t      *s_hist_scroll   = NULL;
static lv_obj_t      *s_hist_modal    = NULL;
static size_t         s_hist_modal_idx;

typedef struct {
    lv_obj_t *scr;
    lv_obj_t *ta_user;
    lv_obj_t *ta_pass;
    lv_obj_t *lbl_err;
    lv_obj_t *kb;
} login_widgets_t;

static login_widgets_t s_login;

#define DEV_ROBOT_NAME_SZ 56
#define DEV_ROBOT_ID_SZ   56

typedef struct {
    lv_obj_t *ta_nome;
    lv_obj_t *ta_id;
    lv_obj_t *kb;
    lv_obj_t *lbl_os_cnt;
    lv_obj_t *lbl_remed_cnt;
} dev_widgets_t;

static dev_widgets_t s_dev;
static uint32_t      s_dev_os_separadas    = 0;
static uint32_t      s_dev_remed_separados = 0;
/* Nome e código do robô em RAM (sincronizados com NVS e usados pelo gateway). */
static char s_dev_robot_name[DEV_ROBOT_NAME_SZ] = "";
static char s_dev_robot_code[DEV_ROBOT_ID_SZ]   = "";

static void build_login_ui_into(lv_obj_t *scr);
static void build_dashboard_into(lv_obj_t *scr);
static void serial_cmd_timer_cb(lv_timer_t *t);
static void os_show_concluida_modal(const char *num, uint32_t total);
static void build_splash(void);
static void historico_close_modal(void);
static void historico_rebuild_list(void);
static void historico_card_clicked(lv_event_t *e);
static void dev_kb_destroy(void);
static void os_pick_timer_stop(void);
static void os_close_modal(void);
static void os_update_panels(void);
static void os_reset_idle(void);
static void os_open_manual_modal(void);
static void os_open_cancel_modal(void);
static void os_start_manual(const char *numero, const char *cliente, uint32_t total);
static void os_finalize_concluida(void);
static void os_finalize_cancelada(const char *motivo);
static void os_pause(void);
static void os_resume(void);
static void os_schedule_next_pick(void);
static void serial_bridge_unit_deferred_cb(lv_timer_t *t);
static void historico_push(const char *numero, const char *cliente, uint32_t separados, uint32_t total,
                           uint32_t duracao_ms, os_hist_estado_t estado, const char *motivo);
static void build_settings_screen(lv_obj_t *dash_scr);
static void os_btn_pausar_os_cb(lv_event_t *e);
static void serial_bridge_os_pause(const char *num);
static void serial_bridge_os_resume(const char *num);
static void stepper_stop(void);
static void stepper_close_modal(void);

/* --------------------------------------------------------------------------
 * NVS — persistência de nome e código do robô
 * -------------------------------------------------------------------------- */
#define NVS_NS        "aris_cfg"
#define NVS_KEY_NAME      "robot_name"
#define NVS_KEY_CODE      "robot_code"
/* Motor de passo */
#define NVS_KEY_STEP_RPM  "step_rpm"   /* uint16: velocidade em RPM          */
#define NVS_KEY_STEP_SPR  "step_spr"   /* uint16: passos por revolução       */
#define NVS_KEY_STEP_MSTP "step_mstp"  /* uint8 : índice microstepping       */

/* Tabela de divisores de microstepping suportados pelo driver */
static const uint8_t MICROSTEP_OPTIONS[] = {1, 2, 4, 8, 16, 32};
#define MICROSTEP_COUNT  ((int)(sizeof(MICROSTEP_OPTIONS)/sizeof(MICROSTEP_OPTIONS[0])))

/* Estado persistente do motor de passo */
static uint16_t s_stepper_rpm       = 60;  /* RPM padrão                     */
static uint16_t s_stepper_steps_rev = 200; /* passos/rev (motor 1.8°)        */
static uint8_t  s_stepper_ms_idx    = 0;   /* índice em MICROSTEP_OPTIONS    */

static void nvs_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
}

static void nvs_load_device_config(char *name, size_t name_sz, char *code, size_t code_sz)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    nvs_get_str(h, NVS_KEY_NAME, name, &name_sz);
    nvs_get_str(h, NVS_KEY_CODE, code, &code_sz);
    nvs_close(h);
}

static void nvs_save_str(const char *key, const char *val)
{
    if (!key || !val) { return; }
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_str(h, key, val);
    nvs_commit(h);
    nvs_close(h);
    APP_INFO("NVS: '%s' = '%s'", key, val);
}

static void nvs_save_u8(const char *key, uint8_t val)
{
    if (!key) { return; }
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_u8(h, key, val);
    nvs_commit(h);
    nvs_close(h);
    APP_INFO("NVS: '%s' = %u", key, (unsigned)val);
}

static uint8_t nvs_load_u8(const char *key, uint8_t def)
{
    if (!key) { return def; }
    nvs_handle_t h;
    uint8_t v = def;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return def;
    }
    nvs_get_u8(h, key, &v);
    nvs_close(h);
    return v;
}

static void nvs_save_u16(const char *key, uint16_t val)
{
    if (!key) { return; }
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) { return; }
    nvs_set_u16(h, key, val);
    nvs_commit(h);
    nvs_close(h);
    APP_INFO("NVS: '%s' = %u", key, (unsigned)val);
}

static uint16_t nvs_load_u16(const char *key, uint16_t def)
{
    if (!key) { return def; }
    nvs_handle_t h;
    uint16_t v = def;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) { return def; }
    nvs_get_u16(h, key, &v);
    nvs_close(h);
    return v;
}

static void str_copy_trunc(char *dst, size_t cap, const char *src)
{
    if (!dst || cap == 0) {
        return;
    }
    if (!src) {
        dst[0] = '\0';
        return;
    }
    size_t i;
    for (i = 0; i + 1 < cap && src[i]; i++) {
        dst[i] = src[i];
    }
    dst[i] = '\0';
}

/** Média ms por pegada; 0 se ainda não há amostras. */
static uint32_t os_avg_pick_ms(void)
{
    if (s_os_pick_samples == 0) {
        return 0;
    }
    return (uint32_t)(s_os_pick_ms_sum / (uint64_t)s_os_pick_samples);
}

/** Segundos estimados para concluir as unidades em falta; 0 se concluído ou sem dados. */
static uint32_t os_eta_seconds(void)
{
    if (s_os_meds_total <= s_os_meds_done) {
        return 0;
    }
    if (s_os_pick_samples == 0) {
        return 0;
    }
    uint32_t avg = os_avg_pick_ms();
    if (avg == 0) {
        avg = 1;
    }
    uint32_t rem = s_os_meds_total - s_os_meds_done;
    uint64_t eta_ms = (uint64_t)rem * (uint64_t)avg;
    if (eta_ms / 1000ULL > UINT32_MAX) {
        return UINT32_MAX;
    }
    return (uint32_t)(eta_ms / 1000ULL);
}

/** Texto legivel de duracao; ms==0 -> "N/D". */
static void format_duracao_ms(char *out, size_t cap, uint32_t ms)
{
    if (!out || cap == 0) {
        return;
    }
    if (ms == 0) {
        snprintf(out, cap, "N/D");
        return;
    }
    if (ms < 1000u) {
        snprintf(out, cap, "< 1 s");
        return;
    }
    uint32_t sec = ms / 1000u;
    uint32_t h   = sec / 3600u;
    uint32_t m   = (sec % 3600u) / 60u;
    uint32_t s   = sec % 60u;
    if (h > 0) {
        snprintf(out, cap, "%lu h %lu min %lu s", (unsigned long)h, (unsigned long)m, (unsigned long)s);
    } else if (m > 0) {
        snprintf(out, cap, "%lu min %lu s", (unsigned long)m, (unsigned long)s);
    } else {
        snprintf(out, cap, "%lu s", (unsigned long)s);
    }
}

/* Paleta baseada no logo A.R.I.S.
 * Fundo: #EEEEF5 (lavanda muito claro)
 * Accent: #7B7AB8 (roxo-lavanda médio — cor das letras do logo)
 * Value: #1F1F60  (azul-marinho escuro — cor do subtítulo do logo)
 */
#define COLOR_BG          lv_color_hex(0xEEEEF5)
#define COLOR_CARD        lv_color_hex(0xFFFFFF)
#define COLOR_BORDER      lv_color_hex(0xD4D4E8)
#define COLOR_LABEL       lv_color_hex(0x6B6B9E)
#define COLOR_VALUE       lv_color_hex(0x1F1F60)
#define COLOR_ACCENT      lv_color_hex(0x7B7AB8)
#define COLOR_ACCENT_DARK lv_color_hex(0x5A5A96)
#define COLOR_OK          lv_color_hex(0x10B981)
#define COLOR_BADGE_BG    lv_color_hex(0xE8E8F4)

/* ====================================================================
 * Splash screen — logo ARIS por 3 s antes do login.
 * O binário logo_aris.bin (RGB565 1024×600) é embedado pelo CMake.
 * ==================================================================== */
extern const uint8_t logo_aris_bin_start[] asm("_binary_logo_aris_bin_start");
extern const uint8_t logo_aris_bin_end[]   asm("_binary_logo_aris_bin_end");

#define SPLASH_W  1024
#define SPLASH_H  600
#define SPLASH_MS 3000

static lv_img_dsc_t s_splash_img_dsc;

static void splash_timer_cb(lv_timer_t *t)
{
    (void)t;
    /* Transição com fade para a tela de login. */
    lv_obj_t *login_scr = lv_obj_create(NULL);
    build_login_ui_into(login_scr);
    lv_scr_load_anim(login_scr, LV_SCR_LOAD_ANIM_FADE_ON, 400, 0, true);
}

static void build_splash(void)
{
    if (!lvgl_port_lock(0)) {
        APP_ERROR("Splash: falha no lock LVGL");
        return;
    }

    /* Descritor da imagem apontando para o binário embedado. */
    s_splash_img_dsc.header.always_zero = 0;
    s_splash_img_dsc.header.w           = SPLASH_W;
    s_splash_img_dsc.header.h           = SPLASH_H;
    s_splash_img_dsc.header.cf          = LV_IMG_CF_TRUE_COLOR;
    s_splash_img_dsc.data_size          = (uint32_t)(logo_aris_bin_end - logo_aris_bin_start);
    s_splash_img_dsc.data               = logo_aris_bin_start;

    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0xEEEEF5), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *img = lv_img_create(scr);
    lv_img_set_src(img, &s_splash_img_dsc);
    lv_obj_center(img);

    lv_scr_load(scr);
    lv_timer_t *t = lv_timer_create(splash_timer_cb, SPLASH_MS, NULL);
    lv_timer_set_repeat_count(t, 1);

    lvgl_port_unlock();
    APP_INFO("Splash screen ativo (%d ms)", SPLASH_MS);
}

/* ================================================================= */

/* Card branco com sombra suave, layout interno em coluna flex. */
static lv_obj_t *make_card(lv_obj_t *parent)
{
    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_set_style_bg_color(card, COLOR_CARD, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(card, COLOR_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_width(card, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(card, 18, LV_PART_MAIN);
    /* Sombra discreta: largura pequena evita area de redraw enorme em cada flush. */
    lv_obj_set_style_shadow_color(card, lv_color_hex(0x9090B8), LV_PART_MAIN);
    lv_obj_set_style_shadow_opa(card, LV_OPA_20, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(card, 10, LV_PART_MAIN);
    lv_obj_set_style_shadow_ofs_y(card, 3, LV_PART_MAIN);
    lv_obj_set_style_pad_all(card, 24, LV_PART_MAIN);
    lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(card, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(card, 12, LV_PART_MAIN);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    return card;
}

/* Label de cabeçalho do card ("UPTIME", "ULTIMO TOQUE", ...). */
static lv_obj_t *make_card_label(lv_obj_t *card, const char *text)
{
    lv_obj_t *lbl = lv_label_create(card);
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl, COLOR_LABEL, LV_PART_MAIN);
    lv_obj_set_style_text_letter_space(lbl, 2, LV_PART_MAIN);
    return lbl;
}

static bool login_obj_is_descendant_of(const lv_obj_t *child, const lv_obj_t *ancestor)
{
    if (child == NULL || ancestor == NULL) {
        return false;
    }
    const lv_obj_t *p = child;
    while (p) {
        if (p == ancestor) {
            return true;
        }
        p = lv_obj_get_parent(p);
    }
    return false;
}

/* Propaga LV_EVENT_CLICKED para o painel esconder o teclado ao tocar fora dele. */
static void login_bubble_events_up(lv_obj_t *root)
{
    if (!root) {
        return;
    }
    lv_obj_add_flag(root, LV_OBJ_FLAG_EVENT_BUBBLE);
    uint32_t i = 0;
    lv_obj_t *c;
    while ((c = lv_obj_get_child(root, i)) != NULL) {
        i++;
        login_bubble_events_up(c);
    }
}

/* Esconde o teclado ao tocar fora dele (alvo original = lv_event_get_target). */
static void login_clicked_dismiss_kb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    login_widgets_t *L = lv_event_get_user_data(e);
    if (!L || !L->kb || lv_obj_has_flag(L->kb, LV_OBJ_FLAG_HIDDEN)) {
        return;
    }

    lv_obj_t *t = lv_event_get_target(e);
    if (login_obj_is_descendant_of(t, L->kb)) {
        return;
    }
    if (L->ta_user && login_obj_is_descendant_of(t, L->ta_user)) {
        return;
    }
    if (L->ta_pass && login_obj_is_descendant_of(t, L->ta_pass)) {
        return;
    }

    lv_keyboard_set_textarea(L->kb, NULL);
    lv_obj_add_flag(L->kb, LV_OBJ_FLAG_HIDDEN);
}

static void style_login_ta(lv_obj_t *ta)
{
    lv_obj_set_style_bg_color(ta, COLOR_CARD, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ta, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(ta, COLOR_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_width(ta, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(ta, 10, LV_PART_MAIN);
    lv_obj_set_style_pad_left(ta, 14, LV_PART_MAIN);
    lv_obj_set_style_pad_right(ta, 14, LV_PART_MAIN);
    lv_obj_set_style_pad_top(ta, 10, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(ta, 10, LV_PART_MAIN);
    lv_obj_set_style_text_font(ta, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(ta, COLOR_VALUE, LV_PART_MAIN);
}

#define DASH_KB_HEIGHT 220

/** Repoe o teclado em layer_top (aba Dispositivo); esconde e desassocia o campo. */
static void dash_kb_undock_to_layer_top(void)
{
    if (!s_dev.kb) {
        return;
    }
    if (lv_obj_get_parent(s_dev.kb) != lv_layer_top()) {
        lv_obj_set_parent(s_dev.kb, lv_layer_top());
    }
    lv_obj_add_flag(s_dev.kb, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_size(s_dev.kb, lv_pct(100), DASH_KB_HEIGHT);
    lv_obj_align(s_dev.kb, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_keyboard_set_textarea(s_dev.kb, NULL);
    lv_obj_add_flag(s_dev.kb, LV_OBJ_FLAG_HIDDEN);
    lv_keyboard_set_mode(s_dev.kb, LV_KEYBOARD_MODE_TEXT_LOWER);
}

/** Mostra o teclado: no modal OS fica embutido em baixo (nao tapa os campos). */
static void dash_kb_show_for(lv_obj_t *ta, bool numeric)
{
    if (!s_dev.kb || !ta) {
        return;
    }
    lv_keyboard_set_textarea(s_dev.kb, ta);
    lv_keyboard_set_mode(s_dev.kb, numeric ? LV_KEYBOARD_MODE_NUMBER : LV_KEYBOARD_MODE_TEXT_LOWER);
    lv_obj_clear_flag(s_dev.kb, LV_OBJ_FLAG_HIDDEN);

    if (s_os.modal) {
        lv_obj_set_parent(s_dev.kb, s_os.modal);
        lv_obj_clear_flag(s_dev.kb, LV_OBJ_FLAG_FLOATING);
        lv_obj_set_width(s_dev.kb, lv_pct(100));
        lv_obj_set_height(s_dev.kb, DASH_KB_HEIGHT);
        uint32_t n = lv_obj_get_child_cnt(s_os.modal);
        if (n > 0) {
            lv_obj_move_to_index(s_dev.kb, n - 1);
        }
        lv_obj_update_layout(s_os.modal);
        lv_obj_scroll_to_view(ta, LV_ANIM_ON);
    } else {
        dash_kb_undock_to_layer_top();
        lv_keyboard_set_textarea(s_dev.kb, ta);
        lv_keyboard_set_mode(s_dev.kb, numeric ? LV_KEYBOARD_MODE_NUMBER : LV_KEYBOARD_MODE_TEXT_LOWER);
        lv_obj_clear_flag(s_dev.kb, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(s_dev.kb);
    }
}

/** Teclado partilhado do dashboard (Dispositivo + formularios OS). */
static void dash_ta_activate_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_FOCUSED && code != LV_EVENT_CLICKED) {
        return;
    }
    dash_kb_show_for(lv_event_get_target(e), false);
}

static void dash_ta_defocus_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_DEFOCUSED) {
        return;
    }
    dash_kb_undock_to_layer_top();
}

static void dash_ta_num_activate_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_FOCUSED && code != LV_EVENT_CLICKED) {
        return;
    }
    dash_kb_show_for(lv_event_get_target(e), true);
}

/** Campo de formulario OS: largura total e cresce no flex para preencher o card. */
static void style_os_form_field(lv_obj_t *ta, bool numeric_kb)
{
    style_login_ta(ta);
    lv_obj_set_width(ta, lv_pct(100));
    lv_obj_set_flex_grow(ta, 1);
    lv_obj_set_style_min_height(ta, 80, LV_PART_MAIN);
    if (numeric_kb) {
        lv_obj_add_event_cb(ta, dash_ta_num_activate_cb, LV_EVENT_FOCUSED, NULL);
        lv_obj_add_event_cb(ta, dash_ta_num_activate_cb, LV_EVENT_CLICKED, NULL);
    } else {
        lv_obj_add_event_cb(ta, dash_ta_activate_cb, LV_EVENT_FOCUSED, NULL);
        lv_obj_add_event_cb(ta, dash_ta_activate_cb, LV_EVENT_CLICKED, NULL);
    }
    lv_obj_add_event_cb(ta, dash_ta_defocus_cb, LV_EVENT_DEFOCUSED, NULL);
}

/* Mostra o teclado e associa ao campo (toque nem sempre envia FOCUS sozinho). */
static void login_ta_activate_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_FOCUSED && code != LV_EVENT_CLICKED) {
        return;
    }
    login_widgets_t *L = lv_event_get_user_data(e);
    if (L && L->kb) {
        lv_obj_t *ta = lv_event_get_target(e);
        lv_keyboard_set_textarea(L->kb, ta);
        lv_obj_clear_flag(L->kb, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(L->kb);
    }
}

static void dev_ta_defocus_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_DEFOCUSED) {
        return;
    }
    dev_widgets_t *D = lv_event_get_user_data(e);
    if (D && D->kb) {
        lv_keyboard_set_textarea(D->kb, NULL);
        lv_obj_add_flag(D->kb, LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_t *ta = lv_event_get_target(e);
    if (D && ta == D->ta_nome) {
        const char *v = lv_textarea_get_text(ta);
        str_copy_trunc(s_dev_robot_name, sizeof s_dev_robot_name, v);
        nvs_save_str(NVS_KEY_NAME, v);
    } else if (D && ta == D->ta_id) {
        const char *v = lv_textarea_get_text(ta);
        str_copy_trunc(s_dev_robot_code, sizeof s_dev_robot_code, v);
        nvs_save_str(NVS_KEY_CODE, v);
    }
}

static void dev_ta_activate_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_FOCUSED && code != LV_EVENT_CLICKED) {
        return;
    }
    dev_widgets_t *D = lv_event_get_user_data(e);
    if (D && D->kb) {
        lv_obj_t *ta = lv_event_get_target(e);
        lv_keyboard_set_textarea(D->kb, ta);
        lv_obj_clear_flag(D->kb, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(D->kb);
    }
}

static void dev_tabview_changed_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) {
        return;
    }
    dash_kb_undock_to_layer_top();
    os_close_modal();
}

static void on_login_btn_clicked(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    login_widgets_t *L = lv_event_get_user_data(e);
    if (!L || !L->ta_user || !L->ta_pass || !L->scr) {
        return;
    }

    lv_keyboard_set_textarea(L->kb, NULL);
    lv_obj_add_flag(L->kb, LV_OBJ_FLAG_HIDDEN);

    const char *u = lv_textarea_get_text(L->ta_user);
    const char *p = lv_textarea_get_text(L->ta_pass);
    if (strcmp(u, LOGIN_DEMO_USER) != 0 || strcmp(p, LOGIN_DEMO_PASS) != 0) {
        if (L->lbl_err) {
            lv_label_set_text(L->lbl_err, "Usuario ou senha incorretos.");
            lv_obj_clear_flag(L->lbl_err, LV_OBJ_FLAG_HIDDEN);
        }
        APP_WARN("Login falhou");
        return;
    }

    if (L->lbl_err) {
        lv_obj_add_flag(L->lbl_err, LV_OBJ_FLAG_HIDDEN);
    }
    APP_INFO("Login OK");

    s_logged_in = true;

    lv_obj_t *dash = lv_obj_create(NULL);
    build_dashboard_into(dash);
    lv_scr_load(dash);
    lv_obj_del(L->scr);
    memset(L, 0, sizeof(*L));
}

static void on_logout_clicked(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    lv_obj_t *dash_scr = lv_event_get_user_data(e);
    if (dash_scr == NULL) {
        return;
    }

    if (!lvgl_port_lock(0)) {
        APP_ERROR("Falha ao obter o lock do LVGL (logout)");
        return;
    }

    /* Marca a sessao como encerrada. NAO paramos o timer de separacao:
     * a OS continua a correr em segundo plano. Se estiver em pausa,
     * permanece em pausa (o estado e mantido nas variaveis globais). */
    s_logged_in = false;
    s_settings_dash_scr = NULL;
    memset(&s_settings_nav, 0, sizeof(s_settings_nav));

    historico_close_modal();
    os_close_modal();
    stepper_close_modal();
    dev_kb_destroy();

    if (s_dash_timer) {
        lv_timer_del(s_dash_timer);
        s_dash_timer = NULL;
    }
    if (s_cmd_timer) {
        lv_timer_del(s_cmd_timer);
        s_cmd_timer = NULL;
    }

    s_lbl_os_number     = NULL;
    s_lbl_os_client     = NULL;
    s_lbl_os_meds       = NULL;
    s_bar_os_meds       = NULL;
    s_lbl_os_eta        = NULL;
    s_lbl_os_eta_detail = NULL;
    s_hist_scroll       = NULL;
    /* Os widgets do dashboard serao destruidos com o ecra; anula os ponteiros
     * para que os callbacks em segundo plano (ex.: os_update_panels via
     * os_reset_idle) nao acedam a memoria ja libertada. */
    memset(&s_os, 0, sizeof(s_os));
    memset(&s_dev, 0, sizeof(s_dev));

    lv_obj_t *login_scr = lv_obj_create(NULL);
    build_login_ui_into(login_scr);
    lv_scr_load(login_scr);
    lv_obj_del(dash_scr);

    lv_obj_update_layout(login_scr);
    APP_INFO("Sessao terminada (logout)");
    lvgl_port_unlock();
}

static void ui_os_paint_labels(void)
{
    if (!s_lbl_os_number) {
        return;
    }

    if (s_lbl_os_number) {
        lv_label_set_text(s_lbl_os_number,
                          (s_os_active && s_os_num_str[0]) ? s_os_num_str : "--");
    }
    if (s_lbl_os_client) {
        lv_label_set_text(s_lbl_os_client,
                          (s_os_active && s_os_client_str[0]) ? s_os_client_str : "--");
    }
    if (s_lbl_os_meds) {
        if (s_os_active && s_os_meds_total > 0) {
            lv_label_set_text_fmt(s_lbl_os_meds, "%lu / %lu  separados",
                                  (unsigned long)s_os_meds_done, (unsigned long)s_os_meds_total);
        } else {
            lv_label_set_text(s_lbl_os_meds, "-- / --  separados");
        }
    }
    if (s_bar_os_meds) {
        int pct = 0;
        if (s_os_meds_total > 0) {
            pct = (int)((s_os_meds_done * 100ULL) / s_os_meds_total);
            if (pct > 100) {
                pct = 100;
            }
        }
        lv_bar_set_value(s_bar_os_meds, pct, LV_ANIM_OFF);
    }
    if (s_lbl_os_eta) {
        if (!s_os_active) {
            lv_label_set_text(s_lbl_os_eta, "--");
        } else if (s_os_meds_total <= s_os_meds_done) {
            lv_label_set_text(s_lbl_os_eta, "Concluido");
        } else if (s_os_pick_samples == 0) {
            lv_label_set_text(s_lbl_os_eta, "Estimativa: --");
        } else {
            uint32_t eta_s = os_eta_seconds();
            uint32_t m = eta_s / 60;
            uint32_t s = eta_s % 60;
            lv_label_set_text_fmt(s_lbl_os_eta, "~ %lum %02lus",
                                  (unsigned long)m, (unsigned long)s);
        }
    }
    if (s_lbl_os_eta_detail) {
        if (!s_os_active) {
            lv_label_set_text(s_lbl_os_eta_detail, "Sem OS em separacao.");
        } else if (s_os_pick_samples == 0) {
            lv_label_set_text(s_lbl_os_eta_detail,
                              "Media por remedio: aguardando\nas primeiras pegadas temporizadas.");
        } else {
            uint32_t avg = os_avg_pick_ms();
            unsigned long sec = (unsigned long)(avg / 1000);
            unsigned long d1 = (unsigned long)((avg % 1000) / 100);
            if (s_os_pick_samples == 1) {
                lv_label_set_text_fmt(s_lbl_os_eta_detail,
                                      "Media ~%lu,%lu s por unidade\n(1 pegada registada)",
                                      sec, d1);
            } else {
                lv_label_set_text_fmt(s_lbl_os_eta_detail,
                                      "Media ~%lu,%lu s por unidade\n(%lu pegadas registadas)",
                                      sec, d1, (unsigned long)s_os_pick_samples);
            }
        }
    }

    /* Botão pausar/retomar */
    if (s_os.btn_pause && s_os.lbl_btn_pause) {
        if (!s_os_active) {
            lv_obj_add_flag(s_os.btn_pause, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_clear_flag(s_os.btn_pause, LV_OBJ_FLAG_HIDDEN);
            if (s_os_paused) {
                lv_label_set_text(s_os.lbl_btn_pause, "Retomar OS");
                lv_obj_set_style_bg_color(s_os.btn_pause, lv_color_hex(0xD1FAE5), LV_PART_MAIN);
                lv_obj_set_style_bg_color(s_os.btn_pause, lv_color_hex(0xA7F3D0), LV_STATE_PRESSED);
                lv_obj_set_style_text_color(s_os.lbl_btn_pause, lv_color_hex(0x065F46), LV_PART_MAIN);
            } else {
                lv_label_set_text(s_os.lbl_btn_pause, "Pausar OS");
                lv_obj_set_style_bg_color(s_os.btn_pause, lv_color_hex(0xEDE8F8), LV_PART_MAIN);
                lv_obj_set_style_bg_color(s_os.btn_pause, lv_color_hex(0xD8D0F0), LV_STATE_PRESSED);
                lv_obj_set_style_text_color(s_os.lbl_btn_pause, lv_color_hex(0x3D3B7A), LV_PART_MAIN);
            }
        }
    }
}

static void dashboard_os_refresh(void)
{
    if (!s_lbl_os_number) {
        return;
    }

    static char     cached_num[OS_NUM_BUF_SZ]    = "";
    static char     cached_client[OS_CLIENT_BUF_SZ] = "";
    static uint32_t cached_done    = UINT32_MAX;
    static uint32_t cached_total   = UINT32_MAX;
    static uint64_t cached_sum     = UINT64_MAX;
    static uint32_t cached_samples = UINT32_MAX;

    bool dirty = (strcmp(cached_num, s_os_num_str) != 0) ||
                 (strcmp(cached_client, s_os_client_str) != 0) ||
                 (cached_done != s_os_meds_done) ||
                 (cached_total != s_os_meds_total) ||
                 (cached_sum != s_os_pick_ms_sum) ||
                 (cached_samples != s_os_pick_samples);

    if (!dirty) {
        return;
    }

    str_copy_trunc(cached_num, sizeof cached_num, s_os_num_str);
    str_copy_trunc(cached_client, sizeof cached_client, s_os_client_str);
    cached_done    = s_os_meds_done;
    cached_total   = s_os_meds_total;
    cached_sum     = s_os_pick_ms_sum;
    cached_samples = s_os_pick_samples;

    ui_os_paint_labels();
}

static void dev_kb_destroy(void)
{
    if (s_dev.kb) {
        lv_keyboard_set_textarea(s_dev.kb, NULL);
        lv_obj_del(s_dev.kb);
        s_dev.kb = NULL;
    }
}

static void dev_refresh_stats_labels(void)
{
    if (!s_dev.lbl_os_cnt || !s_dev.lbl_remed_cnt) {
        return;
    }
    static uint32_t prev_os = UINT32_MAX;
    static uint32_t prev_rm = UINT32_MAX;
    if (prev_os == s_dev_os_separadas && prev_rm == s_dev_remed_separados) {
        return;
    }
    prev_os = s_dev_os_separadas;
    prev_rm = s_dev_remed_separados;
    lv_label_set_text_fmt(s_dev.lbl_os_cnt, "%lu", (unsigned long)s_dev_os_separadas);
    lv_label_set_text_fmt(s_dev.lbl_remed_cnt, "%lu", (unsigned long)s_dev_remed_separados);
}

/* Atualiza os cards dinâmicos. So toca em widget quando o valor mudou,
 * para evitar invalidacoes desnecessarias do LVGL (cada redraw em DSI partial
 * dispara um draw_bitmap assincrono via DMA2D). */
static void dashboard_update_cb(lv_timer_t *t)
{
    (void)t;

    dev_refresh_stats_labels();
    dashboard_os_refresh();
}

/* Grelha 1 linha × 2 colunas (FR/FR) dentro de uma aba. */
static lv_obj_t *build_grid_1x2(lv_obj_t *parent, lv_obj_t **slot_left, lv_obj_t **slot_right)
{
    static lv_coord_t cols[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    static lv_coord_t rows[] = {LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};

    lv_obj_t *grid = lv_obj_create(parent);
    lv_obj_remove_style_all(grid);
    lv_obj_set_size(grid, lv_pct(100), lv_pct(100));
    lv_obj_set_flex_grow(grid, 1);
    lv_obj_set_layout(grid, LV_LAYOUT_GRID);
    lv_obj_set_grid_dsc_array(grid, cols, rows);
    lv_obj_set_style_pad_row(grid, 16, LV_PART_MAIN);
    lv_obj_set_style_pad_column(grid, 16, LV_PART_MAIN);
    lv_obj_clear_flag(grid, LV_OBJ_FLAG_SCROLLABLE);

    *slot_left = lv_obj_create(grid);
    lv_obj_remove_style_all(*slot_left);
    lv_obj_set_grid_cell(*slot_left, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_STRETCH, 0, 1);
    lv_obj_set_flex_flow(*slot_left, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(*slot_left, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(*slot_left, 0, LV_PART_MAIN);
    lv_obj_clear_flag(*slot_left, LV_OBJ_FLAG_SCROLLABLE);

    *slot_right = lv_obj_create(grid);
    lv_obj_remove_style_all(*slot_right);
    lv_obj_set_grid_cell(*slot_right, LV_GRID_ALIGN_STRETCH, 1, 1, LV_GRID_ALIGN_STRETCH, 0, 1);
    lv_obj_set_flex_flow(*slot_right, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(*slot_right, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(*slot_right, 0, LV_PART_MAIN);
    lv_obj_clear_flag(*slot_right, LV_OBJ_FLAG_SCROLLABLE);

    return grid;
}

/** Pagina de aba: 100%% altura/largura do content do tabview; filhos em coluna com stretch. */
static void style_dashboard_tab_page(lv_obj_t *tab)
{
    lv_obj_set_size(tab, lv_pct(100), lv_pct(100));
    lv_obj_set_flex_flow(tab, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(tab, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(tab, LV_OBJ_FLAG_SCROLLABLE);
}

static void style_tabview(lv_obj_t *tv)
{
    lv_obj_set_style_bg_color(tv, COLOR_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(tv, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_pad_all(tv, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(tv, 0, LV_PART_MAIN);

    lv_obj_t *tab_bar = lv_tabview_get_tab_btns(tv);
    lv_obj_set_style_bg_color(tab_bar, COLOR_CARD, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(tab_bar, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_side(tab_bar, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);
    lv_obj_set_style_border_color(tab_bar, COLOR_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_width(tab_bar, 1, LV_PART_MAIN);

    /* Itens normais */
    lv_obj_set_style_text_font(tab_bar, &lv_font_montserrat_24, LV_PART_ITEMS);
    lv_obj_set_style_text_color(tab_bar, COLOR_LABEL, LV_PART_ITEMS);
    lv_obj_set_style_bg_opa(tab_bar, LV_OPA_TRANSP, LV_PART_ITEMS);
    lv_obj_set_style_radius(tab_bar, 0, LV_PART_ITEMS);

    /* Item selecionado: texto escuro + sublinhado na cor ARIS */
    lv_obj_set_style_bg_opa(tab_bar, LV_OPA_TRANSP, LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_text_color(tab_bar, COLOR_VALUE, LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_border_side(tab_bar, LV_BORDER_SIDE_BOTTOM, LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_border_color(tab_bar, COLOR_ACCENT, LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_border_width(tab_bar, 3, LV_PART_ITEMS | LV_STATE_CHECKED);
}

static void historico_item_bubble_events(lv_obj_t *root)
{
    if (!root) {
        return;
    }
    lv_obj_add_flag(root, LV_OBJ_FLAG_EVENT_BUBBLE);
    uint32_t i = 0;
    lv_obj_t *c;
    while ((c = lv_obj_get_child(root, i)) != NULL) {
        i++;
        historico_item_bubble_events(c);
    }
}

static void os_pick_timer_stop(void)
{
    if (s_os_pick_timer) {
        lv_timer_del(s_os_pick_timer);
        s_os_pick_timer = NULL;
    }
}

static void os_close_modal(void)
{
    if (s_os.modal) {
        dash_kb_undock_to_layer_top();
        lv_obj_del(s_os.modal);
        s_os.modal = NULL;
        s_os.ta_num = NULL;
        s_os.ta_client = NULL;
        s_os.ta_total = NULL;
        s_os.ta_motivo = NULL;
    }
}

static void os_update_panels(void)
{
    if (!s_os.panel_idle || !s_os.panel_active) {
        return;
    }
    if (s_os_active) {
        lv_obj_add_flag(s_os.panel_idle, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_os.panel_active, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_clear_flag(s_os.panel_idle, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_os.panel_active, LV_OBJ_FLAG_HIDDEN);
    }
}

/* ====================================================================
 * Controle de LEDs via relé (GPIO 1 = amarelo, GPIO 2 = vermelho)
 * ==================================================================== */
static lv_timer_t *s_led_blink_timer = NULL;
static bool        s_led_blink_state = false;

/* Módulo de relé ativo em LOW: GPIO=0 aciona o relé, GPIO=1 libera. */
#define RELAY_ON  0
#define RELAY_OFF 1

static void led_gpio_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << LED_AMARELO_GPIO) | (1ULL << LED_VERMELHO_GPIO) |
                        (1ULL << STEPPER_STEP_GPIO) | (1ULL << STEPPER_DIR_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(LED_AMARELO_GPIO,  RELAY_OFF);
    gpio_set_level(LED_VERMELHO_GPIO, RELAY_OFF);
    gpio_set_level(STEPPER_STEP_GPIO, 0);
    gpio_set_level(STEPPER_DIR_GPIO,  0);
    APP_INFO("Relés GPIO ok (amarelo=%d, vermelho=%d, step=%d, dir=%d)",
             LED_AMARELO_GPIO, LED_VERMELHO_GPIO,
             STEPPER_STEP_GPIO, STEPPER_DIR_GPIO);
}

static void led_amarelo_set(bool on)
{
    gpio_set_level(LED_AMARELO_GPIO, on ? RELAY_ON : RELAY_OFF);
}

static void led_blink_timer_cb(lv_timer_t *t)
{
    (void)t;
    s_led_blink_state = !s_led_blink_state;
    gpio_set_level(LED_VERMELHO_GPIO, s_led_blink_state ? RELAY_ON : RELAY_OFF);
}

static void led_vermelho_blink_start(void)
{
    if (s_led_blink_timer) { return; }
    s_led_blink_state = true;
    gpio_set_level(LED_VERMELHO_GPIO, RELAY_ON);
    s_led_blink_timer = lv_timer_create(led_blink_timer_cb, 500, NULL);
}

static void led_vermelho_blink_stop(void)
{
    if (s_led_blink_timer) {
        lv_timer_del(s_led_blink_timer);
        s_led_blink_timer = NULL;
    }
    s_led_blink_state = false;
    gpio_set_level(LED_VERMELHO_GPIO, RELAY_OFF);
}

/* ====================================================================
 * Teste do motor de passo (STEP = GPIO 20, DIR = GPIO 32)
 * ==================================================================== */
typedef enum {
    STEPPER_CMD_STOP = 0,
    STEPPER_CMD_CW   = 1,
    STEPPER_CMD_CCW  = 2,
} stepper_cmd_t;

static volatile stepper_cmd_t s_stepper_cmd = STEPPER_CMD_STOP;
static esp_timer_handle_t s_stepper_timer = NULL;
static bool s_step_level = false;
static lv_obj_t *s_stepper_modal    = NULL;
static lv_obj_t *s_stepper_btn_cw   = NULL;
static lv_obj_t *s_stepper_btn_ccw  = NULL;
static lv_obj_t *s_lbl_stepper_run  = NULL;
/* Widgets da seção de configuração (criados em stepper_open_cb) */
static lv_obj_t *s_stepper_lbl_rpm  = NULL;  /* ex.: "60 RPM"          */
static lv_obj_t *s_stepper_sld_rpm  = NULL;  /* slider 1..300          */
static lv_obj_t *s_stepper_lbl_spr  = NULL;  /* ex.: "200 passos/rev"  */
static lv_obj_t *s_stepper_lbl_ms   = NULL;  /* ex.: "1x"              */
static lv_obj_t *s_stepper_lbl_info = NULL;  /* passos/s + pulso µs    */

/* Calcula o semiperiodo em µs a partir das configs atuais.
 * half_period_us = 500 000 / (RPM/60 * SPR * microstep)
 *                = 30 000 000 / (RPM * SPR * microstep)               */
static uint32_t stepper_calc_pulse_us(void)
{
    uint8_t  ms  = MICROSTEP_OPTIONS[s_stepper_ms_idx];
    uint32_t sps = (uint32_t)s_stepper_rpm * s_stepper_steps_rev * ms / 60u;
    if (sps == 0) { return 10000u; }
    uint32_t half_us = 500000UL / sps;
    if (half_us < 20u) { half_us = 20u; }   /* ≤ 25 000 passos/s máx  */
    return half_us;
}

/* Atualiza o label de informação calculada */
static void stepper_refresh_info(void)
{
    if (!s_stepper_lbl_info) { return; }
    uint8_t  ms   = MICROSTEP_OPTIONS[s_stepper_ms_idx];
    uint32_t sps  = (uint32_t)s_stepper_rpm * s_stepper_steps_rev * ms / 60u;
    uint32_t pulse = stepper_calc_pulse_us();
    lv_label_set_text_fmt(s_stepper_lbl_info,
        "%lu passos/s  |  Pulso: %lu µs", (unsigned long)sps, (unsigned long)pulse);
}

static void stepper_timer_cb(void *arg)
{
    (void)arg;
    if (s_stepper_cmd == STEPPER_CMD_STOP) {
        if (s_step_level) {
            gpio_set_level(STEPPER_STEP_GPIO, 0);
            s_step_level = false;
        }
        return;
    }
    s_step_level = !s_step_level;
    gpio_set_level(STEPPER_STEP_GPIO, s_step_level ? 1 : 0);
}

static void stepper_stop(void)
{
    s_stepper_cmd = STEPPER_CMD_STOP;
    if (s_stepper_timer) {
        esp_timer_stop(s_stepper_timer);
    }
    s_step_level = false;
    gpio_set_level(STEPPER_STEP_GPIO, 0);
}

static void stepper_run(stepper_cmd_t cmd)
{
    if (cmd == STEPPER_CMD_STOP) {
        stepper_stop();
        return;
    }
    gpio_set_level(STEPPER_DIR_GPIO, (cmd == STEPPER_CMD_CW) ? 1 : 0);
    s_stepper_cmd = cmd;
    if (!s_stepper_timer) {
        return;
    }
    /* Para completamente antes de iniciar (garante período limpo) */
    esp_timer_stop(s_stepper_timer);
    s_step_level = false;
    gpio_set_level(STEPPER_STEP_GPIO, 0);
    esp_timer_start_periodic(s_stepper_timer, stepper_calc_pulse_us());
}

static void stepper_init(void)
{
    const esp_timer_create_args_t args = {
        .callback = &stepper_timer_cb,
        .name     = "stepper",
    };
    if (esp_timer_create(&args, &s_stepper_timer) != ESP_OK) {
        APP_ERROR("Falha ao criar timer do stepper");
        s_stepper_timer = NULL;
        return;
    }
    stepper_stop();
    APP_INFO("Stepper timer ok (STEP=%d DIR=%d)", (int)STEPPER_STEP_GPIO, (int)STEPPER_DIR_GPIO);
}

static void stepper_style_dir_btn(lv_obj_t *btn, bool active)
{
    if (!btn) {
        return;
    }
    if (active) {
        lv_obj_set_style_bg_color(btn, COLOR_OK, LV_PART_MAIN);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x059669), LV_STATE_PRESSED);
        lv_obj_t *lbl = lv_obj_get_child(btn, 0);
        if (lbl) {
            lv_obj_set_style_text_color(lbl, lv_color_white(), LV_PART_MAIN);
        }
    } else {
        lv_obj_set_style_bg_color(btn, COLOR_BADGE_BG, LV_PART_MAIN);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0xD8D8EC), LV_STATE_PRESSED);
        lv_obj_t *lbl = lv_obj_get_child(btn, 0);
        if (lbl) {
            lv_obj_set_style_text_color(lbl, COLOR_VALUE, LV_PART_MAIN);
        }
    }
}

static void stepper_ui_refresh(void)
{
    stepper_cmd_t cmd = s_stepper_cmd;
    if (s_lbl_stepper_run) {
        if (cmd == STEPPER_CMD_CW) {
            lv_label_set_text(s_lbl_stepper_run, "A rodar no sentido horario");
            lv_obj_set_style_text_color(s_lbl_stepper_run, COLOR_OK, LV_PART_MAIN);
        } else if (cmd == STEPPER_CMD_CCW) {
            lv_label_set_text(s_lbl_stepper_run, "A rodar no sentido anti-horario");
            lv_obj_set_style_text_color(s_lbl_stepper_run, COLOR_OK, LV_PART_MAIN);
        } else {
            lv_label_set_text(s_lbl_stepper_run, "Parado — escolha um sentido");
            lv_obj_set_style_text_color(s_lbl_stepper_run, COLOR_LABEL, LV_PART_MAIN);
        }
    }
    stepper_style_dir_btn(s_stepper_btn_cw,  cmd == STEPPER_CMD_CW);
    stepper_style_dir_btn(s_stepper_btn_ccw, cmd == STEPPER_CMD_CCW);
}

/* ---------- helpers de config do stepper ---------------------------------- */

static void stepper_save_config(void)
{
    nvs_save_u16(NVS_KEY_STEP_RPM,  s_stepper_rpm);
    nvs_save_u16(NVS_KEY_STEP_SPR,  s_stepper_steps_rev);
    nvs_save_u8 (NVS_KEY_STEP_MSTP, s_stepper_ms_idx);
}

static void stepper_refresh_rpm_label(void)
{
    if (s_stepper_lbl_rpm) {
        lv_label_set_text_fmt(s_stepper_lbl_rpm, "%u RPM", (unsigned)s_stepper_rpm);
    }
    if (s_stepper_sld_rpm) {
        lv_slider_set_value(s_stepper_sld_rpm, (int32_t)s_stepper_rpm, LV_ANIM_OFF);
    }
}

static void stepper_refresh_spr_label(void)
{
    if (s_stepper_lbl_spr) {
        lv_label_set_text_fmt(s_stepper_lbl_spr, "%u", (unsigned)s_stepper_steps_rev);
    }
}

static void stepper_refresh_ms_label(void)
{
    if (s_stepper_lbl_ms) {
        lv_label_set_text_fmt(s_stepper_lbl_ms, "%ux", (unsigned)MICROSTEP_OPTIONS[s_stepper_ms_idx]);
    }
}

/* Reaplica a velocidade ao timer se o motor estiver a correr */
static void stepper_apply_config(void)
{
    stepper_refresh_rpm_label();
    stepper_refresh_spr_label();
    stepper_refresh_ms_label();
    stepper_refresh_info();
    stepper_save_config();
    if (s_stepper_cmd != STEPPER_CMD_STOP && s_stepper_timer) {
        /* esp_timer_restart troca o período de forma atômica sem parar o timer */
        esp_err_t err = esp_timer_restart(s_stepper_timer, stepper_calc_pulse_us());
        if (err != ESP_OK) {
            /* fallback: para e reinicia */
            esp_timer_stop(s_stepper_timer);
            esp_timer_start_periodic(s_stepper_timer, stepper_calc_pulse_us());
        }
    }
}

/* Callback do slider de RPM */
static void stepper_rpm_slider_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) { return; }
    lv_obj_t *sld = lv_event_get_target(e);
    int32_t v = lv_slider_get_value(sld);
    if (v < 1)    { v = 1; }
    if (v > 2000) { v = 2000; }
    s_stepper_rpm = (uint16_t)v;
    stepper_apply_config();
}

/* Callback dos botões +/- de passos por revolução */
static void stepper_spr_btn_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) { return; }
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    /* Valores típicos por passo: 50 → 100 → 200 → 400 → 800 */
    static const uint16_t SPR_PRESETS[] = {50, 100, 200, 400, 800, 1600};
    static const int SPR_COUNT = (int)(sizeof(SPR_PRESETS)/sizeof(SPR_PRESETS[0]));
    /* Encontra o preset mais próximo e avança */
    int idx = 0;
    int best_dist = 9999;
    for (int i = 0; i < SPR_COUNT; i++) {
        int d = (int)SPR_PRESETS[i] - (int)s_stepper_steps_rev;
        if (d < 0) { d = -d; }
        if (d < best_dist) { best_dist = d; idx = i; }
    }
    idx += delta;
    if (idx < 0) { idx = 0; }
    if (idx >= SPR_COUNT) { idx = SPR_COUNT - 1; }
    s_stepper_steps_rev = SPR_PRESETS[idx];
    stepper_apply_config();
}

/* Callback dos botões +/- de microstepping */
static void stepper_ms_btn_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) { return; }
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    int idx = (int)s_stepper_ms_idx + delta;
    if (idx < 0) { idx = 0; }
    if (idx >= MICROSTEP_COUNT) { idx = MICROSTEP_COUNT - 1; }
    s_stepper_ms_idx = (uint8_t)idx;
    stepper_apply_config();
}

/* Cria um painel de parâmetro com label de título, valor central e botões +/- */
static lv_obj_t *stepper_make_param_panel(lv_obj_t *parent, const char *title,
                                           lv_obj_t **out_val_lbl,
                                           lv_event_cb_t btn_cb)
{
    lv_obj_t *panel = lv_obj_create(parent);
    lv_obj_remove_style_all(panel);
    lv_obj_set_style_bg_color(panel, lv_color_hex(0xF1F0F8), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(panel, 12, LV_PART_MAIN);
    lv_obj_set_style_pad_all(panel, 10, LV_PART_MAIN);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(panel, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(panel, 6, LV_PART_MAIN);
    lv_obj_set_flex_grow(panel, 1);
    lv_obj_set_height(panel, LV_SIZE_CONTENT);

    lv_obj_t *lbl_title = lv_label_create(panel);
    lv_label_set_text(lbl_title, title);
    lv_obj_set_style_text_font(lbl_title, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_title, COLOR_LABEL, LV_PART_MAIN);

    lv_obj_t *row = lv_obj_create(panel);
    lv_obj_remove_style_all(row);
    lv_obj_set_width(row, lv_pct(100));
    lv_obj_set_height(row, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(row, 8, LV_PART_MAIN);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *btn_minus = lv_btn_create(row);
    lv_obj_set_size(btn_minus, 48, 48);
    lv_obj_set_style_radius(btn_minus, 10, LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn_minus, COLOR_ACCENT, LV_PART_MAIN);
    lv_obj_add_event_cb(btn_minus, btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)-1);
    lv_obj_t *lm = lv_label_create(btn_minus);
    lv_label_set_text(lm, "-");
    lv_obj_set_style_text_font(lm, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(lm, lv_color_white(), LV_PART_MAIN);
    lv_obj_center(lm);

    lv_obj_t *val = lv_label_create(row);
    lv_obj_set_flex_grow(val, 1);
    lv_obj_set_style_text_font(val, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(val, COLOR_VALUE, LV_PART_MAIN);
    lv_obj_set_style_text_align(val, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_text(val, "---");
    if (out_val_lbl) { *out_val_lbl = val; }

    lv_obj_t *btn_plus = lv_btn_create(row);
    lv_obj_set_size(btn_plus, 48, 48);
    lv_obj_set_style_radius(btn_plus, 10, LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn_plus, COLOR_ACCENT, LV_PART_MAIN);
    lv_obj_add_event_cb(btn_plus, btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)+1);
    lv_obj_t *lp = lv_label_create(btn_plus);
    lv_label_set_text(lp, "+");
    lv_obj_set_style_text_font(lp, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(lp, lv_color_white(), LV_PART_MAIN);
    lv_obj_center(lp);

    return panel;
}

/* -------------------------------------------------------------------------- */

static void stepper_close_modal(void)
{
    stepper_stop();
    s_stepper_btn_cw   = NULL;
    s_stepper_btn_ccw  = NULL;
    s_lbl_stepper_run  = NULL;
    s_stepper_lbl_rpm  = NULL;
    s_stepper_sld_rpm  = NULL;
    s_stepper_lbl_spr  = NULL;
    s_stepper_lbl_ms   = NULL;
    s_stepper_lbl_info = NULL;
    if (s_stepper_modal) {
        lv_obj_del(s_stepper_modal);
        s_stepper_modal = NULL;
    }
}

static void stepper_voltar_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    stepper_close_modal();
}

static void stepper_dir_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    stepper_cmd_t cmd = (stepper_cmd_t)(uintptr_t)lv_event_get_user_data(e);
    if (s_stepper_cmd == cmd) {
        stepper_stop();
    } else {
        stepper_run(cmd);
        APP_INFO("Stepper %s (STEP=%d DIR=%d)",
                 (cmd == STEPPER_CMD_CW) ? "horario" : "anti-horario",
                 (int)STEPPER_STEP_GPIO, (int)STEPPER_DIR_GPIO);
    }
    stepper_ui_refresh();
}

static void stepper_parar_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    stepper_stop();
    stepper_ui_refresh();
    APP_INFO("Stepper parado");
}

static void stepper_open_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    stepper_close_modal();

    lv_obj_t *backdrop = lv_obj_create(lv_layer_top());
    s_stepper_modal = backdrop;
    lv_obj_remove_style_all(backdrop);
    lv_obj_set_size(backdrop, lv_pct(100), lv_pct(100));
    lv_obj_set_style_bg_color(backdrop, COLOR_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(backdrop, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_pad_all(backdrop, 16, LV_PART_MAIN);
    lv_obj_set_flex_flow(backdrop, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(backdrop, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(backdrop, 12, LV_PART_MAIN);
    lv_obj_clear_flag(backdrop, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *hdr = lv_obj_create(backdrop);
    lv_obj_remove_style_all(hdr);
    lv_obj_set_width(hdr, lv_pct(100));
    lv_obj_set_height(hdr, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(hdr, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(hdr, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(hdr, 12, LV_PART_MAIN);

    lv_obj_t *b_back = lv_btn_create(hdr);
    lv_obj_set_height(b_back, 48);
    lv_obj_set_style_radius(b_back, 12, LV_PART_MAIN);
    lv_obj_set_style_bg_color(b_back, COLOR_BADGE_BG, LV_PART_MAIN);
    lv_obj_add_event_cb(b_back, stepper_voltar_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l_back = lv_label_create(b_back);
    lv_label_set_text(l_back, "Voltar");
    lv_obj_set_style_text_font(l_back, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(l_back, COLOR_VALUE, LV_PART_MAIN);
    lv_obj_center(l_back);

    lv_obj_t *tit = lv_label_create(hdr);
    lv_label_set_text(tit, "Teste do motor de passo");
    lv_obj_set_style_text_font(tit, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(tit, COLOR_LABEL, LV_PART_MAIN);
    lv_obj_set_flex_grow(tit, 1);

    /* ---- Card de configuração -------------------------------------------- */
    lv_obj_t *cfg_card = make_card(backdrop);
    lv_obj_set_width(cfg_card, lv_pct(100));
    lv_obj_set_height(cfg_card, LV_SIZE_CONTENT);

    make_card_label(cfg_card, "CONFIGURAÇÃO");

    /* Linha superior: RPM | Passos/Rev | Microstepping */
    lv_obj_t *params_row = lv_obj_create(cfg_card);
    lv_obj_remove_style_all(params_row);
    lv_obj_set_width(params_row, lv_pct(100));
    lv_obj_set_height(params_row, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(params_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(params_row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_column(params_row, 10, LV_PART_MAIN);
    lv_obj_clear_flag(params_row, LV_OBJ_FLAG_SCROLLABLE);

    /* --- Painel RPM --- */
    lv_obj_t *rpm_panel = lv_obj_create(params_row);
    lv_obj_remove_style_all(rpm_panel);
    lv_obj_set_style_bg_color(rpm_panel, lv_color_hex(0xF1F0F8), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(rpm_panel, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(rpm_panel, 12, LV_PART_MAIN);
    lv_obj_set_style_pad_all(rpm_panel, 10, LV_PART_MAIN);
    lv_obj_set_flex_grow(rpm_panel, 2);
    lv_obj_set_height(rpm_panel, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(rpm_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(rpm_panel, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(rpm_panel, 6, LV_PART_MAIN);

    lv_obj_t *lbl_rpm_title = lv_label_create(rpm_panel);
    lv_label_set_text(lbl_rpm_title, "Velocidade");
    lv_obj_set_style_text_font(lbl_rpm_title, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_rpm_title, COLOR_LABEL, LV_PART_MAIN);

    s_stepper_lbl_rpm = lv_label_create(rpm_panel);
    lv_obj_set_style_text_font(s_stepper_lbl_rpm, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_stepper_lbl_rpm, COLOR_VALUE, LV_PART_MAIN);

    s_stepper_sld_rpm = lv_slider_create(rpm_panel);
    lv_obj_set_width(s_stepper_sld_rpm, lv_pct(100));
    lv_slider_set_range(s_stepper_sld_rpm, 1, 2000);
    lv_slider_set_value(s_stepper_sld_rpm, (int32_t)s_stepper_rpm, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_stepper_sld_rpm, COLOR_ACCENT, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s_stepper_sld_rpm, COLOR_ACCENT_DARK, LV_PART_KNOB);
    lv_obj_add_event_cb(s_stepper_sld_rpm, stepper_rpm_slider_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* --- Painel Passos/Rev --- */
    stepper_make_param_panel(params_row, "Passos/Rev", &s_stepper_lbl_spr, stepper_spr_btn_cb);

    /* --- Painel Microstepping --- */
    stepper_make_param_panel(params_row, "Microstepping", &s_stepper_lbl_ms, stepper_ms_btn_cb);

    /* Linha de info calculada */
    s_stepper_lbl_info = lv_label_create(cfg_card);
    lv_obj_set_style_text_font(s_stepper_lbl_info, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_stepper_lbl_info, COLOR_LABEL, LV_PART_MAIN);
    lv_obj_set_width(s_stepper_lbl_info, lv_pct(100));
    lv_obj_set_style_text_align(s_stepper_lbl_info, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);

    /* Popula labels com valores atuais */
    stepper_refresh_rpm_label();
    stepper_refresh_spr_label();
    stepper_refresh_ms_label();
    stepper_refresh_info();

    /* ---- Card de controlo ------------------------------------------------- */
    lv_obj_t *card = make_card(backdrop);
    lv_obj_set_width(card, lv_pct(100));
    lv_obj_set_flex_grow(card, 1);
    lv_obj_set_flex_align(card, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    make_card_label(card, "SENTIDO");
    lv_obj_t *hint = lv_label_create(card);
    lv_label_set_text(hint, "Toque num botao para rodar. Toque de novo para parar.");
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(hint, COLOR_LABEL, LV_PART_MAIN);

    s_lbl_stepper_run = lv_label_create(card);
    lv_obj_set_style_text_font(s_lbl_stepper_run, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_width(s_lbl_stepper_run, lv_pct(100));
    lv_obj_set_style_text_align(s_lbl_stepper_run, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);

    lv_obj_t *row = lv_obj_create(card);
    lv_obj_remove_style_all(row);
    lv_obj_set_width(row, lv_pct(100));
    lv_obj_set_height(row, 80);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(row, 16, LV_PART_MAIN);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    s_stepper_btn_ccw = lv_btn_create(row);
    lv_obj_set_flex_grow(s_stepper_btn_ccw, 1);
    lv_obj_set_height(s_stepper_btn_ccw, 72);
    lv_obj_set_style_radius(s_stepper_btn_ccw, 14, LV_PART_MAIN);
    lv_obj_add_event_cb(s_stepper_btn_ccw, stepper_dir_cb, LV_EVENT_CLICKED,
                        (void *)(uintptr_t)STEPPER_CMD_CCW);
    lv_obj_t *lccw = lv_label_create(s_stepper_btn_ccw);
    lv_label_set_text(lccw, "Anti-horario");
    lv_obj_set_style_text_font(lccw, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_center(lccw);

    s_stepper_btn_cw = lv_btn_create(row);
    lv_obj_set_flex_grow(s_stepper_btn_cw, 1);
    lv_obj_set_height(s_stepper_btn_cw, 72);
    lv_obj_set_style_radius(s_stepper_btn_cw, 14, LV_PART_MAIN);
    lv_obj_add_event_cb(s_stepper_btn_cw, stepper_dir_cb, LV_EVENT_CLICKED,
                        (void *)(uintptr_t)STEPPER_CMD_CW);
    lv_obj_t *lcw = lv_label_create(s_stepper_btn_cw);
    lv_label_set_text(lcw, "Horario");
    lv_obj_set_style_text_font(lcw, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_center(lcw);

    lv_obj_t *b_stop = lv_btn_create(card);
    lv_obj_set_width(b_stop, lv_pct(100));
    lv_obj_set_height(b_stop, 56);
    lv_obj_set_style_radius(b_stop, 14, LV_PART_MAIN);
    lv_obj_set_style_bg_color(b_stop, lv_color_hex(0xFEE2E2), LV_PART_MAIN);
    lv_obj_set_style_bg_color(b_stop, lv_color_hex(0xFECACA), LV_STATE_PRESSED);
    lv_obj_add_event_cb(b_stop, stepper_parar_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lstop = lv_label_create(b_stop);
    lv_label_set_text(lstop, "Parar");
    lv_obj_set_style_text_font(lstop, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(lstop, lv_color_hex(0xB91C1C), LV_PART_MAIN);
    lv_obj_center(lstop);

    stepper_ui_refresh();
    lv_obj_move_foreground(backdrop);
}

/* ==================================================================== */

static void os_reset_idle(void)
{
    os_pick_timer_stop();
    s_os_active = false;
    s_os_paused = false;
    s_os_num_str[0]       = '\0';
    s_os_client_str[0]    = '\0';
    s_os_meds_total       = 0;
    s_os_meds_done        = 0;
    s_os_pick_ms_sum      = 0;
    s_os_pick_samples     = 0;
    s_os_sess_start_us    = 0;
    led_amarelo_set(false);
    led_vermelho_blink_stop();
    os_update_panels();
    ui_os_paint_labels();
}

/* ====================================================================
 * Bridge serial — envia eventos JSON para o gateway Python no PC.
 * Cada linha começa com "BRIDGE:" para que o gateway filtre facilmente
 * o output normal do ESP-IDF logger.
 * ==================================================================== */
#ifndef BRIDGE_ROBOT_CODE
#define BRIDGE_ROBOT_CODE  "ESP32-P4-001"
#endif

static void serial_bridge_escape(char *dst, size_t cap, const char *src)
{
    size_t di = 0;
    for (size_t i = 0; src && src[i] && di + 2 < cap; i++) {
        if (src[i] == '"' || src[i] == '\\') {
            if (di + 3 < cap) { dst[di++] = '\\'; dst[di++] = src[i]; }
        } else if (src[i] == '\n') {
            if (di + 3 < cap) { dst[di++] = '\\'; dst[di++] = 'n'; }
        } else {
            dst[di++] = src[i];
        }
    }
    dst[di] = '\0';
}

static void serial_bridge_os_start(const char *num, const char *client, uint32_t total)
{
    if (s_bridge_suppress) { return; }
    char en[OS_NUM_BUF_SZ * 2];
    char ec[OS_CLIENT_BUF_SZ * 2];
    serial_bridge_escape(en, sizeof en, num);
    serial_bridge_escape(ec, sizeof ec, client);
    printf("BRIDGE:{\"ev\":\"os_start\",\"robot\":\"%s\",\"num\":\"%s\",\"client\":\"%s\",\"total\":%lu}\n",
           BRIDGE_ROBOT_CODE, en, ec, (unsigned long)total);
}

static void serial_bridge_unit(const char *num, uint32_t done)
{
    if (s_bridge_suppress) { return; }
    char en[OS_NUM_BUF_SZ * 2];
    serial_bridge_escape(en, sizeof en, num);
    printf("BRIDGE:{\"ev\":\"unit\",\"robot\":\"%s\",\"num\":\"%s\",\"done\":%lu}\n",
           BRIDGE_ROBOT_CODE, en, (unsigned long)done);
}

static void serial_bridge_os_complete(const char *num, uint32_t done)
{
    if (s_bridge_suppress) { return; }
    char en[OS_NUM_BUF_SZ * 2];
    serial_bridge_escape(en, sizeof en, num);
    printf("BRIDGE:{\"ev\":\"os_complete\",\"robot\":\"%s\",\"num\":\"%s\",\"done\":%lu}\n",
           BRIDGE_ROBOT_CODE, en, (unsigned long)done);
}

static void serial_bridge_os_cancel(const char *num, uint32_t done, const char *reason)
{
    if (s_bridge_suppress) { return; }
    char en[OS_NUM_BUF_SZ * 2];
    char er[OS_HIST_MOTIVO_SZ * 2];
    serial_bridge_escape(en, sizeof en, num);
    serial_bridge_escape(er, sizeof er, reason);
    printf("BRIDGE:{\"ev\":\"os_cancel\",\"robot\":\"%s\",\"num\":\"%s\",\"done\":%lu,\"reason\":\"%s\"}\n",
           BRIDGE_ROBOT_CODE, en, (unsigned long)done, er);
}

static void serial_bridge_pong(void)
{
    const char *raw_name = s_dev_robot_name[0] ? s_dev_robot_name : "Separador P4";
    const char *raw_id   = s_dev_robot_code[0] ? s_dev_robot_code : BRIDGE_ROBOT_CODE;
    char en[DEV_ROBOT_NAME_SZ * 2];
    char eid[DEV_ROBOT_ID_SZ * 2];
    serial_bridge_escape(en,  sizeof en,  raw_name);
    serial_bridge_escape(eid, sizeof eid, raw_id);
    printf("BRIDGE:{\"ev\":\"pong\",\"robot\":\"%s\",\"name\":\"%s\"}\n", eid, en);
    APP_INFO("[BRIDGE pong] robot=%s name=%s", raw_id, raw_name);
}

static void serial_bridge_os_pause(const char *num)
{
    if (s_bridge_suppress) { return; }
    char en[OS_NUM_BUF_SZ * 2];
    serial_bridge_escape(en, sizeof en, num);
    printf("BRIDGE:{\"ev\":\"os_pause\",\"num\":\"%s\"}\n", en);
    APP_INFO("[BRIDGE os_pause] %s", num);
}

static void serial_bridge_os_resume(const char *num)
{
    if (s_bridge_suppress) { return; }
    char en[OS_NUM_BUF_SZ * 2];
    serial_bridge_escape(en, sizeof en, num);
    printf("BRIDGE:{\"ev\":\"os_resume\",\"num\":\"%s\"}\n", en);
    APP_INFO("[BRIDGE os_resume] %s", num);
}

/* ====================================================================
 * Recepção de comandos do gateway (Web → ESP32)
 * O gateway escreve linhas "CMD:{json}" na porta serial.
 * A task serial_cmd_task lê stdin e coloca na fila s_cmd_queue.
 * O timer serial_cmd_timer_cb processa a fila dentro do contexto LVGL.
 * ==================================================================== */

static void serial_cmd_task(void *arg)
{
    (void)arg;
    static char buf[512];
    while (1) {
        if (!fgets(buf, sizeof(buf), stdin)) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        /* Localiza o prefixo CMD: na linha (pode haver lixo antes). */
        char *p = strstr(buf, "CMD:");
        if (!p) { continue; }
        p += 4; /* aponta para o JSON */

        /* Remove \r\n do final. */
        size_t len = strlen(p);
        while (len > 0 && (p[len - 1] == '\r' || p[len - 1] == '\n')) {
            p[--len] = '\0';
        }
        if (len == 0 || !s_cmd_queue) { continue; }

        /* Parse manual mínimo: extrai "cmd", "num", "client", "total", "reason". */
        serial_cmd_t cmd;
        memset(&cmd, 0, sizeof(cmd));

        /* Determina tipo pelo campo "cmd". */
        if (strstr(p, "\"os_assign\""))       { cmd.type = CMD_OS_ASSIGN;   }
        else if (strstr(p, "\"os_complete\"")) { cmd.type = CMD_OS_COMPLETE; }
        else if (strstr(p, "\"os_cancel\""))   { cmd.type = CMD_OS_CANCEL;   }
        else if (strstr(p, "\"os_pause\""))    { cmd.type = CMD_OS_PAUSE;    }
        else if (strstr(p, "\"os_resume\""))   { cmd.type = CMD_OS_RESUME;   }
        else if (strstr(p, "\"ping\""))        { cmd.type = CMD_PING;        }
        else if (strstr(p, "\"set_device\"")) { cmd.type = CMD_SET_DEVICE;  }
        else { continue; }

        /* Extrai strings: procura "chave":"valor"
         * "name" e "code" são aliases usados pelo CMD:set_device */
        const char *fields[] = { "num", "client", "reason", "code", "name" };
        char       *dsts[]   = { cmd.num, cmd.client, cmd.reason, cmd.num, cmd.client };
        size_t      caps[]   = { sizeof(cmd.num), sizeof(cmd.client), sizeof(cmd.reason),
                                 sizeof(cmd.num), sizeof(cmd.client) };
        for (int fi = 0; fi < 5; fi++) {
            char key[32];
            snprintf(key, sizeof(key), "\"%s\":\"", fields[fi]);
            char *start = strstr(p, key);
            if (!start) { continue; }
            start += strlen(key);
            char *end = strchr(start, '"');
            if (!end) { continue; }
            size_t n = (size_t)(end - start);
            if (n >= caps[fi]) { n = caps[fi] - 1; }
            memcpy(dsts[fi], start, n);
            dsts[fi][n] = '\0';
        }

        /* Extrai "total": numero inteiro */
        {
            char *t = strstr(p, "\"total\":");
            if (t) {
                t += 8;
                cmd.total = (uint32_t)strtoul(t, NULL, 10);
            }
        }

        xQueueSend(s_cmd_queue, &cmd, 0);
        APP_INFO("[CMD RX] tipo=%d num='%s'", cmd.type, cmd.num);
    }
}

/** Executado no contexto LVGL (timer 200 ms): drena a fila de comandos. */
static void serial_cmd_timer_cb(lv_timer_t *t)
{
    (void)t;
    if (!s_cmd_queue) { return; }
    serial_cmd_t cmd;
    while (xQueueReceive(s_cmd_queue, &cmd, 0) == pdTRUE) {
        s_bridge_suppress = true;  /* nao reemitir o evento de volta */

        if (cmd.type == CMD_OS_ASSIGN) {
            if (!s_os_active && cmd.num[0] && cmd.total > 0) {
                APP_INFO("[CMD] os_assign: %s / %s / %lu", cmd.num, cmd.client, (unsigned long)cmd.total);
                os_start_manual(cmd.num, cmd.client, cmd.total);
            } else {
                APP_WARN("[CMD] os_assign ignorado (OS ja ativa ou dados invalidos)");
            }
        } else if (cmd.type == CMD_OS_COMPLETE) {
            if (s_os_active) {
                APP_INFO("[CMD] os_complete: %s", cmd.num);
                os_finalize_concluida();
            }
        } else if (cmd.type == CMD_OS_CANCEL) {
            if (s_os_active) {
                APP_INFO("[CMD] os_cancel: %s motivo='%s'", cmd.num, cmd.reason);
                os_pick_timer_stop();
                os_finalize_cancelada(cmd.reason[0] ? cmd.reason : "Cancelado pelo painel web");
            }
        } else if (cmd.type == CMD_OS_PAUSE) {
            if (s_os_active && !s_os_paused) {
                APP_INFO("[CMD] os_pause: %s", cmd.num);
                os_pause();
            }
        } else if (cmd.type == CMD_OS_RESUME) {
            if (s_os_active && s_os_paused) {
                APP_INFO("[CMD] os_resume: %s", cmd.num);
                os_resume();
            }
        } else if (cmd.type == CMD_PING) {
            APP_INFO("[CMD] ping recebido — respondendo com pong");
            serial_bridge_pong();
        } else if (cmd.type == CMD_SET_DEVICE) {
            APP_INFO("[CMD] set_device: name='%s' code='%s'", cmd.client, cmd.num);
            if (cmd.client[0]) {
                str_copy_trunc(s_dev_robot_name, sizeof s_dev_robot_name, cmd.client);
                nvs_save_str(NVS_KEY_NAME, s_dev_robot_name);
                if (s_dev.ta_nome) {
                    lv_textarea_set_text(s_dev.ta_nome, s_dev_robot_name);
                }
            }
            if (cmd.num[0]) {
                str_copy_trunc(s_dev_robot_code, sizeof s_dev_robot_code, cmd.num);
                nvs_save_str(NVS_KEY_CODE, s_dev_robot_code);
                if (s_dev.ta_id) {
                    lv_textarea_set_text(s_dev.ta_id, s_dev_robot_code);
                }
            }
        }

        s_bridge_suppress = false;
    }
}

/* ================================================================= */

static void historico_push(const char *numero, const char *cliente, uint32_t separados, uint32_t total,
                           uint32_t duracao_ms, os_hist_estado_t estado, const char *motivo)
{
    uint32_t d = duracao_ms;
    if (d == 0 && s_os_sess_start_us > 0) {
        uint64_t now = esp_timer_get_time();
        if (now > s_os_sess_start_us) {
            uint64_t el_ms = (now - s_os_sess_start_us) / 1000ULL;
            d              = (el_ms > UINT32_MAX) ? UINT32_MAX : (uint32_t)el_ms;
        }
    }
    if (s_os_hist_count >= OS_HIST_MAX) {
        memmove(&s_os_hist[1], &s_os_hist[0], (size_t)(OS_HIST_MAX - 1) * sizeof(s_os_hist[0]));
        s_os_hist_count = OS_HIST_MAX;
    } else {
        memmove(&s_os_hist[1], &s_os_hist[0], (size_t)s_os_hist_count * sizeof(s_os_hist[0]));
        s_os_hist_count++;
    }
    str_copy_trunc(s_os_hist[0].numero, sizeof s_os_hist[0].numero, numero);
    str_copy_trunc(s_os_hist[0].cliente, sizeof s_os_hist[0].cliente, cliente);
    s_os_hist[0].separados  = separados;
    s_os_hist[0].total      = total;
    s_os_hist[0].duracao_ms = d;
    s_os_hist[0].estado     = (uint8_t)estado;
    if (estado == OS_HIST_CANCELADA && motivo) {
        str_copy_trunc(s_os_hist[0].motivo, sizeof s_os_hist[0].motivo, motivo);
    } else {
        s_os_hist[0].motivo[0] = '\0';
    }
    s_dev_os_separadas++;
    s_dev_remed_separados += separados;
    historico_rebuild_list();
}

/* Modal de OS concluída — aparece sobre tudo em layer_top. */
static void os_concluida_ok_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    lv_obj_t *modal = lv_event_get_user_data(e);
    if (modal) {
        lv_obj_del(modal);
    }
}

static void os_show_concluida_modal(const char *num, uint32_t total)
{
    /* Fundo semi-transparente sobre tudo. */
    lv_obj_t *overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(overlay);
    lv_obj_set_size(overlay, lv_pct(100), lv_pct(100));
    lv_obj_set_style_bg_color(overlay, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_50, LV_PART_MAIN);
    lv_obj_clear_flag(overlay, LV_OBJ_FLAG_SCROLLABLE);

    /* Card central. */
    lv_obj_t *card = lv_obj_create(overlay);
    lv_obj_remove_style_all(card);
    lv_obj_set_size(card, 480, LV_SIZE_CONTENT);
    lv_obj_align(card, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(card, COLOR_CARD, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(card, 24, LV_PART_MAIN);
    lv_obj_set_style_border_color(card, COLOR_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_width(card, 1, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(card, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_shadow_opa(card, LV_OPA_20, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(card, 24, LV_PART_MAIN);
    lv_obj_set_style_shadow_ofs_y(card, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_all(card, 32, LV_PART_MAIN);
    lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(card, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(card, 16, LV_PART_MAIN);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    /* Círculo com ícone de check. */
    lv_obj_t *icon_circle = lv_obj_create(card);
    lv_obj_remove_style_all(icon_circle);
    lv_obj_set_size(icon_circle, 80, 80);
    lv_obj_set_style_bg_color(icon_circle, lv_color_hex(0xD1FAE5), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(icon_circle, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(icon_circle, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_t *icon_lbl = lv_label_create(icon_circle);
    lv_label_set_text(icon_lbl, LV_SYMBOL_OK);
    lv_obj_set_style_text_font(icon_lbl, &lv_font_montserrat_42, LV_PART_MAIN);
    lv_obj_set_style_text_color(icon_lbl, COLOR_OK, LV_PART_MAIN);
    lv_obj_center(icon_lbl);

    /* Título. */
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, "OS Concluida!");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_42, LV_PART_MAIN);
    lv_obj_set_style_text_color(title, COLOR_OK, LV_PART_MAIN);

    /* Número e total. */
    lv_obj_t *info = lv_label_create(card);
    lv_label_set_text_fmt(info, "OS  %s\n%lu remedios separados", num, (unsigned long)total);
    lv_obj_set_style_text_font(info, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(info, COLOR_LABEL, LV_PART_MAIN);
    lv_obj_set_style_text_align(info, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_long_mode(info, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(info, lv_pct(100));

    /* Separador fino. */
    lv_obj_t *sep = lv_obj_create(card);
    lv_obj_remove_style_all(sep);
    lv_obj_set_size(sep, lv_pct(100), 1);
    lv_obj_set_style_bg_color(sep, COLOR_BORDER, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(sep, LV_OPA_COVER, LV_PART_MAIN);

    /* Botão OK. */
    lv_obj_t *btn = lv_btn_create(card);
    lv_obj_set_width(btn, lv_pct(100));
    lv_obj_set_height(btn, 56);
    lv_obj_set_style_radius(btn, 14, LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn, COLOR_OK, LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x059669), LV_STATE_PRESSED);
    lv_obj_add_event_cb(btn, os_concluida_ok_cb, LV_EVENT_CLICKED, overlay);
    lv_obj_t *btn_lbl = lv_label_create(btn);
    lv_label_set_text(btn_lbl, "OK");
    lv_obj_set_style_text_font(btn_lbl, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(btn_lbl, lv_color_white(), LV_PART_MAIN);
    lv_obj_center(btn_lbl);

    lv_obj_move_foreground(overlay);
}

static void os_finalize_concluida(void)
{
    if (!s_os_active) {
        return;
    }
    char num_copy[OS_NUM_BUF_SZ];
    str_copy_trunc(num_copy, sizeof num_copy, s_os_num_str);
    uint32_t done_snap = s_os_meds_total;
    historico_push(s_os_num_str, s_os_client_str, s_os_meds_total, s_os_meds_total, 0,
                   OS_HIST_CONCLUIDA, NULL);
    os_reset_idle();
    serial_bridge_os_complete(num_copy, done_snap);
    if (s_logged_in) {
        os_show_concluida_modal(num_copy, done_snap);
    } else {
        /* Concluida em segundo plano: guarda para mostrar ao voltar a entrar. */
        s_os_concl_pending = true;
        str_copy_trunc(s_os_concl_num, sizeof s_os_concl_num, num_copy);
        s_os_concl_total = done_snap;
    }
    APP_INFO("OS %s concluida", num_copy);
}

static void os_finalize_cancelada(const char *motivo)
{
    if (!s_os_active) {
        return;
    }
    char num_copy[OS_NUM_BUF_SZ];
    str_copy_trunc(num_copy, sizeof num_copy, s_os_num_str);
    uint32_t done_snap = s_os_meds_done;
    historico_push(s_os_num_str, s_os_client_str, s_os_meds_done, s_os_meds_total, 0,
                   OS_HIST_CANCELADA, motivo);
    os_reset_idle();
    serial_bridge_os_cancel(num_copy, done_snap, motivo);
    APP_INFO("OS %s cancelada", num_copy);
}

/* Callback diferido: enviado ~20 ms após o pick para garantir que o
 * display já renderizou o novo contador antes de o backend receber o evento.
 * Isso evita a aparência de "+1" no software enquanto a tela ainda mostra
 * o valor anterior. */
static void serial_bridge_unit_deferred_cb(lv_timer_t *t)
{
    (void)t;
    if (!s_bridge_unit_pending || !s_os_active) { return; }
    s_bridge_unit_pending = false;
    serial_bridge_unit(s_os_num_str, s_bridge_unit_pending_done);
    if (s_bridge_unit_pending_done >= s_os_meds_total) {
        os_finalize_concluida();
    } else if (!s_os_paused) {
        os_schedule_next_pick();
    }
}

static void os_pick_timer_cb(lv_timer_t *t)
{
    (void)t;
    s_os_pick_timer = NULL;
    if (!s_os_active || s_os_meds_total == 0 || s_os_meds_done >= s_os_meds_total) {
        return;
    }
    uint32_t pick_ms = 2000 + (esp_random() % 8001U);
    s_os_meds_done++;
    s_os_pick_ms_sum += (uint64_t)pick_ms;
    s_os_pick_samples++;
    ui_os_paint_labels();  /* atualiza tela imediatamente */

    /* Agenda envio do BRIDGE:unit para depois do próximo frame renderizado */
    s_bridge_unit_pending_done = s_os_meds_done;
    s_bridge_unit_pending      = true;
    lv_timer_t *defer = lv_timer_create(serial_bridge_unit_deferred_cb, 20, NULL);
    lv_timer_set_repeat_count(defer, 1);
}

static void os_schedule_next_pick(void)
{
    os_pick_timer_stop();
    if (!s_os_active || s_os_meds_total == 0 || s_os_meds_done >= s_os_meds_total) {
        return;
    }
    uint32_t delay_ms = 2000 + (esp_random() % 8001U);
    s_os_pick_timer = lv_timer_create(os_pick_timer_cb, delay_ms, NULL);
    lv_timer_set_repeat_count(s_os_pick_timer, 1);
}

static void os_start_manual(const char *numero, const char *cliente, uint32_t total)
{
    str_copy_trunc(s_os_num_str, sizeof s_os_num_str, numero);
    str_copy_trunc(s_os_client_str, sizeof s_os_client_str, cliente);
    s_os_meds_total    = total;
    s_os_meds_done     = 0;
    s_os_pick_ms_sum   = 0;
    s_os_pick_samples  = 0;
    s_os_sess_start_us = esp_timer_get_time();
    s_os_active        = true;
    os_close_modal();
    s_os_paused = false;
    led_amarelo_set(true);
    os_update_panels();
    ui_os_paint_labels();
    os_schedule_next_pick();
    serial_bridge_os_start(s_os_num_str, s_os_client_str, total);
    APP_INFO("OS manual iniciada: %s (%lu remedios)", s_os_num_str, (unsigned long)total);
}

static void os_pause(void)
{
    if (!s_os_active || s_os_paused) { return; }
    s_os_paused = true;
    os_pick_timer_stop();
    ui_os_paint_labels();
    led_vermelho_blink_start();
    serial_bridge_os_pause(s_os_num_str);
    APP_INFO("OS %s pausada", s_os_num_str);
}

static void os_resume(void)
{
    if (!s_os_active || !s_os_paused) { return; }
    s_os_paused = false;
    led_vermelho_blink_stop();
    /* Se há um pick em voo aguardando o deferred, ele agendará o próximo pick
     * sozinho ao ser processado; caso contrário, agendamos aqui. */
    if (!s_bridge_unit_pending) {
        os_schedule_next_pick();
    }
    ui_os_paint_labels();
    serial_bridge_os_resume(s_os_num_str);
    APP_INFO("OS %s retomada", s_os_num_str);
}

static lv_obj_t *os_make_fullscreen_shell(const char *titulo, lv_event_cb_t voltar_cb)
{
    historico_close_modal();
    os_close_modal();

    lv_obj_t *backdrop = lv_obj_create(lv_layer_top());
    s_os.modal = backdrop;
    lv_obj_remove_style_all(backdrop);
    lv_obj_set_size(backdrop, lv_pct(100), lv_pct(100));
    lv_obj_set_style_bg_color(backdrop, COLOR_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(backdrop, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_pad_all(backdrop, 16, LV_PART_MAIN);
    lv_obj_set_flex_flow(backdrop, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(backdrop, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(backdrop, 12, LV_PART_MAIN);

    lv_obj_t *hdr = lv_obj_create(backdrop);
    lv_obj_remove_style_all(hdr);
    lv_obj_set_width(hdr, lv_pct(100));
    lv_obj_set_height(hdr, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(hdr, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(hdr, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(hdr, 12, LV_PART_MAIN);

    lv_obj_t *b_back = lv_btn_create(hdr);
    lv_obj_set_height(b_back, 48);
    lv_obj_set_style_radius(b_back, 12, LV_PART_MAIN);
    lv_obj_set_style_bg_color(b_back, COLOR_BADGE_BG, LV_PART_MAIN);
    lv_obj_add_event_cb(b_back, voltar_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l_back = lv_label_create(b_back);
    lv_label_set_text(l_back, "Voltar");
    lv_obj_set_style_text_font(l_back, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(l_back, COLOR_VALUE, LV_PART_MAIN);
    lv_obj_center(l_back);

    lv_obj_t *tit = lv_label_create(hdr);
    lv_label_set_text(tit, titulo);
    lv_obj_set_style_text_font(tit, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(tit, COLOR_LABEL, LV_PART_MAIN);
    lv_obj_set_flex_grow(tit, 1);
    lv_label_set_long_mode(tit, LV_LABEL_LONG_DOT);

    return backdrop;
}

static void os_manual_voltar_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    os_close_modal();
}

static void os_manual_iniciar_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    const char *num = s_os.ta_num ? lv_textarea_get_text(s_os.ta_num) : "";
    const char *cli = s_os.ta_client ? lv_textarea_get_text(s_os.ta_client) : "";
    if (!num || !num[0] || !cli || !cli[0]) {
        APP_WARN("OS manual: numero e cliente obrigatorios");
        return;
    }
    bool ok = false;
    uint32_t total = 0;
    if (s_os.ta_total) {
        const char *ts = lv_textarea_get_text(s_os.ta_total);
        if (ts && ts[0]) {
            char *end = NULL;
            unsigned long v = strtoul(ts, &end, 10);
            while (end && *end == ' ') {
                end++;
            }
            if (end && *end == '\0' && v > 0 && v <= 9999) {
                total = (uint32_t)v;
                ok    = true;
            }
        }
    }
    if (!ok) {
        APP_WARN("OS manual: total de remedios invalido (1-9999)");
        return;
    }
    os_start_manual(num, cli, total);
}

static void os_btn_nova_manual_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    os_open_manual_modal();
}

static void os_open_manual_modal(void)
{
    lv_obj_t *backdrop = os_make_fullscreen_shell("Nova OS manual", os_manual_voltar_cb);

    lv_obj_t *card = make_card(backdrop);
    lv_obj_set_width(card, lv_pct(100));
    lv_obj_set_flex_grow(card, 1);
    lv_obj_set_style_pad_row(card, 16, LV_PART_MAIN);
    lv_obj_add_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(card, LV_SCROLLBAR_MODE_AUTO);

    lv_obj_t *ln = lv_label_create(card);
    lv_label_set_text(ln, "Numero da OS");
    lv_obj_set_style_text_font(ln, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(ln, COLOR_LABEL, LV_PART_MAIN);

    s_os.ta_num = lv_textarea_create(card);
    lv_textarea_set_one_line(s_os.ta_num, true);
    lv_textarea_set_max_length(s_os.ta_num, OS_NUM_BUF_SZ - 1);
    style_os_form_field(s_os.ta_num, false);

    lv_obj_t *lc = lv_label_create(card);
    lv_label_set_text(lc, "Nome do cliente");
    lv_obj_set_style_text_font(lc, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(lc, COLOR_LABEL, LV_PART_MAIN);

    s_os.ta_client = lv_textarea_create(card);
    lv_textarea_set_one_line(s_os.ta_client, true);
    lv_textarea_set_max_length(s_os.ta_client, OS_CLIENT_BUF_SZ - 1);
    style_os_form_field(s_os.ta_client, false);

    lv_obj_t *lt = lv_label_create(card);
    lv_label_set_text(lt, "Total de remedios");
    lv_obj_set_style_text_font(lt, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(lt, COLOR_LABEL, LV_PART_MAIN);

    s_os.ta_total = lv_textarea_create(card);
    lv_textarea_set_one_line(s_os.ta_total, true);
    lv_textarea_set_max_length(s_os.ta_total, 4);
    lv_textarea_set_accepted_chars(s_os.ta_total, "0123456789");
    style_os_form_field(s_os.ta_total, true);

    lv_obj_t *b_go = lv_btn_create(backdrop);
    lv_obj_set_width(b_go, lv_pct(100));
    lv_obj_set_height(b_go, 52);
    lv_obj_set_style_radius(b_go, 14, LV_PART_MAIN);
    lv_obj_set_style_bg_color(b_go, COLOR_ACCENT, LV_PART_MAIN);
    lv_obj_add_event_cb(b_go, os_manual_iniciar_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lg = lv_label_create(b_go);
    lv_label_set_text(lg, "Iniciar separacao");
    lv_obj_set_style_text_color(lg, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(lg, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_center(lg);

    lv_obj_update_layout(backdrop);
    lv_obj_move_foreground(backdrop);
}

static void os_cancel_voltar_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    os_close_modal();
    if (s_os_active) {
        /* Se a OS não estava pausada, para o blink vermelho. */
        if (!s_os_paused) {
            led_vermelho_blink_stop();
        }
        os_schedule_next_pick();
    }
}

static void os_cancel_confirm_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    const char *mot = s_os.ta_motivo ? lv_textarea_get_text(s_os.ta_motivo) : "";
    if (!mot || !mot[0]) {
        APP_WARN("Informe o motivo do cancelamento");
        return;
    }
    os_pick_timer_stop();
    os_finalize_cancelada(mot);
    os_close_modal();
}

static void os_btn_cancelar_os_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    if (!s_os_active) {
        return;
    }
    os_pick_timer_stop();
    led_vermelho_blink_start();
    os_open_cancel_modal();
}

static void os_open_cancel_modal(void)
{
    lv_obj_t *backdrop = os_make_fullscreen_shell("Cancelar OS", os_cancel_voltar_cb);

    lv_obj_t *card = make_card(backdrop);
    lv_obj_set_width(card, lv_pct(100));
    lv_obj_set_flex_grow(card, 1);
    lv_obj_add_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(card, LV_SCROLLBAR_MODE_AUTO);

    lv_obj_t *hint = lv_label_create(card);
    lv_label_set_text(hint, "Descreva o motivo do cancelamento:");
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(hint, COLOR_LABEL, LV_PART_MAIN);
    lv_obj_set_width(hint, lv_pct(100));

    s_os.ta_motivo = lv_textarea_create(card);
    lv_textarea_set_max_length(s_os.ta_motivo, OS_HIST_MOTIVO_SZ - 1);
    lv_textarea_set_placeholder_text(s_os.ta_motivo, "Ex.: cliente desistiu, falta de stock...");
    style_os_form_field(s_os.ta_motivo, false);

    lv_obj_t *b_ok = lv_btn_create(backdrop);
    lv_obj_set_width(b_ok, lv_pct(100));
    lv_obj_set_height(b_ok, 52);
    lv_obj_set_style_radius(b_ok, 14, LV_PART_MAIN);
    lv_obj_set_style_bg_color(b_ok, lv_color_hex(0xDC2626), LV_PART_MAIN);
    lv_obj_add_event_cb(b_ok, os_cancel_confirm_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lo = lv_label_create(b_ok);
    lv_label_set_text(lo, "Confirmar cancelamento");
    lv_obj_set_style_text_color(lo, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(lo, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_center(lo);

    lv_obj_update_layout(backdrop);
    lv_obj_move_foreground(backdrop);
}

static void historico_close_modal(void)
{
    if (s_hist_modal) {
        lv_obj_del(s_hist_modal);
        s_hist_modal = NULL;
    }
}

static void historico_rebuild_list(void)
{
    if (!s_hist_scroll) {
        return;
    }
    lv_obj_clean(s_hist_scroll);
    for (uint32_t i = 0; i < s_os_hist_count; i++) {
        const os_hist_item_t *it = &s_os_hist[i];
        lv_obj_t *card = make_card(s_hist_scroll);
        lv_obj_set_width(card, lv_pct(100));
        lv_obj_add_event_cb(card, historico_card_clicked, LV_EVENT_CLICKED, (void *)(uintptr_t)i);

        lv_obj_t *t1 = lv_label_create(card);
        lv_label_set_text(t1, it->numero);
        lv_obj_set_style_text_font(t1, &lv_font_montserrat_24, LV_PART_MAIN);
        lv_obj_set_style_text_color(t1, COLOR_VALUE, LV_PART_MAIN);

        lv_obj_t *t2 = lv_label_create(card);
        const char *est = (it->estado == OS_HIST_CANCELADA) ? "Cancelada" : "Concluida";
        lv_label_set_text_fmt(t2, "%s  |  %lu / %lu  —  toque para detalhes",
                              est, (unsigned long)it->separados, (unsigned long)it->total);
        lv_obj_set_style_text_font(t2, &lv_font_montserrat_14, LV_PART_MAIN);
        lv_obj_set_style_text_color(t2, COLOR_LABEL, LV_PART_MAIN);
        lv_label_set_long_mode(t2, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(t2, lv_pct(100));

        historico_item_bubble_events(card);
    }

    if (s_os_hist_count == 0) {
        lv_obj_t *empty = lv_label_create(s_hist_scroll);
        lv_label_set_text(empty, "Nenhuma OS no historico.\nConcluidas e canceladas aparecem aqui.");
        lv_obj_set_style_text_font(empty, &lv_font_montserrat_24, LV_PART_MAIN);
        lv_obj_set_style_text_color(empty, COLOR_LABEL, LV_PART_MAIN);
        lv_label_set_long_mode(empty, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(empty, lv_pct(100));
    }
}

static void historico_seed_demo_if_empty(void)
{
    /* Histórico inicia vazio — sem dados de demonstração. */
}

static void historico_btn_fechar_clicked(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    historico_close_modal();
}

static void historico_btn_excluir_clicked(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    size_t idx = s_hist_modal_idx;
    if (idx >= s_os_hist_count) {
        historico_close_modal();
        return;
    }
    memmove(&s_os_hist[idx], &s_os_hist[idx + 1],
            (size_t)(s_os_hist_count - idx - 1) * sizeof(os_hist_item_t));
    s_os_hist_count--;
    historico_close_modal();
    historico_rebuild_list();
    APP_INFO("Historico: OS indice %u removida", (unsigned)idx);
}

static void historico_open_detail(size_t idx)
{
    if (idx >= s_os_hist_count) {
        return;
    }
    historico_close_modal();
    s_hist_modal_idx = idx;
    const os_hist_item_t *it = &s_os_hist[idx];

    /* Ecra completo sobre layer_top (sem popup centrado). */
    lv_obj_t *backdrop = lv_obj_create(lv_layer_top());
    s_hist_modal = backdrop;
    lv_obj_remove_style_all(backdrop);
    lv_obj_set_size(backdrop, lv_pct(100), lv_pct(100));
    lv_obj_set_style_bg_color(backdrop, COLOR_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(backdrop, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_pad_all(backdrop, 16, LV_PART_MAIN);
    lv_obj_set_flex_flow(backdrop, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(backdrop, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(backdrop, 16, LV_PART_MAIN);

    lv_obj_t *hdr = lv_obj_create(backdrop);
    lv_obj_remove_style_all(hdr);
    lv_obj_set_width(hdr, lv_pct(100));
    lv_obj_set_height(hdr, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(hdr, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(hdr, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(hdr, 12, LV_PART_MAIN);

    lv_obj_t *b_back = lv_btn_create(hdr);
    lv_obj_set_height(b_back, 48);
    lv_obj_set_style_radius(b_back, 12, LV_PART_MAIN);
    lv_obj_set_style_bg_color(b_back, COLOR_BADGE_BG, LV_PART_MAIN);
    lv_obj_add_event_cb(b_back, historico_btn_fechar_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l_back = lv_label_create(b_back);
    lv_label_set_text(l_back, "Voltar");
    lv_obj_set_style_text_font(l_back, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(l_back, COLOR_VALUE, LV_PART_MAIN);
    lv_obj_center(l_back);

    lv_obj_t *tit = lv_label_create(hdr);
    lv_label_set_text_fmt(tit, "OS %s", it->numero);
    lv_obj_set_style_text_font(tit, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(tit, COLOR_LABEL, LV_PART_MAIN);
    lv_obj_set_flex_grow(tit, 1);
    lv_label_set_long_mode(tit, LV_LABEL_LONG_DOT);

    lv_obj_t *scroll = lv_obj_create(backdrop);
    lv_obj_remove_style_all(scroll);
    lv_obj_set_width(scroll, lv_pct(100));
    lv_obj_set_flex_grow(scroll, 1);
    lv_obj_add_flag(scroll, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(scroll, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(scroll, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_flex_flow(scroll, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(scroll, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_all(scroll, 4, LV_PART_MAIN);

    char dbuf[72];
    format_duracao_ms(dbuf, sizeof dbuf, it->duracao_ms);

    const char *est = (it->estado == OS_HIST_CANCELADA) ? "Cancelada" : "Concluida";
    char buf[384];
    if (it->estado == OS_HIST_CANCELADA && it->motivo[0]) {
        snprintf(buf, sizeof buf,
                 "Estado:  %s\n\nCliente:\n%s\n\nSeparados:  %lu\nTotal:  %lu\n\nTempo total:  %s\n\nMotivo:\n%s",
                 est, it->cliente, (unsigned long)it->separados, (unsigned long)it->total, dbuf, it->motivo);
    } else {
        snprintf(buf, sizeof buf,
                 "Estado:  %s\n\nCliente:\n%s\n\nSeparados:  %lu\nTotal:  %lu\n\nTempo total:  %s",
                 est, it->cliente, (unsigned long)it->separados, (unsigned long)it->total, dbuf);
    }
    lv_obj_t *body = lv_label_create(scroll);
    lv_label_set_text(body, buf);
    lv_obj_set_style_text_font(body, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(body, COLOR_VALUE, LV_PART_MAIN);
    lv_label_set_long_mode(body, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(body, lv_pct(100));

    lv_obj_t *b_del = lv_btn_create(backdrop);
    lv_obj_set_width(b_del, lv_pct(100));
    lv_obj_set_height(b_del, 48);
    lv_obj_set_style_radius(b_del, 12, LV_PART_MAIN);
    lv_obj_set_style_bg_color(b_del, lv_color_hex(0xFEE2E2), LV_PART_MAIN);
    lv_obj_set_style_bg_color(b_del, lv_color_hex(0xFECACA), LV_STATE_PRESSED);
    lv_obj_add_event_cb(b_del, historico_btn_excluir_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l2 = lv_label_create(b_del);
    lv_label_set_text(l2, "Excluir");
    lv_obj_set_style_text_font(l2, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(l2, lv_color_hex(0xB91C1C), LV_PART_MAIN);
    lv_obj_center(l2);

    lv_obj_move_foreground(backdrop);
}

static void historico_card_clicked(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    size_t idx = (size_t)(uintptr_t)lv_event_get_user_data(e);
    historico_open_detail(idx);
}

/* --------------------------------------------------------------------------
 * Botão Pausar / Retomar OS
 * -------------------------------------------------------------------------- */
static void os_btn_pausar_os_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) { return; }
    if (!s_os_active) { return; }
    if (s_os_paused) {
        os_resume();
    } else {
        os_pause();
    }
}

/* --------------------------------------------------------------------------
 * Tela de Configurações
 * -------------------------------------------------------------------------- */
static void settings_menu_item_set_active(lv_obj_t *item, lv_obj_t *lbl, bool active)
{
    if (!item) {
        return;
    }
    if (active) {
        lv_obj_set_style_bg_color(item, lv_color_hex(0xF0EEFF), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(item, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_border_side(item, LV_BORDER_SIDE_LEFT, LV_PART_MAIN);
        lv_obj_set_style_border_color(item, COLOR_ACCENT, LV_PART_MAIN);
        lv_obj_set_style_border_width(item, 3, LV_PART_MAIN);
        if (lbl) {
            lv_obj_set_style_text_color(lbl, COLOR_ACCENT, LV_PART_MAIN);
        }
    } else {
        lv_obj_set_style_bg_opa(item, LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_border_width(item, 0, LV_PART_MAIN);
        lv_obj_set_style_bg_color(item, lv_color_hex(0xF0EEFF), LV_STATE_PRESSED);
        lv_obj_set_style_bg_opa(item, LV_OPA_COVER, LV_STATE_PRESSED);
        if (lbl) {
            lv_obj_set_style_text_color(lbl, COLOR_VALUE, LV_PART_MAIN);
        }
    }
}

static lv_obj_t *make_settings_menu_item(lv_obj_t *parent, const char *text, lv_obj_t **out_lbl)
{
    lv_obj_t *item = lv_obj_create(parent);
    lv_obj_remove_style_all(item);
    lv_obj_set_width(item, lv_pct(100));
    lv_obj_set_height(item, 52);
    lv_obj_set_style_radius(item, 10, LV_PART_MAIN);
    lv_obj_set_style_pad_left(item, 14, LV_PART_MAIN);
    lv_obj_set_flex_flow(item, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(item, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(item, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(item, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_t *lbl = lv_label_create(item);
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl, COLOR_VALUE, LV_PART_MAIN);
    if (out_lbl) {
        *out_lbl = lbl;
    }
    return item;
}

static lv_obj_t *make_settings_content_panel(lv_obj_t *parent)
{
    lv_obj_t *panel = lv_obj_create(parent);
    lv_obj_remove_style_all(panel);
    lv_obj_set_width(panel, lv_pct(100));
    lv_obj_set_flex_grow(panel, 1);
    lv_obj_set_height(panel, lv_pct(100));
    lv_obj_add_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(panel, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(panel, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(panel, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_all(panel, 12, LV_PART_MAIN);
    lv_obj_set_style_pad_row(panel, 12, LV_PART_MAIN);
    return panel;
}

static void settings_hide_kb(void)
{
    if (s_dev.kb) {
        lv_keyboard_set_textarea(s_dev.kb, NULL);
        lv_obj_add_flag(s_dev.kb, LV_OBJ_FLAG_HIDDEN);
    }
}

static void settings_nav_dev_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    settings_menu_item_set_active(s_settings_nav.mi_dev, s_settings_nav.lbl_dev, true);
    settings_menu_item_set_active(s_settings_nav.mi_telas, s_settings_nav.lbl_telas, false);
    if (s_settings_nav.panel_dev) {
        lv_obj_clear_flag(s_settings_nav.panel_dev, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_settings_nav.panel_telas) {
        lv_obj_add_flag(s_settings_nav.panel_telas, LV_OBJ_FLAG_HIDDEN);
    }
}

static void settings_nav_telas_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    settings_hide_kb();
    settings_menu_item_set_active(s_settings_nav.mi_dev, s_settings_nav.lbl_dev, false);
    settings_menu_item_set_active(s_settings_nav.mi_telas, s_settings_nav.lbl_telas, true);
    if (s_settings_nav.panel_dev) {
        lv_obj_add_flag(s_settings_nav.panel_dev, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_settings_nav.panel_telas) {
        lv_obj_clear_flag(s_settings_nav.panel_telas, LV_OBJ_FLAG_HIDDEN);
    }
}

static void settings_close_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) { return; }
    lv_obj_t *modal = lv_event_get_user_data(e);
    if (!modal) { return; }
    /* Garante save NVS mesmo que defocus não tenha disparado. */
    if (s_dev.ta_nome) {
        const char *v = lv_textarea_get_text(s_dev.ta_nome);
        str_copy_trunc(s_dev_robot_name, sizeof s_dev_robot_name, v);
        nvs_save_str(NVS_KEY_NAME, v);
    }
    if (s_dev.ta_id) {
        const char *v = lv_textarea_get_text(s_dev.ta_id);
        str_copy_trunc(s_dev_robot_code, sizeof s_dev_robot_code, v);
        nvs_save_str(NVS_KEY_CODE, v);
    }
    if (s_dev.kb) {
        lv_keyboard_set_textarea(s_dev.kb, NULL);
        lv_obj_add_flag(s_dev.kb, LV_OBJ_FLAG_HIDDEN);
    }
    s_dev.ta_nome       = NULL;
    s_dev.ta_id         = NULL;
    s_dev.lbl_os_cnt    = NULL;
    s_dev.lbl_remed_cnt = NULL;
    s_settings_dash_scr = NULL;
    memset(&s_settings_nav, 0, sizeof(s_settings_nav));
    lv_obj_del(modal);
}

static void btn_open_settings_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) { return; }
    lv_obj_t *dash_scr = lv_event_get_user_data(e);
    build_settings_screen(dash_scr);
}

static void build_settings_screen(lv_obj_t *dash_scr)
{
    lv_obj_t *scr = lv_scr_act();
    s_settings_dash_scr = dash_scr;

    /* Fundo full-screen */
    lv_obj_t *backdrop = lv_obj_create(scr);
    lv_obj_remove_style_all(backdrop);
    lv_obj_set_size(backdrop, lv_pct(100), lv_pct(100));
    lv_obj_align(backdrop, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_bg_color(backdrop, COLOR_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(backdrop, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_flex_flow(backdrop, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(backdrop, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_all(backdrop, 0, LV_PART_MAIN);
    lv_obj_clear_flag(backdrop, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_move_foreground(backdrop);

    /* ── Cabeçalho ── */
    lv_obj_t *hdr = lv_obj_create(backdrop);
    lv_obj_remove_style_all(hdr);
    lv_obj_set_width(hdr, lv_pct(100));
    lv_obj_set_height(hdr, 64);
    lv_obj_set_style_bg_color(hdr, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(hdr, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(hdr, 0, LV_PART_MAIN);
    lv_obj_set_style_border_side(hdr, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);
    lv_obj_set_style_border_color(hdr, lv_color_hex(0xD4D4E8), LV_PART_MAIN);
    lv_obj_set_style_border_width(hdr, 1, LV_PART_MAIN);
    lv_obj_set_style_pad_hor(hdr, 16, LV_PART_MAIN);
    lv_obj_set_flex_flow(hdr, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(hdr, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *btn_back = lv_btn_create(hdr);
    lv_obj_set_size(btn_back, 130, 44);
    lv_obj_set_style_radius(btn_back, 10, LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn_back, COLOR_ACCENT, LV_PART_MAIN);
    lv_obj_add_event_cb(btn_back, settings_close_cb, LV_EVENT_CLICKED, backdrop);
    lv_obj_t *lbl_back = lv_label_create(btn_back);
    lv_label_set_text(lbl_back, "Voltar");
    lv_obj_set_style_text_color(lbl_back, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_back, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_center(lbl_back);

    lv_obj_t *title = lv_label_create(hdr);
    lv_label_set_text(title, "Configuracoes");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_42, LV_PART_MAIN);
    lv_obj_set_style_text_color(title, COLOR_VALUE, LV_PART_MAIN);
    lv_obj_set_flex_grow(title, 1);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);

    /* Espaçador para equilibrar o título */
    lv_obj_t *spacer = lv_obj_create(hdr);
    lv_obj_remove_style_all(spacer);
    lv_obj_set_size(spacer, 130, 44);

    /* ── Corpo: sidebar + conteúdo ── */
    lv_obj_t *body = lv_obj_create(backdrop);
    lv_obj_remove_style_all(body);
    lv_obj_set_width(body, lv_pct(100));
    lv_obj_set_flex_grow(body, 1);
    lv_obj_set_flex_flow(body, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(body, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(body, LV_OBJ_FLAG_SCROLLABLE);

    /* Sidebar */
    lv_obj_t *sidebar = lv_obj_create(body);
    lv_obj_remove_style_all(sidebar);
    lv_obj_set_size(sidebar, 210, lv_pct(100));
    lv_obj_set_style_bg_color(sidebar, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(sidebar, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_side(sidebar, LV_BORDER_SIDE_RIGHT, LV_PART_MAIN);
    lv_obj_set_style_border_color(sidebar, lv_color_hex(0xD4D4E8), LV_PART_MAIN);
    lv_obj_set_style_border_width(sidebar, 1, LV_PART_MAIN);
    lv_obj_set_flex_flow(sidebar, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(sidebar, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_all(sidebar, 12, LV_PART_MAIN);
    lv_obj_set_style_pad_row(sidebar, 6, LV_PART_MAIN);
    lv_obj_clear_flag(sidebar, LV_OBJ_FLAG_SCROLLABLE);

    /* Itens do menu (parte superior) */
    lv_obj_t *menu_top = lv_obj_create(sidebar);
    lv_obj_remove_style_all(menu_top);
    lv_obj_set_width(menu_top, lv_pct(100));
    lv_obj_set_height(menu_top, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(menu_top, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(menu_top, 4, LV_PART_MAIN);
    lv_obj_clear_flag(menu_top, LV_OBJ_FLAG_SCROLLABLE);

    memset(&s_settings_nav, 0, sizeof(s_settings_nav));

    /* Item: Dispositivo */
    s_settings_nav.mi_dev = make_settings_menu_item(menu_top, "Dispositivo", &s_settings_nav.lbl_dev);
    lv_obj_add_event_cb(s_settings_nav.mi_dev, settings_nav_dev_cb, LV_EVENT_CLICKED, NULL);
    settings_menu_item_set_active(s_settings_nav.mi_dev, s_settings_nav.lbl_dev, true);

    /* Item: Telas */
    s_settings_nav.mi_telas = make_settings_menu_item(menu_top, "Telas", &s_settings_nav.lbl_telas);
    lv_obj_add_event_cb(s_settings_nav.mi_telas, settings_nav_telas_cb, LV_EVENT_CLICKED, NULL);
    settings_menu_item_set_active(s_settings_nav.mi_telas, s_settings_nav.lbl_telas, false);

    /* Item: Sair (parte inferior do sidebar) */
    lv_obj_t *mi_sair = lv_obj_create(sidebar);
    lv_obj_remove_style_all(mi_sair);
    lv_obj_set_width(mi_sair, lv_pct(100));
    lv_obj_set_height(mi_sair, 52);
    lv_obj_set_style_bg_color(mi_sair, lv_color_hex(0xFEE2E2), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(mi_sair, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(mi_sair, 10, LV_PART_MAIN);
    lv_obj_set_style_bg_color(mi_sair, lv_color_hex(0xFECACA), LV_STATE_PRESSED);
    lv_obj_set_style_pad_left(mi_sair, 14, LV_PART_MAIN);
    lv_obj_set_flex_flow(mi_sair, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(mi_sair, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(mi_sair, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(mi_sair, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(mi_sair, on_logout_clicked, LV_EVENT_CLICKED, dash_scr);
    lv_obj_t *lmi_sair = lv_label_create(mi_sair);
    lv_label_set_text(lmi_sair, "Sair");
    lv_obj_set_style_text_font(lmi_sair, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(lmi_sair, lv_color_hex(0xB91C1C), LV_PART_MAIN);

    /* Área de conteúdo à direita (dois painéis: Dispositivo e Telas) */
    lv_obj_t *content_host = lv_obj_create(body);
    lv_obj_remove_style_all(content_host);
    lv_obj_set_flex_grow(content_host, 1);
    lv_obj_set_height(content_host, lv_pct(100));
    lv_obj_set_flex_flow(content_host, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(content_host, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(content_host, LV_OBJ_FLAG_SCROLLABLE);

    s_settings_nav.panel_dev = make_settings_content_panel(content_host);
    lv_obj_t *content = s_settings_nav.panel_dev;

    s_settings_nav.panel_telas = make_settings_content_panel(content_host);
    lv_obj_add_flag(s_settings_nav.panel_telas, LV_OBJ_FLAG_HIDDEN);

    /* Card: Identificação do Robô */
    lv_obj_t *card_id = make_card(content);
    lv_obj_set_width(card_id, lv_pct(100));
    lv_obj_set_height(card_id, LV_SIZE_CONTENT);
    lv_obj_set_style_pad_row(card_id, 8, LV_PART_MAIN);
    make_card_label(card_id, "IDENTIFICACAO DO ROBO");

    lv_obj_t *ln = lv_label_create(card_id);
    lv_label_set_text(ln, "Nome do robo");
    lv_obj_set_style_text_font(ln, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(ln, COLOR_LABEL, LV_PART_MAIN);

    s_dev.ta_nome = lv_textarea_create(card_id);
    lv_textarea_set_one_line(s_dev.ta_nome, true);
    lv_textarea_set_max_length(s_dev.ta_nome, DEV_ROBOT_NAME_SZ - 1);
    lv_textarea_set_placeholder_text(s_dev.ta_nome, "ex.: Separador 01");
    style_login_ta(s_dev.ta_nome);
    lv_obj_set_width(s_dev.ta_nome, lv_pct(100));
    lv_obj_set_style_min_height(s_dev.ta_nome, 52, LV_PART_MAIN);
    lv_obj_add_event_cb(s_dev.ta_nome, dev_ta_activate_cb, LV_EVENT_FOCUSED, &s_dev);
    lv_obj_add_event_cb(s_dev.ta_nome, dev_ta_activate_cb, LV_EVENT_CLICKED, &s_dev);
    lv_obj_add_event_cb(s_dev.ta_nome, dev_ta_defocus_cb, LV_EVENT_DEFOCUSED, &s_dev);

    lv_obj_t *lid = lv_label_create(card_id);
    lv_label_set_text(lid, "Identificacao (ID)");
    lv_obj_set_style_text_font(lid, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(lid, COLOR_LABEL, LV_PART_MAIN);

    s_dev.ta_id = lv_textarea_create(card_id);
    lv_textarea_set_one_line(s_dev.ta_id, true);
    lv_textarea_set_max_length(s_dev.ta_id, DEV_ROBOT_ID_SZ - 1);
    lv_textarea_set_placeholder_text(s_dev.ta_id, "ex.: SN / inventario");
    style_login_ta(s_dev.ta_id);
    lv_obj_set_width(s_dev.ta_id, lv_pct(100));
    lv_obj_set_style_min_height(s_dev.ta_id, 52, LV_PART_MAIN);
    lv_obj_add_event_cb(s_dev.ta_id, dev_ta_activate_cb, LV_EVENT_FOCUSED, &s_dev);
    lv_obj_add_event_cb(s_dev.ta_id, dev_ta_activate_cb, LV_EVENT_CLICKED, &s_dev);
    lv_obj_add_event_cb(s_dev.ta_id, dev_ta_defocus_cb, LV_EVENT_DEFOCUSED, &s_dev);

    /* Popula com os valores em RAM (já sincronizados com NVS). */
    lv_textarea_set_text(s_dev.ta_nome, s_dev_robot_name[0] ? s_dev_robot_name : "Separador P4");
    lv_textarea_set_text(s_dev.ta_id,   s_dev_robot_code[0] ? s_dev_robot_code : "ESP32-P4-001");

    /* Card: Totais */
    lv_obj_t *card_tot = make_card(content);
    lv_obj_set_width(card_tot, lv_pct(100));
    lv_obj_set_height(card_tot, LV_SIZE_CONTENT);
    lv_obj_set_style_pad_row(card_tot, 8, LV_PART_MAIN);
    make_card_label(card_tot, "TOTAIS (DESDE O ARRANQUE)");

    lv_obj_t *lo = lv_label_create(card_tot);
    lv_label_set_text(lo, "OS separadas");
    lv_obj_set_style_text_font(lo, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(lo, COLOR_LABEL, LV_PART_MAIN);

    s_dev.lbl_os_cnt = lv_label_create(card_tot);
    lv_label_set_text_fmt(s_dev.lbl_os_cnt, "%lu", (unsigned long)s_dev_os_separadas);
    lv_obj_set_style_text_font(s_dev.lbl_os_cnt, &lv_font_montserrat_42, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_dev.lbl_os_cnt, COLOR_VALUE, LV_PART_MAIN);
    lv_obj_set_width(s_dev.lbl_os_cnt, lv_pct(100));

    lv_obj_t *lr = lv_label_create(card_tot);
    lv_label_set_text(lr, "Remedios separados");
    lv_obj_set_style_text_font(lr, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(lr, COLOR_LABEL, LV_PART_MAIN);

    s_dev.lbl_remed_cnt = lv_label_create(card_tot);
    lv_label_set_text_fmt(s_dev.lbl_remed_cnt, "%lu", (unsigned long)s_dev_remed_separados);
    lv_obj_set_style_text_font(s_dev.lbl_remed_cnt, &lv_font_montserrat_42, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_dev.lbl_remed_cnt, COLOR_VALUE, LV_PART_MAIN);
    lv_obj_set_width(s_dev.lbl_remed_cnt, lv_pct(100));

    lv_obj_update_layout(backdrop);
    lv_obj_move_foreground(backdrop);
    if (s_dev.kb) {
        lv_obj_move_foreground(s_dev.kb);
    }
}

static void build_dashboard_into(lv_obj_t *scr)
{
    if (!scr) {
        return;
    }

    s_lbl_os_number     = NULL;
    s_lbl_os_client     = NULL;
    s_lbl_os_meds       = NULL;
    s_bar_os_meds       = NULL;
    s_lbl_os_eta        = NULL;
    s_lbl_os_eta_detail = NULL;
    s_lbl_bomba_state   = NULL;
    s_lbl_valvula_state = NULL;

    historico_close_modal();
    os_close_modal();
    stepper_close_modal();
    dev_kb_destroy();
    s_hist_scroll = NULL;

    /* NAO paramos o timer de separacao nem zeramos s_os_active aqui:
     * ao reconstruir o dashboard (ex.: apos re-login) a OS que estava
     * a correr em segundo plano deve ser preservada e reexibida. */
    memset(&s_os, 0, sizeof(s_os));
    memset(&s_dev, 0, sizeof(s_dev));

    lv_obj_set_style_bg_color(scr, COLOR_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_style_pad_all(scr, 12, LV_PART_MAIN);
    lv_obj_set_style_pad_row(scr, 0, LV_PART_MAIN);
    lv_obj_set_flex_flow(scr, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(scr, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    lv_obj_t *tv = lv_tabview_create(scr, LV_DIR_TOP, 52);
    lv_obj_set_width(tv, lv_pct(100));
    lv_obj_set_flex_grow(tv, 1);
    style_tabview(tv);
    lv_obj_add_event_cb(tv, dev_tabview_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* Config: flutuante, canto inferior direito. */
    lv_obj_t *btn_cfg = lv_btn_create(scr);
    lv_obj_add_flag(btn_cfg, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_size(btn_cfg, 130, 44);
    lv_obj_set_style_radius(btn_cfg, 12, LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn_cfg, lv_color_hex(0xF0EEFF), LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn_cfg, lv_color_hex(0xE0D8FF), LV_STATE_PRESSED);
    lv_obj_set_style_border_width(btn_cfg, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(btn_cfg, lv_color_hex(0xB0A8E8), LV_PART_MAIN);
    lv_obj_add_event_cb(btn_cfg, btn_open_settings_cb, LV_EVENT_CLICKED, scr);
    lv_obj_align(btn_cfg, LV_ALIGN_BOTTOM_RIGHT, -14, -14);
    lv_obj_t *lbl_cfg = lv_label_create(btn_cfg);
    lv_label_set_text(lbl_cfg, "Config");
    lv_obj_set_style_text_color(lbl_cfg, lv_color_hex(0x5048A8), LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_cfg, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_center(lbl_cfg);
    lv_obj_move_foreground(btn_cfg);

    /* Teclado em layer_top: nao redimensiona o tabview (evita crash no btnmatrix). */
    s_dev.kb = lv_keyboard_create(lv_layer_top());
    lv_obj_add_flag(s_dev.kb, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_size(s_dev.kb, lv_pct(100), 220);
    lv_obj_align(s_dev.kb, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_keyboard_set_mode(s_dev.kb, LV_KEYBOARD_MODE_TEXT_LOWER);
    lv_obj_add_flag(s_dev.kb, LV_OBJ_FLAG_HIDDEN);

    /* ---------- Aba OS (ordem de servico — separador de remedios) ---------- */
    {
        lv_obj_t *t_os = lv_tabview_add_tab(tv, "OS");
        style_dashboard_tab_page(t_os);
        lv_obj_set_style_pad_all(t_os, 8, LV_PART_MAIN);
        lv_obj_set_style_pad_bottom(t_os, 68, LV_PART_MAIN);
        lv_obj_set_style_pad_row(t_os, 8, LV_PART_MAIN);

        /* Sem OS ativa: ecra cheio com botao para criar OS manual. */
        s_os.panel_idle = lv_obj_create(t_os);
        lv_obj_remove_style_all(s_os.panel_idle);
        lv_obj_set_width(s_os.panel_idle, lv_pct(100));
        lv_obj_set_flex_grow(s_os.panel_idle, 1);
        lv_obj_set_flex_flow(s_os.panel_idle, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(s_os.panel_idle, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(s_os.panel_idle, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *card_idle = make_card(s_os.panel_idle);
        lv_obj_set_width(card_idle, lv_pct(100));
        lv_obj_set_flex_grow(card_idle, 1);
        lv_obj_set_flex_flow(card_idle, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(card_idle, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_row(card_idle, 16, LV_PART_MAIN);

        lv_obj_t *idle_t = lv_label_create(card_idle);
        lv_label_set_text(idle_t, "Nenhuma OS ativa");
        lv_obj_set_style_text_font(idle_t, &lv_font_montserrat_42, LV_PART_MAIN);
        lv_obj_set_style_text_color(idle_t, COLOR_VALUE, LV_PART_MAIN);

        lv_obj_t *idle_h = lv_label_create(card_idle);
        lv_label_set_text(idle_h, "Crie uma OS manual para iniciar a separacao.");
        lv_obj_set_style_text_font(idle_h, &lv_font_montserrat_24, LV_PART_MAIN);
        lv_obj_set_style_text_color(idle_h, COLOR_LABEL, LV_PART_MAIN);
        lv_label_set_long_mode(idle_h, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(idle_h, lv_pct(100));

        lv_obj_t *b_new = lv_btn_create(card_idle);
        lv_obj_set_width(b_new, lv_pct(100));
        lv_obj_set_height(b_new, 56);
        lv_obj_set_style_radius(b_new, 14, LV_PART_MAIN);
        lv_obj_set_style_bg_color(b_new, COLOR_ACCENT, LV_PART_MAIN);
        lv_obj_add_event_cb(b_new, os_btn_nova_manual_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_t *lb = lv_label_create(b_new);
        lv_label_set_text(lb, "Nova OS manual");
        lv_obj_set_style_text_color(lb, lv_color_white(), LV_PART_MAIN);
        lv_obj_set_style_text_font(lb, &lv_font_montserrat_24, LV_PART_MAIN);
        lv_obj_center(lb);

        /* OS em curso: painel ativo em ecra cheio. */
        s_os.panel_active = lv_obj_create(t_os);
        lv_obj_remove_style_all(s_os.panel_active);
        lv_obj_set_width(s_os.panel_active, lv_pct(100));
        lv_obj_set_flex_grow(s_os.panel_active, 1);
        lv_obj_set_flex_flow(s_os.panel_active, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_row(s_os.panel_active, 8, LV_PART_MAIN);
        lv_obj_add_flag(s_os.panel_active, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_os.panel_active, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *card_hdr = make_card(s_os.panel_active);
        lv_obj_set_width(card_hdr, lv_pct(100));
        lv_obj_set_flex_grow(card_hdr, 2);
        make_card_label(card_hdr, "ORDEM DE SERVICO");

        lv_obj_t *ln = lv_label_create(card_hdr);
        lv_label_set_text(ln, "Numero da OS");
        lv_obj_set_style_text_font(ln, &lv_font_montserrat_24, LV_PART_MAIN);
        lv_obj_set_style_text_color(ln, COLOR_LABEL, LV_PART_MAIN);

        s_lbl_os_number = lv_label_create(card_hdr);
        lv_label_set_text(s_lbl_os_number, "--");
        lv_obj_set_style_text_font(s_lbl_os_number, &lv_font_montserrat_42, LV_PART_MAIN);
        lv_obj_set_style_text_color(s_lbl_os_number, COLOR_VALUE, LV_PART_MAIN);
        lv_obj_set_width(s_lbl_os_number, lv_pct(100));

        lv_obj_t *lc = lv_label_create(card_hdr);
        lv_label_set_text(lc, "Cliente");
        lv_obj_set_style_text_font(lc, &lv_font_montserrat_24, LV_PART_MAIN);
        lv_obj_set_style_text_color(lc, COLOR_LABEL, LV_PART_MAIN);

        s_lbl_os_client = lv_label_create(card_hdr);
        lv_label_set_text(s_lbl_os_client, "--");
        lv_obj_set_style_text_font(s_lbl_os_client, &lv_font_montserrat_24, LV_PART_MAIN);
        lv_obj_set_style_text_color(s_lbl_os_client, COLOR_VALUE, LV_PART_MAIN);
        lv_label_set_long_mode(s_lbl_os_client, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(s_lbl_os_client, lv_pct(100));

        lv_obj_t *col_l, *col_r;
        lv_obj_t *grid_os = build_grid_1x2(s_os.panel_active, &col_l, &col_r);
        lv_obj_set_flex_grow(grid_os, 1);
        lv_obj_set_style_pad_row(grid_os, 8, LV_PART_MAIN);
        lv_obj_set_style_pad_column(grid_os, 8, LV_PART_MAIN);

        lv_obj_t *card_m = make_card(col_l);
        lv_obj_set_size(card_m, lv_pct(100), lv_pct(100));
        lv_obj_set_flex_grow(card_m, 1);
        lv_obj_set_style_pad_all(card_m, 16, LV_PART_MAIN);
        lv_obj_set_style_pad_row(card_m, 8, LV_PART_MAIN);
        make_card_label(card_m, "REMEDIOS");

        s_lbl_os_meds = lv_label_create(card_m);
        lv_label_set_text(s_lbl_os_meds, "-- / --  separados");
        lv_obj_set_style_text_font(s_lbl_os_meds, &lv_font_montserrat_24, LV_PART_MAIN);
        lv_obj_set_style_text_color(s_lbl_os_meds, COLOR_ACCENT, LV_PART_MAIN);

        s_bar_os_meds = lv_bar_create(card_m);
        lv_obj_set_size(s_bar_os_meds, lv_pct(100), 12);
        lv_obj_set_style_bg_color(s_bar_os_meds, COLOR_BADGE_BG, LV_PART_MAIN);
        lv_obj_set_style_radius(s_bar_os_meds, 8, LV_PART_MAIN);
        lv_obj_set_style_bg_color(s_bar_os_meds, COLOR_OK, LV_PART_INDICATOR);
        lv_obj_set_style_radius(s_bar_os_meds, 8, LV_PART_INDICATOR);
        lv_bar_set_range(s_bar_os_meds, 0, 100);
        lv_bar_set_value(s_bar_os_meds, 0, LV_ANIM_OFF);

        lv_obj_t *card_e = make_card(col_r);
        lv_obj_set_size(card_e, lv_pct(100), lv_pct(100));
        lv_obj_set_flex_grow(card_e, 1);
        lv_obj_set_style_pad_all(card_e, 16, LV_PART_MAIN);
        lv_obj_set_style_pad_row(card_e, 8, LV_PART_MAIN);
        make_card_label(card_e, "TEMPO RESTANTE");

        s_lbl_os_eta = lv_label_create(card_e);
        lv_obj_set_style_text_font(s_lbl_os_eta, &lv_font_montserrat_24, LV_PART_MAIN);
        lv_obj_set_style_text_color(s_lbl_os_eta, COLOR_VALUE, LV_PART_MAIN);
        lv_obj_set_width(s_lbl_os_eta, lv_pct(100));

        s_lbl_os_eta_detail = lv_label_create(card_e);
        lv_obj_set_style_text_font(s_lbl_os_eta_detail, &lv_font_montserrat_14, LV_PART_MAIN);
        lv_obj_set_style_text_color(s_lbl_os_eta_detail, COLOR_LABEL, LV_PART_MAIN);
        lv_label_set_long_mode(s_lbl_os_eta_detail, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(s_lbl_os_eta_detail, lv_pct(100));

        /* Linha com Pausar | Cancelar lado a lado */
        lv_obj_t *btns_row = lv_obj_create(s_os.panel_active);
        lv_obj_remove_style_all(btns_row);
        lv_obj_set_width(btns_row, lv_pct(100));
        lv_obj_set_height(btns_row, 56);
        lv_obj_set_flex_flow(btns_row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(btns_row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_column(btns_row, 8, LV_PART_MAIN);
        lv_obj_clear_flag(btns_row, LV_OBJ_FLAG_SCROLLABLE);

        s_os.btn_pause = lv_btn_create(btns_row);
        lv_obj_set_flex_grow(s_os.btn_pause, 1);
        lv_obj_set_height(s_os.btn_pause, 52);
        lv_obj_set_style_radius(s_os.btn_pause, 12, LV_PART_MAIN);
        lv_obj_set_style_bg_color(s_os.btn_pause, lv_color_hex(0xEDE8F8), LV_PART_MAIN);
        lv_obj_set_style_bg_color(s_os.btn_pause, lv_color_hex(0xD8D0F0), LV_STATE_PRESSED);
        lv_obj_add_event_cb(s_os.btn_pause, os_btn_pausar_os_cb, LV_EVENT_CLICKED, NULL);
        s_os.lbl_btn_pause = lv_label_create(s_os.btn_pause);
        lv_label_set_text(s_os.lbl_btn_pause, "Pausar OS");
        lv_obj_set_style_text_font(s_os.lbl_btn_pause, &lv_font_montserrat_24, LV_PART_MAIN);
        lv_obj_set_style_text_color(s_os.lbl_btn_pause, lv_color_hex(0x3D3B7A), LV_PART_MAIN);
        lv_obj_center(s_os.lbl_btn_pause);
        lv_obj_add_flag(s_os.btn_pause, LV_OBJ_FLAG_HIDDEN);

        lv_obj_t *b_cancel = lv_btn_create(btns_row);
        lv_obj_set_flex_grow(b_cancel, 1);
        lv_obj_set_height(b_cancel, 52);
        lv_obj_set_style_radius(b_cancel, 12, LV_PART_MAIN);
        lv_obj_set_style_bg_color(b_cancel, lv_color_hex(0xFEE2E2), LV_PART_MAIN);
        lv_obj_set_style_bg_color(b_cancel, lv_color_hex(0xFECACA), LV_STATE_PRESSED);
        lv_obj_add_event_cb(b_cancel, os_btn_cancelar_os_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_t *lcancel = lv_label_create(b_cancel);
        lv_label_set_text(lcancel, "Cancelar OS");
        lv_obj_set_style_text_font(lcancel, &lv_font_montserrat_24, LV_PART_MAIN);
        lv_obj_set_style_text_color(lcancel, lv_color_hex(0xB91C1C), LV_PART_MAIN);
        lv_obj_center(lcancel);

        os_update_panels();
        ui_os_paint_labels();
    }

    /* ---------- Aba Motor de Passo ---------- */
    {
        lv_obj_t *t_at = lv_tabview_add_tab(tv, "Motor");
        style_dashboard_tab_page(t_at);
        lv_obj_set_style_pad_all(t_at, 8, LV_PART_MAIN);
        lv_obj_set_style_pad_bottom(t_at, 68, LV_PART_MAIN);
        lv_obj_set_style_pad_row(t_at, 8, LV_PART_MAIN);

        lv_obj_t *card_step = make_card(t_at);
        lv_obj_set_width(card_step, lv_pct(100));
        lv_obj_set_height(card_step, 110);
        lv_obj_set_flex_flow(card_step, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(card_step, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_bg_color(card_step, lv_color_hex(0xF7F5FF), LV_STATE_PRESSED);
        lv_obj_add_flag(card_step, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(card_step, stepper_open_cb, LV_EVENT_CLICKED, NULL);

        lv_obj_t *col_st = lv_obj_create(card_step);
        lv_obj_remove_style_all(col_st);
        lv_obj_set_flex_grow(col_st, 1);
        lv_obj_set_height(col_st, LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(col_st, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(col_st, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
        lv_obj_set_style_pad_row(col_st, 4, LV_PART_MAIN);
        lv_obj_clear_flag(col_st, LV_OBJ_FLAG_SCROLLABLE);

        make_card_label(col_st, "TESTE MOTOR DE PASSO");
        lv_obj_t *st_sub = lv_label_create(col_st);
        lv_label_set_text(st_sub, "STEP GPIO 20  ·  DIR GPIO 32");
        lv_obj_set_style_text_font(st_sub, &lv_font_montserrat_14, LV_PART_MAIN);
        lv_obj_set_style_text_color(st_sub, COLOR_LABEL, LV_PART_MAIN);
        lv_obj_t *st_hint = lv_label_create(col_st);
        lv_label_set_text(st_hint, "Toque para escolher o sentido");
        lv_obj_set_style_text_font(st_hint, &lv_font_montserrat_24, LV_PART_MAIN);
        lv_obj_set_style_text_color(st_hint, COLOR_VALUE, LV_PART_MAIN);

        historico_item_bubble_events(card_step);
    }

    /* ---------- Aba Historico (lista de OS anteriores) ---------- */
    {
        lv_obj_t *t_hist = lv_tabview_add_tab(tv, "Historico");
        style_dashboard_tab_page(t_hist);
        lv_obj_set_style_pad_all(t_hist, 8, LV_PART_MAIN);
        lv_obj_set_style_pad_row(t_hist, 8, LV_PART_MAIN);

        lv_obj_t *hint = lv_label_create(t_hist);
        lv_label_set_text(hint, "Toque num cartao para ver detalhes ou excluir.");
        lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, LV_PART_MAIN);
        lv_obj_set_style_text_color(hint, COLOR_LABEL, LV_PART_MAIN);
        lv_obj_set_width(hint, lv_pct(100));

        s_hist_scroll = lv_obj_create(t_hist);
        lv_obj_remove_style_all(s_hist_scroll);
        lv_obj_set_width(s_hist_scroll, lv_pct(100));
        lv_obj_set_flex_grow(s_hist_scroll, 1);
        lv_obj_set_height(s_hist_scroll, lv_pct(100));
        lv_obj_add_flag(s_hist_scroll, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_scroll_dir(s_hist_scroll, LV_DIR_VER);
        lv_obj_set_scrollbar_mode(s_hist_scroll, LV_SCROLLBAR_MODE_AUTO);
        lv_obj_set_flex_flow(s_hist_scroll, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(s_hist_scroll, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
        lv_obj_set_style_pad_row(s_hist_scroll, 10, LV_PART_MAIN);

        historico_seed_demo_if_empty();
        historico_rebuild_list();
    }

    /* Sincroniza abas apos construir tudo (evita indice invalido no btnmatrix). */
    lv_tabview_set_act(tv, 0, LV_ANIM_OFF);
    lv_obj_move_foreground(btn_cfg);

    if (s_dash_timer) {
        lv_timer_del(s_dash_timer);
        s_dash_timer = NULL;
    }
    s_dash_timer = lv_timer_create(dashboard_update_cb, 1000, NULL);
    dashboard_update_cb(NULL);

    if (s_cmd_timer) {
        lv_timer_del(s_cmd_timer);
        s_cmd_timer = NULL;
    }
    s_cmd_timer = lv_timer_create(serial_cmd_timer_cb, 50, NULL);

    /* Se uma OS foi concluida enquanto o utilizador estava fora, mostra
     * agora o modal de conclusao que ficou pendente. */
    if (s_os_concl_pending) {
        s_os_concl_pending = false;
        os_show_concluida_modal(s_os_concl_num, s_os_concl_total);
    }
}

static void build_login_ui_into(lv_obj_t *scr)
{
    if (!scr) {
        return;
    }

    memset(&s_login, 0, sizeof(s_login));
    s_login.scr = scr;
    lv_obj_clean(scr);

    lv_obj_set_style_bg_color(scr, COLOR_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    /* Sem flex no ecra: evita altura de scroll incorrecta com filho em ecra cheio. */
    lv_obj_set_layout(scr, 0);
    lv_obj_set_style_pad_all(scr, 0, LV_PART_MAIN);

    /* Painel branco em ecra inteiro; o conteudo real vai numa coluna com altura = conteudo. */
    lv_obj_t *panel = make_card(scr);
    lv_obj_set_size(panel, lv_pct(100), lv_pct(100));
    lv_obj_align(panel, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_add_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(panel, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(panel, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_flex_align(panel, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_radius(panel, 0, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(panel, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(panel, 0, LV_PART_MAIN);
    /* Subir LV_EVENT_CLICKED ao ecra para fechar o teclado ao tocar no formulario. */
    lv_obj_add_flag(panel, LV_OBJ_FLAG_EVENT_BUBBLE);

    lv_obj_t *col = lv_obj_create(panel);
    lv_obj_remove_style_all(col);
    lv_obj_set_width(col, lv_pct(100));
    lv_obj_set_style_max_width(col, 560, LV_PART_MAIN);
    lv_obj_set_height(col, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(col, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(col, 12, LV_PART_MAIN);
    lv_obj_clear_flag(col, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *t = lv_label_create(col);
    lv_label_set_text(t, "Iniciar sessao");
    lv_obj_set_style_text_font(t, &lv_font_montserrat_42, LV_PART_MAIN);
    lv_obj_set_style_text_color(t, COLOR_VALUE, LV_PART_MAIN);

    lv_obj_t *lu = lv_label_create(col);
    lv_label_set_text(lu, "Usuario");
    lv_obj_set_style_text_font(lu, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(lu, COLOR_LABEL, LV_PART_MAIN);

    s_login.ta_user = lv_textarea_create(col);
    lv_textarea_set_one_line(s_login.ta_user, true);
    lv_textarea_set_max_length(s_login.ta_user, 48);
    lv_textarea_set_placeholder_text(s_login.ta_user, "nome de usuario");
    style_login_ta(s_login.ta_user);
    lv_obj_set_width(s_login.ta_user, lv_pct(100));
    lv_obj_set_style_min_height(s_login.ta_user, 52, LV_PART_MAIN);
    lv_obj_add_event_cb(s_login.ta_user, login_ta_activate_cb, LV_EVENT_FOCUSED, &s_login);
    lv_obj_add_event_cb(s_login.ta_user, login_ta_activate_cb, LV_EVENT_CLICKED, &s_login);

    lv_obj_t *lp = lv_label_create(col);
    lv_label_set_text(lp, "senha");
    lv_obj_set_style_text_font(lp, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(lp, COLOR_LABEL, LV_PART_MAIN);

    s_login.ta_pass = lv_textarea_create(col);
    lv_textarea_set_one_line(s_login.ta_pass, true);
    lv_textarea_set_password_mode(s_login.ta_pass, true);
    lv_textarea_set_max_length(s_login.ta_pass, 48);
    lv_textarea_set_placeholder_text(s_login.ta_pass, "********");
    style_login_ta(s_login.ta_pass);
    lv_obj_set_width(s_login.ta_pass, lv_pct(100));
    lv_obj_set_style_min_height(s_login.ta_pass, 52, LV_PART_MAIN);
    lv_obj_add_event_cb(s_login.ta_pass, login_ta_activate_cb, LV_EVENT_FOCUSED, &s_login);
    lv_obj_add_event_cb(s_login.ta_pass, login_ta_activate_cb, LV_EVENT_CLICKED, &s_login);

    s_login.lbl_err = lv_label_create(col);
    lv_label_set_text(s_login.lbl_err, "");
    lv_obj_set_style_text_font(s_login.lbl_err, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_login.lbl_err, lv_color_hex(0xDC2626), LV_PART_MAIN);
    lv_obj_add_flag(s_login.lbl_err, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *hint = lv_label_create(col);
    lv_label_set_text_fmt(hint,
                          "Demo: utilizador \"%s\"  |  palavra-passe \"%s\"",
                          LOGIN_DEMO_USER, LOGIN_DEMO_PASS);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(hint, COLOR_LABEL, LV_PART_MAIN);
    lv_label_set_long_mode(hint, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(hint, lv_pct(100));

    lv_obj_t *btn = lv_btn_create(col);
    lv_obj_set_size(btn, lv_pct(100), 54);
    lv_obj_set_style_radius(btn, 14, LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn, COLOR_ACCENT, LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn, COLOR_ACCENT_DARK, LV_STATE_PRESSED);
    lv_obj_add_event_cb(btn, on_login_btn_clicked, LV_EVENT_CLICKED, &s_login);
    lv_obj_t *bl = lv_label_create(btn);
    lv_label_set_text(bl, "Entrar");
    lv_obj_set_style_text_color(bl, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(bl, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_center(bl);

    /* Toques nos filhos sobem ao painel para fechar o teclado (alvo original via get_target). */
    login_bubble_events_up(col);

    /* Teclado flutuante: nao entra na coluna flex, nao rouba altura ao formulario. */
    s_login.kb = lv_keyboard_create(scr);
    lv_obj_add_flag(s_login.kb, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_size(s_login.kb, lv_pct(100), 220);
    lv_obj_align(s_login.kb, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_keyboard_set_mode(s_login.kb, LV_KEYBOARD_MODE_TEXT_LOWER);
    lv_obj_add_flag(s_login.kb, LV_OBJ_FLAG_HIDDEN);

    lv_obj_add_event_cb(scr, login_clicked_dismiss_kb, LV_EVENT_CLICKED, &s_login);

    lv_obj_update_layout(scr);
}


static void build_ui(void)
{
    build_splash();
}

void ui_os_set_ordem(const char *numero_os, const char *nome_cliente)
{
    if (!lvgl_port_lock(pdMS_TO_TICKS(50))) {
        APP_WARN("ui_os_set_ordem: lock LVGL");
        return;
    }
    str_copy_trunc(s_os_num_str, sizeof s_os_num_str, numero_os);
    str_copy_trunc(s_os_client_str, sizeof s_os_client_str, nome_cliente);
    if (numero_os && numero_os[0]) {
        s_os_active        = true;
        s_os_sess_start_us = esp_timer_get_time();
        os_update_panels();
    }
    ui_os_paint_labels();
    lvgl_port_unlock();
}

void ui_os_set_remedios(uint32_t separados, uint32_t total)
{
    if (!lvgl_port_lock(pdMS_TO_TICKS(50))) {
        APP_WARN("ui_os_set_remedios: lock LVGL");
        return;
    }
    s_os_meds_done  = separados;
    s_os_meds_total = total;
    ui_os_paint_labels();
    lvgl_port_unlock();
}

void ui_os_registar_tempo_pegada_ms(uint32_t duracao_ms)
{
    if (!lvgl_port_lock(pdMS_TO_TICKS(50))) {
        APP_WARN("ui_os_registar_tempo_pegada_ms: lock LVGL");
        return;
    }
    s_os_pick_ms_sum += (uint64_t)duracao_ms;
    s_os_pick_samples++;
    ui_os_paint_labels();
    lvgl_port_unlock();
}

void ui_historico_registar_os(const char *numero_os, const char *nome_cliente,
                              uint32_t separados, uint32_t total, uint32_t duracao_ms)
{
    if (!lvgl_port_lock(pdMS_TO_TICKS(50))) {
        APP_WARN("ui_historico_registar_os: lock LVGL");
        return;
    }
    historico_push(numero_os, nome_cliente, separados, total, duracao_ms, OS_HIST_CONCLUIDA, NULL);
    lvgl_port_unlock();
}

static void touch_log_task(void *arg)
{
    (void)arg;
    uint16_t last_x = 0xFFFF, last_y = 0xFFFF;
    bool last_pressed = false;

    while (1) {
        if (touch_read() == ESP_OK) {
            uint16_t x, y;
            bool pressed;
            get_coor(&x, &y, &pressed);

            if (pressed) {
                s_last_touch_x = x;
                s_last_touch_y = y;
                s_last_touch_pressed = true;
                if (!last_pressed || x != last_x || y != last_y) {
                    APP_INFO("Touch X=%u Y=%u", x, y);
                    last_x = x;
                    last_y = y;
                }
            } else {
                s_last_touch_pressed = false;
                if (last_pressed) {
                    APP_INFO("Touch liberado");
                }
            }
            last_pressed = pressed;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void fail_loop(const char *who, esp_err_t err)
{
    while (1) {
        APP_ERROR("[%s] init falhou: %s", who, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void system_init(void)
{
    esp_err_t err;

    /* 0. NVS flash — persistência de configurações. */
    nvs_init();

    /* 0a. GPIOs dos LEDs (relé) e motor de passo. */
    led_gpio_init();
    stepper_init();
    /* Carrega nome/código do robô na RAM para uso imediato. */
    strncpy(s_dev_robot_name, "Separador P4", sizeof(s_dev_robot_name) - 1);
    strncpy(s_dev_robot_code, "ESP32-P4-001", sizeof(s_dev_robot_code) - 1);
    nvs_load_device_config(s_dev_robot_name, sizeof s_dev_robot_name,
                           s_dev_robot_code, sizeof s_dev_robot_code);
    /* Carrega configuração do motor de passo */
    s_stepper_rpm       = nvs_load_u16(NVS_KEY_STEP_RPM,  60);
    s_stepper_steps_rev = nvs_load_u16(NVS_KEY_STEP_SPR,  200);
    s_stepper_ms_idx    = nvs_load_u8 (NVS_KEY_STEP_MSTP, 0);
    if (s_stepper_rpm < 1 || s_stepper_rpm > 2000)  { s_stepper_rpm = 60; }
    if (s_stepper_steps_rev < 1)                     { s_stepper_steps_rev = 200; }
    if (s_stepper_ms_idx >= MICROSTEP_COUNT)         { s_stepper_ms_idx = 0; }
    APP_INFO("NVS ok — nome='%s' codigo='%s' stepper=%uRPM/%u/%ux",
             s_dev_robot_name, s_dev_robot_code,
             s_stepper_rpm, s_stepper_steps_rev,
             (unsigned)MICROSTEP_OPTIONS[s_stepper_ms_idx]);

    /* 1. LDOs internos: alimentam o transmissor MIPI-DSI do P4. */
    esp_ldo_channel_config_t ldo3_cfg = { .chan_id = 3, .voltage_mv = 2500 };
    err = esp_ldo_acquire_channel(&ldo3_cfg, &s_ldo3);
    if (err != ESP_OK) fail_loop("LDO3", err);

    esp_ldo_channel_config_t ldo4_cfg = { .chan_id = 4, .voltage_mv = 3300 };
    err = esp_ldo_acquire_channel(&ldo4_cfg, &s_ldo4);
    if (err != ESP_OK) fail_loop("LDO4", err);
    APP_INFO("LDOs ligados (3=2.5V, 4=3.3V)");

    /* 2. I2C master (touch GT911). */
    err = i2c_init();
    if (err != ESP_OK) fail_loop("I2C", err);
    APP_INFO("I2C ok (SDA=7, SCL=8)");

    /* 3. Touch GT911 antes do display (handle usado em lvgl_init). */
    err = touch_init();
    if (err != ESP_OK) fail_loop("GT911", err);
    APP_INFO("Touch GT911 ok");

    /* 4. LCD MIPI-DSI JD9165 + LVGL 1024×600. */
    err = display_init();
    if (err != ESP_OK) fail_loop("Display", err);
    APP_INFO("LCD JD9165 + LVGL ok (1024x600)");

    /* 5. Backlight no maximo. */
    err = set_lcd_blight(100);
    if (err != ESP_OK) fail_loop("Backlight", err);
    APP_INFO("Backlight 100%%");
}

void app_main(void)
{
    APP_INFO("==> Guition ESP32-P4 7'' JC1060 - Dashboard LVGL");

    system_init();
    build_ui();
    /* IMPORTANTE: nao chamar lv_refr_now aqui (ou em qualquer task que nao
     * seja a taskLVGL) sem lvgl_port_lock/unlock. Fazer isso dispara
     * esp_lcd_panel_draw_bitmap em paralelo com a propria taskLVGL,
     * o DMA2D rejeita ("previous draw operation is not finished") e o
     * lv_refr_now fica em busy-wait esperando flush_ready -> Task WDT
     * em IDLE0. A taskLVGL ja faz o primeiro refresh sozinha. */

    BaseType_t ok = xTaskCreate(touch_log_task, "touch_log", 4096, NULL, 4, NULL);
    if (ok != pdPASS) {
        APP_WARN("Nao foi possivel criar a task de log do touch");
    }

    /* Fila e task de recepcao de comandos do gateway (Web → ESP32). */
    s_cmd_queue = xQueueCreate(SERIAL_CMD_QUEUE_LEN, sizeof(serial_cmd_t));
    if (s_cmd_queue) {
        BaseType_t rc = xTaskCreate(serial_cmd_task, "serial_cmd", 4096, NULL, 3, NULL);
        if (rc != pdPASS) {
            APP_WARN("Nao foi possivel criar a task serial_cmd");
        } else {
            APP_INFO("Task serial_cmd iniciada (aguarda CMDs do gateway)");
        }
    } else {
        APP_WARN("Falha ao criar s_cmd_queue");
    }

    APP_INFO("Setup concluido - dashboard ativo");
}
