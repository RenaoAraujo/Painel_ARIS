/*
 * Dashboard LVGL - Guition JC1060P470C (ESP32-P4 7" 1024×600 MIPI-DSI).
 * UI: login -> dashboard com 3 abas (OS, Historico, Dispositivo).
 */

#include "app_main.h"
#include "esp_random.h"
#include "freertos/queue.h"
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>

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

static char       s_os_num_str[OS_NUM_BUF_SZ]       = "";
static char       s_os_client_str[OS_CLIENT_BUF_SZ] = "";
static uint32_t   s_os_meds_total   = 0;
static uint32_t   s_os_meds_done    = 0;
static uint64_t   s_os_pick_ms_sum  = 0;
static uint32_t   s_os_pick_samples = 0;
static uint64_t   s_os_sess_start_us = 0;
static bool       s_os_active       = false;
static lv_timer_t *s_os_pick_timer  = NULL;

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

static void build_login_ui_into(lv_obj_t *scr);
static void build_dashboard_into(lv_obj_t *scr);
static void serial_cmd_timer_cb(lv_timer_t *t);
static void os_show_concluida_modal(const char *num, uint32_t total);
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

/* Paleta. */
#define COLOR_BG          lv_color_hex(0xEEF2F7)
#define COLOR_CARD        lv_color_hex(0xFFFFFF)
#define COLOR_BORDER      lv_color_hex(0xE5E7EB)
#define COLOR_LABEL       lv_color_hex(0x6B7280)
#define COLOR_VALUE       lv_color_hex(0x111827)
#define COLOR_ACCENT      lv_color_hex(0x2563EB)
#define COLOR_ACCENT_DARK lv_color_hex(0x1E40AF)
#define COLOR_OK          lv_color_hex(0x10B981)
#define COLOR_BADGE_BG    lv_color_hex(0xE5E7EB)

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
    lv_obj_set_style_shadow_color(card, lv_color_hex(0x9CA3AF), LV_PART_MAIN);
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

    historico_close_modal();
    os_pick_timer_stop();
    os_close_modal();
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
    lv_obj_set_style_text_font(tab_bar, &lv_font_montserrat_24, LV_PART_ITEMS);
    lv_obj_set_style_text_color(tab_bar, COLOR_LABEL, LV_PART_ITEMS);
    lv_obj_set_style_bg_color(tab_bar, COLOR_ACCENT, LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_text_color(tab_bar, lv_color_white(), LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_radius(tab_bar, 12, LV_PART_ITEMS);
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
    const char *raw_name = (s_dev.ta_nome && lv_textarea_get_text(s_dev.ta_nome)[0])
                           ? lv_textarea_get_text(s_dev.ta_nome)
                           : "Separador P4";
    const char *raw_id   = (s_dev.ta_id && lv_textarea_get_text(s_dev.ta_id)[0])
                           ? lv_textarea_get_text(s_dev.ta_id)
                           : BRIDGE_ROBOT_CODE;
    char en[DEV_ROBOT_NAME_SZ * 2];
    char eid[DEV_ROBOT_ID_SZ * 2];
    serial_bridge_escape(en,  sizeof en,  raw_name);
    serial_bridge_escape(eid, sizeof eid, raw_id);
    printf("BRIDGE:{\"ev\":\"pong\",\"robot\":\"%s\",\"name\":\"%s\"}\n", eid, en);
    APP_INFO("[BRIDGE pong] robot=%s name=%s", raw_id, raw_name);
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
            if (s_dev.ta_nome && cmd.client[0]) {
                lv_textarea_set_text(s_dev.ta_nome, cmd.client);
            }
            if (s_dev.ta_id && cmd.num[0]) {
                lv_textarea_set_text(s_dev.ta_id, cmd.num);
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
    os_show_concluida_modal(num_copy, done_snap);
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
    APP_INFO("OS %s pausada", s_os_num_str);
}

static void os_resume(void)
{
    if (!s_os_active || !s_os_paused) { return; }
    s_os_paused = false;
    /* Se há um pick em voo aguardando o deferred, ele agendará o próximo pick
     * sozinho ao ser processado; caso contrário, agendamos aqui. */
    if (!s_bridge_unit_pending) {
        os_schedule_next_pick();
    }
    ui_os_paint_labels();
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
    if (s_os_hist_count > 0) {
        return;
    }
    str_copy_trunc(s_os_hist[0].numero, sizeof s_os_hist[0].numero, "2025-01720");
    str_copy_trunc(s_os_hist[0].cliente, sizeof s_os_hist[0].cliente, "Farmacia Sul");
    s_os_hist[0].separados = 48;
    s_os_hist[0].total     = 48;
    s_os_hist[0].duracao_ms = 18U * 60U * 1000U + 40U * 1000U;
    s_os_hist[0].estado     = OS_HIST_CONCLUIDA;
    s_os_hist[0].motivo[0]  = '\0';

    str_copy_trunc(s_os_hist[1].numero, sizeof s_os_hist[1].numero, "2025-01698");
    str_copy_trunc(s_os_hist[1].cliente, sizeof s_os_hist[1].cliente, "Hospital Dia Norte");
    s_os_hist[1].separados = 120;
    s_os_hist[1].total     = 120;
    s_os_hist[1].duracao_ms = 52U * 60U * 1000U + 15U * 1000U;
    s_os_hist[1].estado     = OS_HIST_CONCLUIDA;
    s_os_hist[1].motivo[0]  = '\0';

    str_copy_trunc(s_os_hist[2].numero, sizeof s_os_hist[2].numero, "2025-01655");
    str_copy_trunc(s_os_hist[2].cliente, sizeof s_os_hist[2].cliente, "Clinica Vida");
    s_os_hist[2].separados = 12;
    s_os_hist[2].total     = 30;
    s_os_hist[2].duracao_ms = 6U * 60U * 1000U + 33U * 1000U;
    s_os_hist[2].estado     = OS_HIST_CONCLUIDA;
    s_os_hist[2].motivo[0]  = '\0';

    s_os_hist_count = 3;
    s_dev_os_separadas += 3;
    s_dev_remed_separados += 48u + 120u + 12u;
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

    historico_close_modal();
    os_pick_timer_stop();
    os_close_modal();
    dev_kb_destroy();
    s_hist_scroll = NULL;

    memset(&s_os, 0, sizeof(s_os));
    memset(&s_dev, 0, sizeof(s_dev));
    s_os_active = false;

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

    /* Sair: flutuante para nao roubar altura ao tabview. */
    lv_obj_t *btn_logout = lv_btn_create(scr);
    lv_obj_add_flag(btn_logout, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_size(btn_logout, 112, 44);
    lv_obj_set_style_radius(btn_logout, 12, LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn_logout, lv_color_hex(0xFEE2E2), LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn_logout, lv_color_hex(0xFECACA), LV_STATE_PRESSED);
    lv_obj_set_style_border_width(btn_logout, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(btn_logout, lv_color_hex(0xFCA5A5), LV_PART_MAIN);
    lv_obj_add_event_cb(btn_logout, on_logout_clicked, LV_EVENT_CLICKED, scr);
    lv_obj_align(btn_logout, LV_ALIGN_BOTTOM_RIGHT, -14, -14);
    lv_obj_t *lbl_out = lv_label_create(btn_logout);
    lv_label_set_text(lbl_out, "Sair");
    lv_obj_set_style_text_color(lbl_out, lv_color_hex(0xB91C1C), LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_out, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_center(lbl_out);
    lv_obj_move_foreground(btn_logout);

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

        lv_obj_t *b_cancel = lv_btn_create(s_os.panel_active);
        lv_obj_set_width(b_cancel, lv_pct(100));
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

    /* ---------- Aba Dispositivo (identificacao + totais de producao) ---------- */
    {
        lv_obj_t *t_dev = lv_tabview_add_tab(tv, "Dispositivo");
        style_dashboard_tab_page(t_dev);
        lv_obj_set_style_pad_all(t_dev, 8, LV_PART_MAIN);
        lv_obj_set_style_pad_row(t_dev, 10, LV_PART_MAIN);

        lv_obj_t *card_id = make_card(t_dev);
        lv_obj_set_width(card_id, lv_pct(100));
        lv_obj_set_flex_grow(card_id, 1);
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
        lv_textarea_set_text(s_dev.ta_nome, "Separador P4");
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
        lv_textarea_set_text(s_dev.ta_id, "ESP32-P4-001");
        lv_obj_add_event_cb(s_dev.ta_id, dev_ta_activate_cb, LV_EVENT_FOCUSED, &s_dev);
        lv_obj_add_event_cb(s_dev.ta_id, dev_ta_activate_cb, LV_EVENT_CLICKED, &s_dev);
        lv_obj_add_event_cb(s_dev.ta_id, dev_ta_defocus_cb, LV_EVENT_DEFOCUSED, &s_dev);

        lv_obj_t *card_tot = make_card(t_dev);
        lv_obj_set_width(card_tot, lv_pct(100));
        lv_obj_set_flex_grow(card_tot, 1);
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
    }

    /* Sincroniza abas apos construir tudo (evita indice invalido no btnmatrix). */
    lv_tabview_set_act(tv, 0, LV_ANIM_OFF);
    lv_obj_move_foreground(btn_logout);

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

static void build_login_ui(void)
{
    if (!lvgl_port_lock(0)) {
        APP_ERROR("Falha ao obter o lock do LVGL");
        return;
    }
    build_login_ui_into(lv_scr_act());
    lvgl_port_unlock();
}

static void build_ui(void)
{
    build_login_ui();
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
