# Guition JC1060 (7" 1024×600) — ESP32-P4 + LVGL

Projeto em **ESP-IDF 6.x** / **LVGL 8.3** para **Guition JC1060P470C-I-W-Y** (ex.: [AliExpress](https://pt.aliexpress.com/item/1005010425193499.html)):

- ESP32-P4, 32 MB PSRAM, 16 MB flash.
- LCD 7" **1024×600** via **MIPI-DSI** com **JD9165**.
- Touch **GT911** em I2C (**SDA GPIO7**, **SCL GPIO8**), endereço típico **0x5D**.

Resolução lógica: **paisagem 1024×600** (sem `sw_rotate` no `esp_lvgl_port`).

## Estrutura

```
esp32p4/
├── main/
├── components/
│   ├── bsp_i2c/          # I2C (7/8)
│   ├── bsp_display/      # GT911
│   ├── bsp_illuminate/   # JD9165 + LVGL + backlight
│   └── esp_lcd_touch_gsl3680/  # não usado (kit antigo 10.1")
└── main/idf_component.yml    # jd9165 + gt911 + lvgl_port + lvgl
```

## Pinout (JC1060 — referências da comunidade)

| Função | GPIO |
|--------|------|
| I2C SDA | 7 |
| I2C SCL | 8 |
| LCD RST | **5** (se a placa não arrancar o vídeo, experimente **27** — ver `bsp_illuminate.h`) |
| Backlight | 23 |

Touch **RST/INT** neste BSP ficam em `GPIO_NUM_NC` (init só por I2C + reset interno do driver); se o fabricante tiver pinos dedicados e o touch falhar, veja `esp_lcd_touch_gt911` e os macros de endereço (`0x5D` / `0x14`).

## Build

```bash
idf.py fullclean
idf.py set-target esp32p4
idf.py build flash monitor
```

Na primeira build após mudar `idf_component.yml`, o gestor descarrega **esp_lcd_jd9165** e **esp_lcd_touch_gt911**.

## Kit 10.1" JC8012 (800×1280, JD9365, GSL3680)

A configuração por defeito passou a ser o **JC1060 7"**. Para o ecrã 10.1" é preciso voltar a `esp_lcd_jd9365`, resolução 800×1280, `esp_lcd_touch_gsl3680`, pins GSL3680/RST 27 e o `bsp_illuminate.c` antigo.

## Inicialização do JD9165

O componente oficial só envia SLPOUT/DISPON; neste módulo Guition costuma ser precisa uma sequência longa. Está em `components/bsp_illuminate/include/bsp_jd9165_guition_init.h` (base comunitária, ex. [LovyanGFX #803](https://github.com/lovyan03/LovyanGFX/issues/803)).

## Timing DPI

Em `bsp_illuminate.c`: **52 MHz**, HSYNC **20 / 160 / 160**, VSYNC **10 / 23 / 12** (alinhado a relatos de [MiniWebRadio / Guition 7"](https://github.com/schreibfaul1/ESP32-MiniWebRadio/issues/786)). Se a imagem tremer ou sair deslocada, compare com o macro `JD9165_1024_600_PANEL_60HZ_DPI_CONFIG_CF` em [`esp_lcd_jd9165`](https://components.espressif.com/components/espressif/esp_lcd_jd9165).

## Créditos

- **JD9165**: [`espressif/esp_lcd_jd9165`](https://components.espressif.com/components/espressif/esp_lcd_jd9165)
- **GT911**: [`espressif/esp_lcd_touch_gt911`](https://components.espressif.com/components/espressif/esp_lcd_touch_gt911)
