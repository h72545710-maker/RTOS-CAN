/*
 * ESP32-S3 CAN HMI LCD demo
 *
 * Board: Waveshare ESP32-S3-Touch-LCD-2.8
 * Screen: ST7789 240x320 with LVGL
 *
 * This sketch uses demo frames by default. Change USE_REAL_CAN to 1 after
 * connecting an external 3.3 V CAN transceiver to GPIO18/GPIO15.
 */

#include <Arduino.h>
#include "driver/twai.h"
#include "Display_ST7789.h"
#include "LVGL_Driver.h"

#define USE_REAL_CAN 0

static constexpr gpio_num_t CAN_TX_GPIO = GPIO_NUM_18;
static constexpr gpio_num_t CAN_RX_GPIO = GPIO_NUM_15;

typedef struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
} CanFrame;

typedef struct {
  uint32_t rx_count;
  uint32_t tx_count;
  uint32_t err_count;
  uint32_t last_id;
  uint8_t last_counter;
  uint8_t last_data[8];
  uint32_t last_update_ms;
  bool bus_ok;
} CanDashboardState;

static CanDashboardState dashboard = {0};

static lv_obj_t *bus_value;
static lv_obj_t *mode_value;
static lv_obj_t *rx_value;
static lv_obj_t *err_value;
static lv_obj_t *id_value;
static lv_obj_t *counter_value;
static lv_obj_t *data_value;
static lv_obj_t *age_value;

static bool can_init(void);
static bool can_receive(CanFrame *frame);
static void make_demo_frame(CanFrame *frame);
static void handle_frame(const CanFrame *frame);
static void ui_create(void);
static void ui_update(void);
static lv_obj_t *ui_make_value(lv_obj_t *parent, const char *name, int y);

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("ESP32-S3 CAN HMI LCD starting...");

  Backlight_Init();
  LCD_Init();
  Lvgl_Init();
  ui_create();

  dashboard.bus_ok = can_init();
}

void loop() {
  CanFrame frame = {0};

  if (USE_REAL_CAN) {
    if (can_receive(&frame)) {
      handle_frame(&frame);
    }
  } else {
    static uint32_t last_demo_ms = 0;
    if (millis() - last_demo_ms >= 1000) {
      last_demo_ms = millis();
      make_demo_frame(&frame);
      handle_frame(&frame);
    }
  }

  static uint32_t last_ui_ms = 0;
  if (millis() - last_ui_ms >= 200) {
    last_ui_ms = millis();
    ui_update();
  }

  Lvgl_Loop();
  delay(5);
}

static bool can_init(void) {
  if (!USE_REAL_CAN) {
    return true;
  }

  twai_general_config_t general_config =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
  twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(&general_config, &timing_config, &filter_config);
  if (err != ESP_OK) {
    dashboard.err_count++;
    return false;
  }

  err = twai_start();
  if (err != ESP_OK) {
    dashboard.err_count++;
    return false;
  }

  return true;
}

static bool can_receive(CanFrame *frame) {
  twai_message_t message = {0};
  esp_err_t err = twai_receive(&message, pdMS_TO_TICKS(20));

  if (err == ESP_ERR_TIMEOUT) {
    return false;
  }

  if (err != ESP_OK) {
    dashboard.err_count++;
    return false;
  }

  if (message.rtr) {
    return false;
  }

  frame->id = message.identifier;
  frame->dlc = message.data_length_code;
  for (uint8_t i = 0; i < frame->dlc && i < sizeof(frame->data); i++) {
    frame->data[i] = message.data[i];
  }

  return true;
}

static void make_demo_frame(CanFrame *frame) {
  static uint8_t counter = 0;

  frame->id = 0x123;
  frame->dlc = 8;
  frame->data[0] = counter++;
  frame->data[1] = 0x11;
  frame->data[2] = 0x22;
  frame->data[3] = 0x33;
  frame->data[4] = 0x44;
  frame->data[5] = 0x55;
  frame->data[6] = 0x66;
  frame->data[7] = 0x77;
}

static void handle_frame(const CanFrame *frame) {
  dashboard.rx_count++;
  dashboard.last_id = frame->id;
  dashboard.last_counter = frame->dlc > 0 ? frame->data[0] : 0;
  dashboard.last_update_ms = millis();

  for (uint8_t i = 0; i < sizeof(dashboard.last_data); i++) {
    dashboard.last_data[i] = i < frame->dlc ? frame->data[i] : 0;
  }
}

static void ui_create(void) {
  lv_obj_t *screen = lv_scr_act();
  lv_obj_set_style_bg_color(screen, lv_color_hex(0x101820), 0);
  lv_obj_set_style_text_color(screen, lv_color_hex(0xEAF2F8), 0);
  lv_obj_set_style_text_font(screen, &lv_font_montserrat_14, 0);

  lv_obj_t *title = lv_label_create(screen);
  lv_label_set_text(title, "CAN Monitor");
  lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(title, LV_ALIGN_TOP_LEFT, 14, 12);

  bus_value = ui_make_value(screen, "Bus", 52);
  mode_value = ui_make_value(screen, "Mode", 82);
  rx_value = ui_make_value(screen, "RX Count", 112);
  err_value = ui_make_value(screen, "Errors", 142);
  id_value = ui_make_value(screen, "Last ID", 172);
  counter_value = ui_make_value(screen, "Counter", 202);

  lv_obj_t *data_title = lv_label_create(screen);
  lv_label_set_text(data_title, "Data");
  lv_obj_set_style_text_color(data_title, lv_color_hex(0x9FB3C8), 0);
  lv_obj_align(data_title, LV_ALIGN_TOP_LEFT, 14, 238);

  data_value = lv_label_create(screen);
  lv_obj_set_style_text_font(data_value, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(data_value, lv_color_hex(0x5EEAD4), 0);
  lv_obj_align(data_value, LV_ALIGN_TOP_LEFT, 14, 262);

  age_value = lv_label_create(screen);
  lv_obj_set_style_text_color(age_value, lv_color_hex(0x9FB3C8), 0);
  lv_obj_align(age_value, LV_ALIGN_BOTTOM_LEFT, 14, -10);
}

static lv_obj_t *ui_make_value(lv_obj_t *parent, const char *name, int y) {
  lv_obj_t *label = lv_label_create(parent);
  lv_label_set_text(label, name);
  lv_obj_set_style_text_color(label, lv_color_hex(0x9FB3C8), 0);
  lv_obj_align(label, LV_ALIGN_TOP_LEFT, 14, y);

  lv_obj_t *value = lv_label_create(parent);
  lv_obj_set_style_text_font(value, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(value, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(value, LV_ALIGN_TOP_RIGHT, -14, y - 2);
  return value;
}

static void ui_update(void) {
  char buf[96];

  lv_label_set_text(bus_value, dashboard.bus_ok ? "OK" : "WAIT");
  lv_label_set_text(mode_value, USE_REAL_CAN ? "CAN" : "DEMO");

  snprintf(buf, sizeof(buf), "%lu", static_cast<unsigned long>(dashboard.rx_count));
  lv_label_set_text(rx_value, buf);

  snprintf(buf, sizeof(buf), "%lu", static_cast<unsigned long>(dashboard.err_count));
  lv_label_set_text(err_value, buf);

  snprintf(buf, sizeof(buf), "0x%03lX", static_cast<unsigned long>(dashboard.last_id));
  lv_label_set_text(id_value, buf);

  snprintf(buf, sizeof(buf), "%u", dashboard.last_counter);
  lv_label_set_text(counter_value, buf);

  snprintf(buf, sizeof(buf), "%02X %02X %02X %02X\n%02X %02X %02X %02X",
           dashboard.last_data[0], dashboard.last_data[1],
           dashboard.last_data[2], dashboard.last_data[3],
           dashboard.last_data[4], dashboard.last_data[5],
           dashboard.last_data[6], dashboard.last_data[7]);
  lv_label_set_text(data_value, buf);

  uint32_t age = dashboard.last_update_ms == 0 ? 0 : millis() - dashboard.last_update_ms;
  snprintf(buf, sizeof(buf), "Updated %lums ago", static_cast<unsigned long>(age));
  lv_label_set_text(age_value, buf);
}
