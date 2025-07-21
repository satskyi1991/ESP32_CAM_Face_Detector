#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"

#define WIFI_SSID "TP-Link_02C4"
#define WIFI_PASS "99134501"

ei::signal_t ei_signal;
ei_impulse_result_t result;

// === CAMERA CONFIG (ESP32-CAM OV2640) ===
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

bool setup_camera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_96X96;
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x", err);
    return false;
  }
  return true;
}

// === Edge Impulse Callback ===
float *features;
uint8_t *rgb888_full; // на стек не класти!
uint8_t *rgb888_resized;

static int get_signal_data(size_t offset, size_t length, float *out_ptr) {
  memcpy(out_ptr, features + offset, length * sizeof(float));
  return 0;
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

    Serial.println("Init...");

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  // config.pixel_format = PIXFORMAT_JPEG;
  // config.frame_size   = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_96X96;
  config.jpeg_quality = 12;                // 0-63 (низьке число = висока якість)
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
  } else {
    Serial.println("Camera init OK!");
  }

  // Serial.println("init camera");
  // if (!setup_camera()) {
  //   Serial.println("Camera setup failed");
  //   delay(1000);
  //   while (1);
  // }

  rgb888_full = (uint8_t*)heap_caps_malloc(96 * 96 * 3, MALLOC_CAP_SPIRAM);
  rgb888_resized = (uint8_t*)heap_caps_malloc(EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 3, MALLOC_CAP_SPIRAM);
  features = (float*)heap_caps_malloc(EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 3 * sizeof(float), MALLOC_CAP_SPIRAM);

  if (!rgb888_full || !rgb888_resized || !features) {
    Serial.println("PSRAM malloc failed");
    while (1);
  }
  //Serial.println("camera ok");

  //   Serial.begin(115200);
  // while (!Serial); // Дочекатись готовності UART (на деяких платах)
  // delay(2000);      // Дати час на запуск
  // Serial.println("BOOT OK");
}

void loop() {
  Serial.println("camera ok");

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  if (fb->width != 96 || fb->height != 96) {
    Serial.printf("Expected 96x96 image, got %dx%d\n", fb->width, fb->height);
    esp_camera_fb_return(fb);
    return;
  }

  // 3. Створюємо буфер RGB888 з RGB565
  for (size_t i = 0; i < EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT; i++) {
    uint16_t pixel = ((uint16_t *)fb->buf)[i];

    rgb888_resized[i * 3 + 0] = ((pixel >> 11) & 0x1F) << 3; // R
    rgb888_resized[i * 3 + 1] = ((pixel >> 5) & 0x3F) << 2;  // G
    rgb888_resized[i * 3 + 2] = (pixel & 0x1F) << 3;         // B
  }

  esp_camera_fb_return(fb); // Звільняємо кадр

  // 4. Подаємо RGB888 як features[] без нормалізації
  for (size_t i = 0; i < EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 3; i++) {
    //features[i] = static_cast<float>(rgb888_resized[i]);
    features[i] = static_cast<float>(rgb888_resized[i]) / 255.0f;
  }

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
  signal.get_data = [](size_t offset, size_t length, float *out_ptr) -> int {
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
  };

  ei_impulse_result_t result;
  EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
  if (res != EI_IMPULSE_OK) {
    Serial.printf("Classifier error: %d\n", res);
    return;
  }

  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    ei_printf("%s: %.2f%%\n", result.classification[ix].label, result.classification[ix].value * 100.f);
  }

  delay(5000);
}

