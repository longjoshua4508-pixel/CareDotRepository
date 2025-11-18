/*
  Sketch for ESP32-S3 Touch LCD 1.85C BOX
  Trigger: On-screen button → record audio via I2S → HTTPS POST to webhook
  Libraries assumed installed:
    * GFX Library for Arduino
    * CST816S
*/

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Arduino_GFX_Library.h>    // for display/graphics
#include <CST816S.h>                // for touch controller

#include "driver/i2s.h"             // for I2S microphone input

// ---- Board-specific pin definitions (verify with your board revision) ----
#define MIC_WS_PIN    2     // I2S WS (word select)
#define MIC_SCK_PIN   15    // I2S SCK/BCK
#define MIC_SD_PIN    39    // I2S data in

#define TP_INT_PIN    4     // Touch controller interrupt
#define TP_SDA_PIN    11    // I2C SDA for touch
#define TP_SCL_PIN    10    // I2C SCL for touch
#define TP_RST_PIN    0     // (if applicable) reset pin for touch

#define LCD_SDA0_PIN  46
#define LCD_SDA1_PIN  45
#define LCD_SDA2_PIN  42
#define LCD_SDA3_PIN  41
#define LCD_SCK_PIN   40
#define LCD_CS_PIN    21
#define LCD_RST_PIN   2     // Reset pin for LCD
#define LCD_BL_PIN    5     // Back-light pin

// ---- User configuration ----
const char* ssid       = "YOUR_SSID";
const char* password   = "YOUR_PASSWORD";
const char* webhookURL = "https://YOUR_N8N_SERVER/webhook/YOUR_WEBHOOK_ID";

const int sampleRate       = 16000;    // Hz
const int bitsPerSample    = 16;
const int audioChannels    = 1;
const int recordDurationMs = 5000;     // record 5s
const int bufferSamples    = (sampleRate * recordDurationMs) / 1000;

int16_t *audioBuffer = nullptr;

Arduino_GFX *gfx = nullptr;
CST816S touch(TP_SDA_PIN, TP_SCL_PIN, TP_RST_PIN, TP_INT_PIN); // sda, scl, rst, irq

#define SCREEN_W 360
#define SCREEN_H 360
#define BUTTON_W 200
#define BUTTON_H 60
#define BUTTON_X ((SCREEN_W - BUTTON_W)/2)
#define BUTTON_Y (SCREEN_H - BUTTON_H - 20)

volatile bool buttonTouched = false;

// ---- Function prototypes ----
void initDisplay();
void drawButton();
void checkTouch();
void initI2SMic();
void recordAudio();
void sendAudioToWebhook();

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting up…");

  // Allocate buffer
  audioBuffer = (int16_t*) malloc(bufferSamples * sizeof(int16_t));
  if (!audioBuffer) {
    Serial.println("ERROR: Could not allocate audio buffer");
    while (true) { delay(1000); }
  }

  // Connect WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWiFi connected! IP: " + WiFi.localIP().toString());

  // Initialize display and touch
  initDisplay();
  drawButton();

  // Initialize I2S microphone
  initI2SMic();

  Serial.println("Setup complete. Waiting for button press…");
}

void loop() {
  // Check touch
  checkTouch();
  if (buttonTouched) {
    Serial.println("Button pressed — capturing audio...");
    buttonTouched = false;

    recordAudio();
    sendAudioToWebhook();

    Serial.println("Done. Ready again.");
    drawButton();
  }

  delay(10);
}

void initDisplay() {
  // Using Arduino_GFX library for display
  const int DC_PIN = 22; // Replace with correct DC if your board uses a different one
  Arduino_DataBus *bus = new Arduino_SWSPI(DC_PIN, LCD_CS_PIN, LCD_SCK_PIN, LCD_SDA0_PIN /* MOSI */, GFX_NOT_DEFINED /* MISO */);
  gfx = new Arduino_ST7796(bus, LCD_RST_PIN, 0 /* rotation */, true /* IPS */, SCREEN_W, SCREEN_H);

  if (!gfx->begin()) {
    Serial.println("ERROR: Display begin failed!");
    while (true) { delay(1000); }
  }

  gfx->fillScreen(0x0000);    // Black background
  gfx->setTextColor(0xFFFF);  // White text
  gfx->setTextSize(2);
  gfx->setCursor(10,10);
  gfx->println("Press button to record");

  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH); // Turn backlight on
}

void drawButton() {
  gfx->fillRect(BUTTON_X, BUTTON_Y, BUTTON_W, BUTTON_H, 0x001F); // Blue button
  gfx->setTextColor(0xFFFF); // White text
  gfx->setTextSize(2);
  gfx->setCursor(BUTTON_X + 20, BUTTON_Y + (BUTTON_H/2 - 8));
  gfx->println("Start Recording");
}

void checkTouch() {
  if (touch.available()) {
    uint16_t x = touch.data.x;
    uint16_t y = touch.data.y;
    Serial.printf("Touch at x=%u, y=%u\n", x, y);
    if ((x >= BUTTON_X) && (x <= BUTTON_X + BUTTON_W) &&
        (y >= BUTTON_Y) && (y <= BUTTON_Y + BUTTON_H)) {
      buttonTouched = true;
    }
    delay(300);  // simple debounce
  }
}


void initI2SMic() {
  Serial.println("Initializing I2S mic...");
  i2s_config_t i2s_config = {
    .mode             = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate      = sampleRate,
    .bits_per_sample  = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format   = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count    = 4,
    .dma_buf_len      = 512,
    .use_apll         = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk       = 0
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num   = MIC_SCK_PIN,
    .ws_io_num    = MIC_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = MIC_SD_PIN
  };
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  Serial.println("I2S mic ready.");
}

void recordAudio() {
  Serial.println("Recording audio buffer...");
  size_t bytesRead = 0;
  int totalBytes = bufferSamples * sizeof(int16_t);
  uint8_t *ptr = (uint8_t*)audioBuffer;
  int bytesToRead = totalBytes;
  while (bytesToRead > 0) {
    size_t done = 0;
    i2s_read(I2S_NUM_0, (void*)ptr, bytesToRead, &done, portMAX_DELAY);
    ptr += done;
    bytesToRead -= done;
  }
  Serial.printf("Recorded %d bytes\n", totalBytes);
}

void sendAudioToWebhook() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("ERROR: WiFi not connected");
    return;
  }

  WiFiClientSecure client;
  client.setInsecure();  // For testing only; validate certificate for production
  HTTPClient https;
  https.begin(client, webhookURL);
  https.addHeader("Content-Type", "application/octet-stream");
  https.addHeader("X-Device-ID", "ESP32S3-Touch-1.85C-BOX");

  int payloadSize = bufferSamples * sizeof(int16_t);
  int rc = https.sendRequest("POST", (uint8_t*)audioBuffer, payloadSize);
  if (rc > 0) {
    String resp = https.getString();
    Serial.printf("HTTPS POST code: %d\n", rc);
    Serial.printf("Response: %s\n", resp.c_str());
  } else {
    Serial.printf("HTTPS POST failed, code: %d\n", rc);
  }
  https.end();
}

