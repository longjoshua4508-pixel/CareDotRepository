/*
  Sketch for ESP32-S3 Touch LCD 1.85C BOX
  Trigger: On-screen button → record audio via I2S → HTTPS POST to webhook
  Libraries assumed installed:
    * GFX Library for Arduino
    * CST816S

  Flow overview:
    1. Boot, connect Wi-Fi, and init display/touch/I2S.
    2. Draw a button; when touched, capture 5 s of audio.
    3. Wrap the raw PCM in a WAV header and POST it to the webhook.
*/

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

#include <Arduino_GFX_Library.h>
#include <CST816S.h>
#include "driver/i2s.h"

// ---- Board-specific pin definitions (verify with your board revision) ----
// LCD reset is purposely kept off GPIO2 so it doesn’t conflict with the mic WS pin.
#define MIC_WS_PIN    2
#define MIC_SCK_PIN   15
#define MIC_SD_PIN    39

#define TP_INT_PIN    4
#define TP_SDA_PIN    11
#define TP_SCL_PIN    10
#define TP_RST_PIN    0

#define LCD_SDA0_PIN  46
#define LCD_SDA1_PIN  45
#define LCD_SDA2_PIN  42
#define LCD_SDA3_PIN  41
#define LCD_SCK_PIN   40
#define LCD_CS_PIN    21
#define LCD_RST_PIN   48
#define LCD_BL_PIN    5

// ---- User configuration ----
const char* ssid       = "YOUR_SSID";
const char* password   = "YOUR_PASSWORD";
const char* webhookURL = "https://YOUR_N8N_SERVER/webhook/YOUR_WEBHOOK_ID";

// Replace with your TLS root CA so HTTPS is validated properly.
const char rootCACert[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
...YOUR ROOT CA HERE...
-----END CERTIFICATE-----
)EOF";

// ---- Audio capture parameters ----
const int sampleRate       = 16000;
const int bitsPerSample    = 16;
const int audioChannels    = 1;
const int recordDurationMs = 5000;
const int bufferSamples    = (sampleRate * recordDurationMs) / 1000;  // 80,000 samples @ 5 s

int16_t *audioBuffer = nullptr;

Arduino_GFX *gfx = nullptr;
CST816S touch(TP_SDA_PIN, TP_SCL_PIN, TP_RST_PIN, TP_INT_PIN);
bool touchReady = false;

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
void initTouch();
bool initI2SMic();
bool recordAudio();
bool sendAudioToWebhook();
void buildWavHeader(uint8_t *buffer, uint32_t dataLength);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting up…");

  // Allocate the PCM buffer once up front to avoid heap fragmentation later.
  audioBuffer = static_cast<int16_t*>(malloc(bufferSamples * sizeof(int16_t)));
  if (!audioBuffer) {
    Serial.println("ERROR: Could not allocate audio buffer");
    while (true) { delay(1000); }
  }

  // Wi-Fi connect with a 20 s timeout so we don’t hang forever if credentials are wrong.
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ");
  const unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
    if (millis() - wifiStart > 20000) {
      Serial.println("\nERROR: WiFi connection timed out");
      while (true) { delay(1000); }
    }
  }
  Serial.println("\nWiFi connected! IP: " + WiFi.localIP().toString());

  initDisplay();
  drawButton();

  // Touch init is optional (board can still upload via serial commands even if touch fails).
  initTouch();

  if (!initI2SMic()) {
    Serial.println("ERROR: I2S init failed");
    while (true) { delay(1000); }
  }

  Serial.println("Setup complete. Waiting for button press…");
}

void loop() {
  checkTouch();
  if (buttonTouched) {
    Serial.println("Button pressed — capturing audio...");
    buttonTouched = false;

    // Only upload if the recording completed; otherwise keep the UI responsive.
    if (recordAudio()) {
      sendAudioToWebhook();
    } else {
      Serial.println("Recording failed, skipping upload.");
    }
    Serial.println("Done. Ready again.");
    drawButton();
  }
  delay(10);  // small idle delay to avoid busy-looping the touch chip
}

void initDisplay() {
  // Arduino_GFX handles the ST7796 panel; adjust DC if your revision uses a different pin.
  const int DC_PIN = 22;
  Arduino_DataBus *bus = new Arduino_SWSPI(DC_PIN, LCD_CS_PIN, LCD_SCK_PIN, LCD_SDA0_PIN, GFX_NOT_DEFINED);
  gfx = new Arduino_ST7796(bus, LCD_RST_PIN, 0, true, SCREEN_W, SCREEN_H);

  if (!gfx->begin()) {
    Serial.println("ERROR: Display begin failed!");
    while (true) { delay(1000); }
  }

  gfx->fillScreen(0x0000);
  gfx->setTextColor(0xFFFF);
  gfx->setTextSize(2);
  gfx->setCursor(10,10);
  gfx->println("Press button to record");

  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH);  // turn the backlight on
}

void drawButton() {
  gfx->fillRect(BUTTON_X, BUTTON_Y, BUTTON_W, BUTTON_H, 0x001F);
  gfx->setTextColor(0xFFFF);
  gfx->setTextSize(2);
  gfx->setCursor(BUTTON_X + 20, BUTTON_Y + (BUTTON_H/2 - 8));
  gfx->println("Start Recording");
}

void initTouch() {
  touch.begin();                // library call is void
  touchReady = true;            // flag so checkTouch() knows it was invoked
  Serial.println("Touch initialised (no HW ack available).");
}


void checkTouch() {
  if (!touchReady) {
    return;
  }
  if (touch.available()) {
    uint16_t x = touch.data.x;
    uint16_t y = touch.data.y;
    Serial.printf("Touch at x=%u, y=%u\n", x, y);
    if ((x >= BUTTON_X) && (x <= BUTTON_X + BUTTON_W) &&
        (y >= BUTTON_Y) && (y <= BUTTON_Y + BUTTON_H)) {
      buttonTouched = true;
    }
    delay(300);  // crude debounce to prevent repeated triggers
  }
}

bool initI2SMic() {
  Serial.println("Initializing I2S mic...");
  i2s_config_t i2s_config = {
    .mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = sampleRate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = 4,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num = MIC_SCK_PIN,
    .ws_io_num = MIC_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = MIC_SD_PIN
  };

  esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("ERROR: i2s_driver_install failed (%d)\n", err);
    return false;
  }

  err = i2s_set_pin(I2S_NUM_0, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("ERROR: i2s_set_pin failed (%d)\n", err);
    i2s_driver_uninstall(I2S_NUM_0);
    return false;
  }

  Serial.println("I2S mic ready.");
  return true;
}

bool recordAudio() {
  Serial.println("Recording audio buffer...");
  const size_t totalBytes = bufferSamples * sizeof(int16_t);
  size_t bytesRemaining = totalBytes;
  uint8_t *ptr = reinterpret_cast<uint8_t*>(audioBuffer);
  const size_t chunkSize = 1024;
  const TickType_t readTimeout = pdMS_TO_TICKS(200);
  uint32_t timeoutCount = 0;
  const uint32_t maxTimeouts = 10;

  // Read in small chunks with a timeout so a dead mic doesn’t block the UI forever.
  while (bytesRemaining > 0) {
    size_t bytesToRead = bytesRemaining > chunkSize ? chunkSize : bytesRemaining;
    size_t bytesRead = 0;
    esp_err_t err = i2s_read(I2S_NUM_0, ptr, bytesToRead, &bytesRead, readTimeout);
    if (err == ESP_OK && bytesRead > 0) {
      ptr += bytesRead;
      bytesRemaining -= bytesRead;
      timeoutCount = 0;
    } else if (err == ESP_ERR_TIMEOUT || bytesRead == 0) {
      timeoutCount++;
      Serial.println("WARN: I2S read timeout");
      if (timeoutCount >= maxTimeouts) {
        Serial.println("ERROR: Microphone not providing data");
        return false;
      }
    } else {
      Serial.printf("ERROR: i2s_read failed (%d)\n", err);
      return false;
    }
  }

  Serial.printf("Recorded %u bytes\n", static_cast<unsigned>(totalBytes));
  return true;
}

// Build a minimal PCM WAV header around the raw samples before uploading.
void buildWavHeader(uint8_t *buffer, uint32_t dataLength) {
  const uint32_t byteRate = sampleRate * audioChannels * (bitsPerSample / 8);
  const uint16_t blockAlign = audioChannels * (bitsPerSample / 8);

  memcpy(buffer + 0, "RIFF", 4);
  uint32_t chunkSize = 36 + dataLength;
  memcpy(buffer + 4, &chunkSize, 4);
  memcpy(buffer + 8, "WAVE", 4);
  memcpy(buffer + 12, "fmt ", 4);
  uint32_t subchunk1Size = 16;
  memcpy(buffer + 16, &subchunk1Size, 4);
  uint16_t audioFormat = 1;
  memcpy(buffer + 20, &audioFormat, 2);
  memcpy(buffer + 22, &audioChannels, 2);
  memcpy(buffer + 24, &sampleRate, 4);
  memcpy(buffer + 28, &byteRate, 4);
  memcpy(buffer + 32, &blockAlign, 2);
  memcpy(buffer + 34, &bitsPerSample, 2);
  memcpy(buffer + 36, "data", 4);
  memcpy(buffer + 40, &dataLength, 4);
}

bool sendAudioToWebhook() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("ERROR: WiFi not connected");
    return false;
  }

  // Allocate a temporary buffer for WAV header + PCM payload.
  const uint32_t payloadSize = bufferSamples * sizeof(int16_t);
  const uint32_t wavSize = 44 + payloadSize;
  uint8_t *wavBuffer = static_cast<uint8_t*>(malloc(wavSize));
  if (!wavBuffer) {
    Serial.println("ERROR: Failed to allocate WAV buffer");
    return false;
  }

  buildWavHeader(wavBuffer, payloadSize);
  memcpy(wavBuffer + 44, audioBuffer, payloadSize);

  WiFiClientSecure client;
  client.setCACert(rootCACert);
  HTTPClient https;

  if (!https.begin(client, webhookURL)) {
    Serial.println("ERROR: HTTPS begin failed");
    free(wavBuffer);
    return false;
  }

  https.addHeader("Content-Type", "audio/wav");
  https.addHeader("X-Device-ID", "ESP32S3-Touch-1.85C-BOX");

  int rc = https.sendRequest("POST", wavBuffer, wavSize);
  free(wavBuffer);

  if (rc > 0) {
    Serial.printf("HTTPS POST code: %d\n", rc);
    Serial.printf("Response: %s\n", https.getString().c_str());
  } else {
    Serial.printf("HTTPS POST failed, code: %d\n", rc);
  }
  https.end();
  return rc > 0;
}
