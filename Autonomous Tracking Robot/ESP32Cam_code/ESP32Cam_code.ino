#include "esp_camera.h"

#define BLACK_RATIO_MIN      0.015
#define BLACK_RATIO_TOOCLOSE 0.65
#define BLACK_RATIO_SLOW     0.45
#define BLACK_RATIO_MED      0.20
#define CONFIRM_NEEDED       3
#define LOST_FRAMES_MAX      8
#define LOOP_DELAY_MS        50
#define DIR_CONFIRM_NEEDED   3

int  confirmCount = 0;
int  lostCount    = 0;
bool tracking     = false;

char lastSentDir = 'S';
char pendingDir  = 'S';
int  dirConfirm  = 0;

void setupCamera() {
  camera_config_t cfg;
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0       = 5;
  cfg.pin_d1       = 18;
  cfg.pin_d2       = 19;
  cfg.pin_d3       = 21;
  cfg.pin_d4       = 36;
  cfg.pin_d5       = 39;
  cfg.pin_d6       = 34;
  cfg.pin_d7       = 35;
  cfg.pin_xclk     = 0;
  cfg.pin_pclk     = 22;
  cfg.pin_vsync    = 25;
  cfg.pin_href     = 23;
  cfg.pin_sscb_sda = 26;
  cfg.pin_sscb_scl = 27;
  cfg.pin_pwdn     = 32;
  cfg.pin_reset    = -1;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_RGB565;
  cfg.frame_size   = FRAMESIZE_QQVGA;
  cfg.jpeg_quality = 12;
  cfg.fb_count     = 1;

  if (esp_camera_init(&cfg) != ESP_OK) {
    Serial.println("Camera init failed!");
    while (true) { delay(1000); }
  }

  sensor_t* s = esp_camera_sensor_get();
  s->set_whitebal(s,      1);
  s->set_awb_gain(s,      1);
  s->set_wb_mode(s,       0);
  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s,          0);
  s->set_gain_ctrl(s,     1);
  s->set_brightness(s,    0);  // brightness thodi kam
  s->set_saturation(s,    0);
  s->set_contrast(s,      2);  // contrast zyada — black better detect hoga

  Serial.print("Camera warmup");
  for (int i = 0; i < 20; i++) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    delay(150);
    Serial.print(".");
  }
  Serial.println(" done");
}

void sendCmd(char dir) {
  if (dir == lastSentDir) return;
  lastSentDir = dir;
  Serial2.write(dir);
  Serial.print("CMD: ");
  Serial.println(dir);
}

void stopCar() {
  sendCmd('S');
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-CAM Black Follower");
  setupCamera();
  Serial2.begin(9600, SERIAL_8N1, 13, 12);
  Serial.println("Ready");
}

void loop() {
  camera_fb_t* fb = esp_camera_fb_get();

  if (!fb) {
    stopCar();
    delay(LOOP_DELAY_MS);
    return;
  }
  if (fb->len < 1000) {
    esp_camera_fb_return(fb);
    stopCar();
    delay(LOOP_DELAY_MS);
    return;
  }

  int fw = (fb->width  > 0) ? fb->width  : 160;
  int fh = (fb->height > 0) ? fb->height : 120;

  int blackTotal = 0;
  int blackLeft  = 0;
  int blackCentr = 0;
  int blackRight = 0;
  int totalPixels = 0;
  int pixelIndex  = 0;

  int rowStart = fh * 0.05;
  int rowEnd   = fh * 0.95;

  for (int i = 0; i < (int)fb->len - 1; i += 2) {
    int row = pixelIndex / fw;
    if (row >= rowStart && row <= rowEnd) {
      uint16_t pixel = ((uint16_t)fb->buf[i] << 8) | fb->buf[i + 1];
      uint8_t r = (pixel >> 11) & 0x1F;
      uint8_t g = (pixel >> 5)  & 0x3F;
      uint8_t b = (pixel)       & 0x1F;

      // BLACK detection — teenon channels low
      bool isBlack = (r < 6) &&
                     (g < 6) &&
                     (b < 6);

      if (isBlack) {
        blackTotal++;
        int col = pixelIndex % fw;
        if      (col < fw * 0.40)  blackLeft++;
        else if (col >= fw * 0.60) blackRight++;
        else                       blackCentr++;
      }
      totalPixels++;
    }
    pixelIndex++;
  }

  esp_camera_fb_return(fb);

  float ratio = (totalPixels > 0) ? (float)blackTotal / totalPixels : 0.0f;

  Serial.print("ratio="); Serial.print(ratio, 4);
  Serial.print(" L=");    Serial.print(blackLeft);
  Serial.print(" C=");    Serial.print(blackCentr);
  Serial.print(" R=");    Serial.println(blackRight);

  if (ratio >= BLACK_RATIO_MIN) {
    lostCount = 0;
    confirmCount++;
    if (confirmCount >= CONFIRM_NEEDED) tracking = true;

    if (tracking) {
      if (ratio >= BLACK_RATIO_TOOCLOSE) {
        stopCar();
        Serial.println("TOO CLOSE");
      } else {
        char newDir;
        if      (blackLeft  > blackCentr && blackLeft  > blackRight) newDir = 'L';
        else if (blackRight > blackCentr && blackRight > blackLeft)  newDir = 'R';
        else                                                          newDir = 'F';

        if (newDir == pendingDir) {
          dirConfirm++;
        } else {
          pendingDir = newDir;
          dirConfirm = 1;
        }

        if (dirConfirm >= DIR_CONFIRM_NEEDED) {
          sendCmd(pendingDir);
          dirConfirm = 0;
        }

        Serial.print("dir=");      Serial.print(pendingDir);
        Serial.print(" confirm="); Serial.println(dirConfirm);
      }
    } else {
      Serial.print("confirming ");
      Serial.print(confirmCount);
      Serial.print("/");
      Serial.println(CONFIRM_NEEDED);
      stopCar();
    }

  } else {
    confirmCount = 0;
    lostCount++;
    if (lostCount >= LOST_FRAMES_MAX) {
      tracking  = false;
      lostCount = LOST_FRAMES_MAX;
    }
    stopCar();
    Serial.println("no black");
  }

  delay(LOOP_DELAY_MS);
}