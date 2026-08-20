// ============================================================
// HappyMac v0 — 规则版产品固件
//
// 目的：先完成“装进外壳后能活起来”的初版，不依赖摄像头和 ML。
// 研究采集固件仍保留在 ../new_radar/new_radar.ino。
//
// 状态：
//   SLEEP    无目标一段时间，低亮度呼吸
//   GOODBYE  刚刚失去目标，短暂目送
//   WAKING   目标重新出现，慢慢睁眼
//   IDLE     有人但动作很小，正常待机
//   ACTIVE   有明显移动，眼睛睁大
//   APPROACH 目标距离变近，惊讶
//   RETREAT 目标距离变远，目送
//
// 左右跟随不是状态，而是连续的 faceLook 参数，避免表情跳变。
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <ld2410.h>
#include <HardwareSerial.h>

// ─── 引脚，与采集固件保持一致 ────────────────────────
#define PIN_SDA       8
#define PIN_SCL       9
#define PIN_IR        5
#define PIN_2410_RX   4
#define PIN_2410_TX   3
#define PIN_2450_RX   20
#define PIN_2450_TX   21
#define RADAR_BAUD    256000

#define EMA_A         0.55f
#define DISPLAY_MS    50       // 20Hz
#define ABSENT_MS     8000     // 8 秒无人后睡眠
#define GOODBYE_MS    1800
#define WAKING_MS     1800
#define STATE_HOLD_MS 450

U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
ld2410 radar2410;
HardwareSerial radar2450(0);

// ─── LD2450 ──────────────────────────────────────────
int16_t r50_x = 0, r50_y = 0, r50_v = 0;
bool r50_ok = false;
uint32_t r50_last = 0;
float flt_x = 0, flt_y = 0;
float prev_y = 0;
bool filter_ok = false;
float center_x = 0;
bool center_ok = false;

// ─── LD2410C / SR602 ─────────────────────────────────
int r10_em = 0, r10_es = 0;
bool r10_pres = false;
bool ir_present = false;

uint8_t frame_buf[64];
int frame_i = 0;

enum HappyState {
  HM_SLEEP,
  HM_GOODBYE,
  HM_WAKING,
  HM_IDLE,
  HM_ACTIVE,
  HM_APPROACH,
  HM_RETREAT
};

HappyState state = HM_SLEEP;
HappyState previous_state = HM_SLEEP;
uint32_t state_since = 0;
uint32_t last_seen = 0;
uint32_t last_display = 0;
uint32_t last_state_log = 0;
bool had_target = false;

// 动画参数：以屏幕中心为 0，范围约 ±6px。
float face_look = 0;
float face_look_target = 0;
float face_breath = 0;
float face_breath_target = 0;
float y_delta = 0;

const char* stateName(HappyState s) {
  switch (s) {
    case HM_SLEEP:    return "SLEEP";
    case HM_GOODBYE:  return "GOODBYE";
    case HM_WAKING:   return "WAKING";
    case HM_IDLE:     return "IDLE";
    case HM_ACTIVE:   return "ACTIVE";
    case HM_APPROACH: return "APPROACH";
    case HM_RETREAT:  return "RETREAT";
  }
  return "UNKNOWN";
}

void parse2450() {
  while (radar2450.available()) {
    frame_buf[frame_i++] = radar2450.read();
    if (frame_i >= 30) {
      bool valid = frame_buf[0] == 0xAA && frame_buf[1] == 0xFF &&
                   frame_buf[2] == 0x03 && frame_buf[3] == 0x00 &&
                   frame_buf[28] == 0x55 && frame_buf[29] == 0xCC;
      if (valid) {
        r50_ok = false;
        for (int t = 0; t < 3; t++) {
          int o = 4 + t * 8;
          int16_t rx = frame_buf[o] | frame_buf[o + 1] << 8;
          int16_t ry = frame_buf[o + 2] | frame_buf[o + 3] << 8;
          int16_t rv = frame_buf[o + 4] | frame_buf[o + 5] << 8;
          int x = (rx & 0x8000) ? (rx & 0x7FFF) : -(rx & 0x7FFF);
          int y = (ry & 0x8000) ? (ry & 0x7FFF) : -(ry & 0x7FFF);
          int v = (rv & 0x8000) ? (rv & 0x7FFF) : -(rv & 0x7FFF);
          if (!(x == 0 && y == 0)) {
            r50_x = x; r50_y = y; r50_v = v; r50_ok = true;
            if (!filter_ok) {
              flt_x = x; flt_y = y; prev_y = y;
              filter_ok = true;
            } else {
              prev_y = flt_y;
              flt_x = EMA_A * x + (1.0f - EMA_A) * flt_x;
              flt_y = EMA_A * y + (1.0f - EMA_A) * flt_y;
            }
            if (!center_ok) {
              center_x = flt_x;
              center_ok = true;
            }
            break;
          }
        }
        r50_last = millis();
        frame_i = 0;
      } else {
        memmove(frame_buf, frame_buf + 1, --frame_i);
      }
    }
  }
}

void readSensors() {
  radar2410.read();
  r10_pres = radar2410.movingTargetDetected() ||
             radar2410.stationaryTargetDetected();
  r10_em = radar2410.movingTargetEnergy();
  r10_es = radar2410.stationaryTargetEnergy();
  ir_present = digitalRead(PIN_IR) == HIGH;

  parse2450();
  if (r50_ok && millis() - r50_last > 500) r50_ok = false;
}

bool targetPresent() {
  return r50_ok || r10_pres || ir_present;
}

bool targetMoving() {
  return abs(r50_v) > 10 || abs(r50_x - (int)flt_x) > 80 ||
         abs(r50_y - (int)flt_y) > 60;
}

void changeState(HappyState next) {
  if (next == state) return;
  previous_state = state;
  state = next;
  state_since = millis();
  Serial.printf("STATE,%lu,%s\n", millis(), stateName(state));
}

void updateState() {
  uint32_t now = millis();
  bool present = targetPresent();

  if (present) {
    last_seen = now;
    if (!had_target || state == HM_SLEEP || state == HM_GOODBYE) {
      had_target = true;
      changeState(HM_WAKING);
    }
  } else if (had_target && now - last_seen < ABSENT_MS) {
    if (state != HM_GOODBYE && state != HM_SLEEP) changeState(HM_GOODBYE);
  } else if (had_target && now - last_seen >= ABSENT_MS) {
    had_target = false;
    changeState(HM_SLEEP);
  }

  if (!present) return;
  if (state == HM_WAKING && now - state_since < WAKING_MS) return;

  // LD2450 Y 越小通常代表越近。只使用窗口趋势，不信任单帧绝对值。
  y_delta = flt_y - prev_y;
  if (r50_ok && y_delta < -7 && targetMoving()) {
    changeState(HM_APPROACH);
    return;
  }
  if (r50_ok && y_delta > 7 && targetMoving()) {
    changeState(HM_RETREAT);
    return;
  }
  if (targetMoving()) changeState(HM_ACTIVE);
  else if (now - state_since > STATE_HOLD_MS) changeState(HM_IDLE);
}

void updateAnimation() {
  // 启动后以当前目标位置为中心，缓慢吸收 LD2450 的长期漂移。
  if (center_ok && !targetMoving() && state == HM_IDLE) {
    center_x = center_x * 0.995f + flt_x * 0.005f;
  }

  // 原始 X 方向与 OLED 镜像方向一致，限幅后只作为“注意力方向”。
  float x_rel = flt_x - center_x;
  face_look_target = constrain(-x_rel / 180.0f * 6.0f, -6.0f, 6.0f);
  if (state == HM_SLEEP) face_look_target = 0;

  uint32_t now = millis();
  float phase = (now % 2600) / 2600.0f * 6.283185f;
  face_breath_target = (sinf(phase) + 1.0f) * 0.5f;
  face_look += (face_look_target - face_look) * 0.15f;
  face_breath += (face_breath_target - face_breath) * 0.08f;
}

void drawEye(int cx, int cy, int open, int pupil_dx, bool sleepy) {
  if (sleepy || open <= 1) {
    oled.drawLine(cx - 7, cy, cx + 7, cy);
    return;
  }
  int h = constrain(open, 4, 11);
  oled.drawRBox(cx - 8, cy - h / 2, 16, h, 3);
  oled.setDrawColor(0);
  oled.drawDisc(cx + constrain(pupil_dx, -4, 4), cy, 3);
  oled.setDrawColor(1);
}

void drawFace() {
  uint32_t now = millis();
  oled.clearBuffer();

  int look = (int)face_look;
  int eye_open = 8;
  bool sleepy = false;
  bool surprised = false;
  bool happy = false;

  switch (state) {
    case HM_SLEEP:
      sleepy = true;
      eye_open = 1;
      break;
    case HM_GOODBYE:
      eye_open = 5;
      break;
    case HM_WAKING: {
      float p = constrain((now - state_since) / (float)WAKING_MS, 0.0f, 1.0f);
      eye_open = (int)(2 + p * 8);
      break;
    }
    case HM_ACTIVE:
      eye_open = 10;
      happy = true;
      break;
    case HM_APPROACH:
      eye_open = 11;
      surprised = true;
      break;
    case HM_RETREAT:
      eye_open = 5;
      break;
    case HM_IDLE:
      eye_open = 8;
      break;
  }

  int blink = ((now / 3700) % 17 == 0) ? 0 : eye_open;
  int eye_y = 28 + (int)((face_breath - 0.5f) * 2.0f);
  drawEye(38 + look, eye_y, blink, look * 0.35f, sleepy);
  drawEye(90 + look, eye_y, blink, look * 0.35f, sleepy);

  // 鼻子：前层移动最大。
  if (!sleepy) oled.drawDisc(64 + look, 38, surprised ? 2 : 1);

  // 嘴巴：不同状态使用简单像素表情。
  if (sleepy) {
    oled.drawLine(59, 47, 69, 47);
  } else if (surprised) {
    oled.drawCircle(64, 48, 4);
  } else if (happy) {
    oled.drawLine(57, 47, 61, 50);
    oled.drawLine(61, 50, 67, 50);
    oled.drawLine(67, 50, 71, 47);
  } else if (state == HM_RETREAT || state == HM_GOODBYE) {
    oled.drawLine(59, 50, 64, 48);
    oled.drawLine(64, 48, 69, 50);
  } else {
    oled.drawLine(60, 49, 68, 49);
  }

  // 睡眠呼吸：亮度保持很低，但不完全熄灭。
  int contrast = state == HM_SLEEP ? 35 + (int)(face_breath * 20) : 170;
  oled.setContrast(constrain(contrast, 20, 200));
  oled.sendBuffer();
}

void handleCommand() {
  while (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command == "!CAL") {
      center_ok = false;
      filter_ok = false;
      Serial.println("CAL,OK");
    } else if (command == "!STATE") {
      Serial.printf("STATE,%lu,%s\n", millis(), stateName(state));
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_IR, INPUT);
  Wire.begin(PIN_SDA, PIN_SCL);
  oled.begin();
  oled.setContrast(170);
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(22, 28, "HappyMac");
  oled.drawStr(16, 43, "v0 animation");
  oled.sendBuffer();

  Serial1.begin(RADAR_BAUD, SERIAL_8N1, PIN_2410_RX, PIN_2410_TX);
  {
    unsigned long until = millis() + 2000;
    while (millis() < until) while (Serial1.available()) Serial1.read();
  }
  if (radar2410.begin(Serial1)) {
    for (int g = 0; g < 4; g++) radar2410.setGateSensitivityThreshold(g, 80, 80);
    Serial.println("[2410] OK");
  } else {
    Serial.println("[2410] WARN");
  }

  radar2450.begin(RADAR_BAUD, SERIAL_8N1, PIN_2450_RX, PIN_2450_TX);
  Serial.println("[2450] OK");
  Serial.println("[HappyMac v0] rules-only animation ready");
  changeState(HM_SLEEP);
}

void loop() {
  handleCommand();
  readSensors();
  updateState();
  updateAnimation();

  uint32_t now = millis();
  if (now - last_display >= DISPLAY_MS) {
    last_display = now;
    drawFace();
  }

  // 保留轻量日志，方便初版装壳后的现场验证。
  static uint8_t radar_skip = 0;
  if (Serial && ++radar_skip >= 2) {
    radar_skip = 0;
    Serial.printf("RADAR,%lu,%d,%d,%d,%d,%d,%d,%d,%d\n",
      now, r50_ok ? r50_x : 0, r50_ok ? r50_y : 0,
      r50_ok ? r50_v : 0, r10_em, r10_es,
      r50_ok ? 1 : 0, targetPresent() ? 1 : 0, ir_present ? 1 : 0);
  }
  if (now - last_state_log >= 1000) {
    last_state_log = now;
    Serial.printf("ANIM,%lu,%s,look=%.1f,x=%.0f,y=%.0f\n",
      now, stateName(state), face_look, flt_x, flt_y);
  }
  delay(10);
}
