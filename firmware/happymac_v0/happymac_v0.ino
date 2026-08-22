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
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
#include "USBCDC.h"
#endif
#include <Wire.h>
#include <U8g2lib.h>
#include <ld2410.h>
#include <HardwareSerial.h>
#include "tinyml_v0_tree_d4.h"

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
#define DISPLAY_MS    80       // 约 12.5Hz；与采集固件一致，给 100kHz I²C 留余量
#define ABSENT_MS     8000     // 8 秒无人后睡眠
#define GOODBYE_MS    1800
#define WAKING_MS     1800
#define STATE_HOLD_MS 450
#define ML_WINDOW_MS  2000
#define ML_MIN_SAMPLES 20
#define ML_RING_CAP   64
#define ML_CONFIRM_MS 900
#define PRESENCE_DEBOUNCE_MS 500
#define DEBUG_ANIM_LOG_MS 100
#define POSITION_ENTER_PX 170.0f
#define POSITION_EXIT_PX  80.0f

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
uint32_t r50_seq = 0;

struct MlSample {
  uint32_t t;
  int16_t x;
  int16_t y;
  int16_t v;
  uint16_t es;
};

MlSample ml_ring[ML_RING_CAP];
uint8_t ml_head = 0;
uint8_t ml_count = 0;
uint32_t ml_seq_seen = 0;
uint8_t ml_candidate = 255;
uint8_t ml_stable = 255;
uint32_t ml_candidate_since = 0;
uint32_t last_ml_log = 0;

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

// Model A v0：位置模型只输出左右位置，不参与状态判断。
enum PositionClass {
  POS_CENTER,
  POS_LEFT,
  POS_RIGHT
};

HappyState state = HM_SLEEP;
HappyState previous_state = HM_SLEEP;
uint32_t state_since = 0;
uint32_t last_seen = 0;
uint32_t last_display = 0;
uint32_t last_state_log = 0;
bool had_target = false;
bool stable_presence = false;
uint32_t presence_candidate_since = 0;
bool debug_overlay = true;
int oled_contrast = -1;
uint32_t last_contrast_update = 0;
uint32_t last_diag_log = 0;
uint32_t last_loop_ms = 0;
uint32_t max_loop_gap_ms = 0;
uint32_t last_draw_ms = 0;
uint32_t max_draw_ms = 0;
uint32_t slow_draw_count = 0;

char command_buf[24];
uint8_t command_i = 0;

// 动画参数：以屏幕中心为 0，范围约 ±6px。
float face_look = 0;
float face_look_target = 0;
float face_breath = 0;
float face_breath_target = 0;
float y_delta = 0;
PositionClass model_a_position = POS_CENTER;
PositionClass model_a_previous = POS_CENTER;
float model_a_direction = 0.0f;

const char* positionClassName(PositionClass p) {
  switch (p) {
    case POS_LEFT:   return "LEFT";
    case POS_RIGHT:  return "RIGHT";
    default:         return "CENTER";
  }
}

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
        r50_seq++;
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

bool stableTargetPresent() {
  uint32_t now = millis();
  bool raw = targetPresent();
  if (raw == stable_presence) {
    presence_candidate_since = 0;
    return stable_presence;
  }
  if (presence_candidate_since == 0) presence_candidate_since = now;
  if (now - presence_candidate_since >= PRESENCE_DEBOUNCE_MS) {
    stable_presence = raw;
    presence_candidate_since = 0;
  }
  return stable_presence;
}

bool targetMoving() {
  return abs(r50_v) > 10 || abs(r50_x - (int)flt_x) > 80 ||
         abs(r50_y - (int)flt_y) > 60;
}

const char* positionLabel() {
  if (!r50_ok) return "NONE";
  return positionClassName(model_a_position);
}

void updateModelA() {
  if (!r50_ok || state == HM_SLEEP || !center_ok) {
    model_a_position = POS_CENTER;
    model_a_direction *= 0.90f;
    return;
  }

  float delta = flt_x - center_x;
  // 进入阈值和退出阈值分开，避免人在边界附近时左右抖动。
  if (model_a_position == POS_LEFT) {
    if (delta > -POSITION_EXIT_PX) model_a_position = POS_CENTER;
  } else if (model_a_position == POS_RIGHT) {
    if (delta < POSITION_EXIT_PX) model_a_position = POS_CENTER;
  } else {
    if (delta < -POSITION_ENTER_PX) model_a_position = POS_LEFT;
    else if (delta > POSITION_ENTER_PX) model_a_position = POS_RIGHT;
  }

  // 连续方向仍使用原始相对位置，分类只负责可解释的 LEFT/CENTER/RIGHT。
  model_a_direction = constrain(-delta / 180.0f * 6.0f, -6.0f, 6.0f);
  if (model_a_position == POS_CENTER && fabsf(delta) < POSITION_EXIT_PX) {
    model_a_direction *= fabsf(delta) / POSITION_EXIT_PX;
  }

  if (model_a_position != model_a_previous) {
    model_a_previous = model_a_position;
    if (Serial && Serial.availableForWrite() > 64) {
      Serial.printf("POS,%lu,%s,delta=%.0f,look=%.1f\n",
        millis(), positionClassName(model_a_position), delta, model_a_direction);
    }
  }
}

void mlPushSample() {
  MlSample &s = ml_ring[ml_head];
  s.t = r50_last;
  s.x = r50_x;
  s.y = r50_y;
  s.v = r50_v;
  s.es = (uint16_t)constrain(r10_es, 0, 65535);
  ml_head = (ml_head + 1) % ML_RING_CAP;
  if (ml_count < ML_RING_CAP) ml_count++;
}

void updateTinyML() {
  uint32_t now = millis();
  if (!r50_ok) {
    ml_candidate = 255;
    ml_stable = 255;
    return;
  }
  if (r50_seq == ml_seq_seen) return;
  ml_seq_seen = r50_seq;
  mlPushSample();

  uint8_t first = 0;
  while (first < ml_count) {
    uint8_t idx = (ml_head + ML_RING_CAP - ml_count + first) % ML_RING_CAP;
    if (now - ml_ring[idx].t <= ML_WINDOW_MS) break;
    first++;
  }
  int n = ml_count - first;
  if (n < ML_MIN_SAMPLES) return;

  float sum_k = 0, sum_k2 = 0, sum_x = 0, sum_y = 0;
  float sum_es = 0, sum_es2 = 0, sum_abs_v = 0, max_abs_v = 0;
  int es_edge = 0;
  int prev_es = -1;
  for (int j = 0; j < n; j++) {
    uint8_t idx = (ml_head + ML_RING_CAP - ml_count + first + j) % ML_RING_CAP;
    const MlSample &s = ml_ring[idx];
    float k = (float)j;
    float afv = fabsf((float)s.v);
    sum_k += k;
    sum_k2 += k * k;
    sum_x += s.x;
    sum_y += s.y;
    sum_es += s.es;
    sum_es2 += (float)s.es * s.es;
    sum_abs_v += afv;
    if (afv > max_abs_v) max_abs_v = afv;
    if (prev_es == 0 && s.es > 0) es_edge++;
    prev_es = s.es;
  }

  float denom = n * sum_k2 - sum_k * sum_k;
  if (denom <= 0.0f) return;
  float slope_x = (n * sum_x /* placeholder to keep expression readable */);
  float sum_kx = 0, sum_ky = 0;
  for (int j = 0; j < n; j++) {
    uint8_t idx = (ml_head + ML_RING_CAP - ml_count + first + j) % ML_RING_CAP;
    sum_kx += (float)j * ml_ring[idx].x;
    sum_ky += (float)j * ml_ring[idx].y;
  }
  slope_x = (n * sum_kx - sum_k * sum_x) / denom;
  float slope_y = (n * sum_ky - sum_k * sum_y) / denom;
  float mean_es = sum_es / n;
  float var_es = max(0.0f, sum_es2 / n - mean_es * mean_es);
  float features[9] = {
    fabsf(slope_x), slope_y,
    0.0f, 0.0f, mean_es, sqrtf(var_es),
    sum_abs_v / n, max_abs_v, (float)es_edge,
  };

  float mean_x = sum_x / n, mean_y = sum_y / n;
  float var_x = 0.0f, var_y = 0.0f;
  for (int j = 0; j < n; j++) {
    uint8_t idx = (ml_head + ML_RING_CAP - ml_count + first + j) % ML_RING_CAP;
    var_x += sq((float)ml_ring[idx].x - mean_x);
    var_y += sq((float)ml_ring[idx].y - mean_y);
  }
  features[2] = sqrtf(var_x / n);
  features[3] = sqrtf(var_y / n);

  uint8_t predicted = tinymlV0Predict(features);
  if (predicted != ml_candidate) {
    ml_candidate = predicted;
    ml_candidate_since = now;
  } else if (ml_stable != ml_candidate && now - ml_candidate_since >= ML_CONFIRM_MS) {
    ml_stable = ml_candidate;
    if (Serial && Serial.availableForWrite() > 100) {
      Serial.printf("ML,%lu,%s,meanEs=%.1f,slopeY=%.1f\n",
        now, tinymlV0LabelName(ml_stable), features[4], features[1]);
    }
  }

  if (now - last_ml_log >= 5000) {
    last_ml_log = now;
    if (Serial && Serial.availableForWrite() > 100) {
      Serial.printf("MLDBG,%lu,%s,candidate=%s,n=%d\n", now,
        tinymlV0LabelName(ml_stable), tinymlV0LabelName(ml_candidate), n);
    }
  }
}

void changeState(HappyState next) {
  if (next == state) return;
  previous_state = state;
  state = next;
  state_since = millis();
  if (Serial && Serial.availableForWrite() > 64) {
    Serial.printf("STATE,%lu,%s\n", millis(), stateName(state));
  }
}

void updateState() {
  uint32_t now = millis();
  bool present = stableTargetPresent();

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

  // T1b v0 先接管较稳定的 ABSENT/STILL/LATERAL；左右位置仍由
  // LD2450 规则控制，APPROACH/RETREAT 留给 Y 趋势规则兜底。
  if (ml_stable <= 4) {
    if (ml_stable == 2) {
      changeState(HM_ACTIVE);
      return;
    }
    if (ml_stable == 1 && now - state_since > STATE_HOLD_MS) {
      changeState(HM_IDLE);
      return;
    }
  }

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
  updateModelA();

  // 启动后以当前目标位置为中心，缓慢吸收 LD2450 的长期漂移。
  if (center_ok && !targetMoving() && state == HM_IDLE) {
    center_x = center_x * 0.995f + flt_x * 0.005f;
  }

  // 原始 X 方向与 OLED 镜像方向一致，限幅后只作为“注意力方向”。
  face_look_target = model_a_direction;
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

  if (debug_overlay) {
    char line[32];
    oled.setFont(u8g2_font_5x7_tr);
    snprintf(line, sizeof(line), "S:%s M:%s", stateName(state),
             tinymlV0LabelName(ml_stable));
    oled.drawStr(0, 7, line);
    snprintf(line, sizeof(line), "A:%s X:%d Y:%d", positionLabel(),
             r50_ok ? r50_x : 0, r50_ok ? r50_y : 0);
    oled.drawStr(0, 63, line);
    oled.setFont(u8g2_font_6x10_tr);
  }

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
  if (contrast != oled_contrast &&
      (now - last_contrast_update >= 100 || oled_contrast < 0)) {
    oled_contrast = contrast;
    last_contrast_update = now;
    oled.setContrast(contrast);
  }
  uint32_t draw_start = millis();
  oled.sendBuffer();
  last_draw_ms = millis() - draw_start;
  if (last_draw_ms > max_draw_ms) max_draw_ms = last_draw_ms;
  if (last_draw_ms > 120) slow_draw_count++;
}

void handleCommandLine(const char *command) {
  if (strcmp(command, "!CAL") == 0) {
    center_ok = false;
    filter_ok = false;
    if (Serial && Serial.availableForWrite() > 32) Serial.println("CAL,OK");
  } else if (strcmp(command, "!DEBUG") == 0) {
    debug_overlay = !debug_overlay;
    if (Serial && Serial.availableForWrite() > 32) {
      Serial.printf("DEBUG,%s\n", debug_overlay ? "ON" : "OFF");
    }
  } else if (strcmp(command, "!STATE") == 0) {
    if (Serial && Serial.availableForWrite() > 64) {
      Serial.printf("STATE,%lu,%s\n", millis(), stateName(state));
    }
  }
}

void handleCommand() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      command_buf[command_i] = '\0';
      if (command_i > 0) handleCommandLine(command_buf);
      command_i = 0;
    } else if (command_i < sizeof(command_buf) - 1) {
      command_buf[command_i++] = c;
    } else {
      // 丢弃过长命令，避免下一次半条输入污染命令解析。
      command_i = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_IR, INPUT);
  Wire.begin(PIN_SDA, PIN_SCL);
  // 与采集固件保持一致：GPIO8 同时挂着板载蓝灯和 OLED SDA，
  // 高速 I²C 容易造成偶发总线卡住；低速加超时优先保证主循环不死锁。
  Wire.setClock(100000);
  Wire.setTimeOut(20);
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
  Serial.println("[HappyMac v0] T1b tree_d4 animation ready");
  changeState(HM_SLEEP);
}

void loop() {
  uint32_t loop_start = millis();
  if (last_loop_ms != 0) {
    uint32_t gap = loop_start - last_loop_ms;
    if (gap > max_loop_gap_ms) max_loop_gap_ms = gap;
  }
  last_loop_ms = loop_start;

  handleCommand();
  readSensors();
  updateTinyML();
  updateState();
  updateAnimation();

  uint32_t now = millis();
  if (now - last_display >= DISPLAY_MS) {
    last_display = now;
    drawFace();
  }

  // 保留轻量日志，方便初版装壳后的现场验证。
  static uint8_t radar_skip = 0;
  if (Serial && ++radar_skip >= 2 && Serial.availableForWrite() > 120) {
    radar_skip = 0;
    Serial.printf("RADAR,%lu,%d,%d,%d,%d,%d,%d,%d,%d\n",
      now, r50_ok ? r50_x : 0, r50_ok ? r50_y : 0,
      r50_ok ? r50_v : 0, r10_em, r10_es,
      r50_ok ? 1 : 0, targetPresent() ? 1 : 0, ir_present ? 1 : 0);
  }
  uint32_t anim_log_ms = debug_overlay ? DEBUG_ANIM_LOG_MS : 1000;
  if (now - last_state_log >= anim_log_ms) {
    last_state_log = now;
    if (Serial && Serial.availableForWrite() > 100) {
      Serial.printf("ANIM,%lu,%s,look=%.1f,x=%.0f,y=%.0f\n",
        now, stateName(state), face_look, flt_x, flt_y);
    }
  }

  // 用低频诊断区分“主循环停了”和“某次 OLED/I²C 刷新变慢”。
  if (now - last_diag_log >= 2000) {
    last_diag_log = now;
    if (Serial && Serial.availableForWrite() > 120) {
      Serial.printf("DIAG,%lu,loopMax=%lu,drawLast=%lu,drawMax=%lu,drawSlow=%lu\n",
        now, max_loop_gap_ms, last_draw_ms, max_draw_ms, slow_draw_count);
    }
    max_loop_gap_ms = 0;
    max_draw_ms = 0;
    slow_draw_count = 0;
  }
  delay(10);
}
