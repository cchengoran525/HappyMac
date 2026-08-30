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
// 睡前过渡链（一次性事件动画）：无人 2.5s 后 打哈欠→揉眼→渐暗→入睡，
// 期间有人回来随时打断。事件只做帧内表情覆盖，不锁状态。
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
#define ML_CONFIRM_MS 2000
#define V_NOISE_FLOOR 8.0f
#define PRESENCE_DEBOUNCE_MS 500
#define DEBUG_ANIM_LOG_MS 100
#define POSITION_ENTER_PX 170.0f
#define POSITION_EXIT_PX  80.0f
#define LOOK_MAX_PX       30.0f
#define LOOK_NORMAL_PX    24.0f
#define LOOK_NORMAL_INPUT 700.0f
#define LOOK_EXTREME_INPUT 1400.0f
#define BLINK_DURATION_MS 180
#define EYE_WIDTH_PX      8
#define EYE_MAX_HEIGHT_PX 14
#define LOOK_DEADZONE_MM  80.0f
#define LOOK_TARGET_ALPHA 0.08f
#define LOOK_MOVE_ALPHA   0.06f
#define LOOK_MAX_STEP_PX  0.55f
#define ATTENTION_DRIFT_PX 1.5f
#define Y_TREND_SAMPLE_MS 140
#define Y_TREND_ALPHA     0.28f
#define Y_TREND_TRIGGER_MM 15.0f
#define Y_TREND_RELEASE_MM 6.0f   // 事件后趋势回落到此值以内才重新武装
#define TRANSIENT_HOLD_MS 900
#define TRANSIENT_COOLDOWN_MS 400
// 运动判定：近距静态噪声 σ≈100mm，单帧硬阈值必然误触发。改为逐帧证据
// 积分 + 进入/退出双阈值，进入要连续多帧证据，退出要持续安静。
#define MOTION_SPEED_TH   15.0f    // 滤波速度阈值，放在静态噪声带之上
#define MOTION_REF_MS     700      // 位移参考点的采样间隔
#define MOTION_DELTA_MM   130.0f   // 参考间隔内滤波位移超过此值算一份移动证据
#define MOTION_SCORE_MAX  8
#define MOTION_ENTER_SCORE 4       // 连续约 0.5s 的证据才进入“在动”
#define MOTION_EXIT_SCORE  1       // 持续安静后才退出“在动”
#define SMILE_ENTER_MS    600      // ACTIVE 持续这么久嘴才笑：短暂误进不闪嘴
#define SMILE_MIN_MS      1500     // 笑容最短保持时间，防止嘴型反复横跳
#define BEDTIME_YAWN_AT   2500     // 无人 2.5s 后开始睡前链（在 GOODBYE 目送之后）
#define BEDTIME_YAWN_MS   2000
#define BEDTIME_RUB_MS    1500
#define BEDTIME_DIM_MS    1200     // 渐暗结束转 SLEEP（合计 last_seen+7.2s，早于 8s 兜底）

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
float y_trend_sample = 0;
float y_step_filtered = 0;
uint32_t y_trend_sample_at = 0;
bool y_trend_ready = false;
bool y_trend_armed = true;
float flt_v = 0;
float motion_ref_x = 0, motion_ref_y = 0;
uint32_t motion_ref_at = 0;
int8_t motion_score = 0;
bool motion_stable = false;

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
float attention_offset = 0.0f;
float attention_target = 0.0f;
uint32_t attention_until = 0;
uint32_t next_attention_at = 7000;
uint8_t attention_sequence = 0;
int8_t nose_direction = -1;  // 只改变底部横线方向，竖线保持不变
uint32_t transient_until = 0;
uint32_t transient_cooldown_until = 0;
bool blink_active = false;
uint32_t blink_started = 0;
uint32_t next_blink_at = 5200;
uint8_t blink_sequence = 0;
uint32_t smile_until = 0;
uint32_t smile_pending_since = 0;

// 一次性事件动画：帧内覆盖表情，播完回姿态层，不锁状态。
enum OneShotEv { EV_NONE, EV_YAWN, EV_RUB, EV_DIM };
OneShotEv ev_cur = EV_NONE;
uint32_t ev_started = 0;
int mouth_mode = 0;   // 0 平嘴 1 微笑 2 哈欠
int eye_wiggle = 0;

const char* evName(OneShotEv e) {
  switch (e) {
    case EV_YAWN: return "YAWN";
    case EV_RUB:  return "RUB";
    case EV_DIM:  return "DIM";
    default:      return "-";
  }
}

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

// 每收到一帧有效雷达报告积分一次运动证据：单帧噪声只贡献 ±1，
// 真实移动会连续累积，跨过进入阈值后才切换运动状态。
void updateMotionEvidence() {
  uint32_t now = millis();
  if (motion_ref_at == 0) {
    motion_ref_x = flt_x;
    motion_ref_y = flt_y;
    motion_ref_at = now;
    return;
  }
  bool evidence = fabsf(flt_v) > MOTION_SPEED_TH ||
                  fabsf(flt_x - motion_ref_x) > MOTION_DELTA_MM ||
                  fabsf(flt_y - motion_ref_y) > MOTION_DELTA_MM;
  motion_score += evidence ? 1 : -1;
  if (motion_score < 0) motion_score = 0;
  if (motion_score > MOTION_SCORE_MAX) motion_score = MOTION_SCORE_MAX;
  if (!motion_stable && motion_score >= MOTION_ENTER_SCORE) {
    motion_stable = true;
  } else if (motion_stable && motion_score <= MOTION_EXIT_SCORE) {
    motion_stable = false;
  }
  if (now - motion_ref_at >= MOTION_REF_MS) {
    motion_ref_x = flt_x;
    motion_ref_y = flt_y;
    motion_ref_at = now;
  }
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
              flt_v = v;
              filter_ok = true;
            } else {
              prev_y = flt_y;
              flt_x = EMA_A * x + (1.0f - EMA_A) * flt_x;
              flt_y = EMA_A * y + (1.0f - EMA_A) * flt_y;
              flt_v = 0.35f * v + 0.65f * flt_v;
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
        updateMotionEvidence();
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
  // 雷达失联时让运动证据自然衰减，避免运动状态挂在过期数据上。
  if (!r50_ok && motion_score > 0) {
    motion_score--;
    if (motion_stable && motion_score <= MOTION_EXIT_SCORE) motion_stable = false;
  }
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
  // 已经是积分 + 滞回后的稳定判定，单帧噪声不再直接翻动状态。
  if (!r50_ok) return false;
  return motion_stable;
}

void updateYTrend(uint32_t now) {
  if (!r50_ok) {
    y_trend_ready = false;
    y_step_filtered = 0.0f;
    return;
  }
  if (!y_trend_ready) {
    y_trend_sample = flt_y;
    y_trend_sample_at = now;
    y_trend_ready = true;
    return;
  }
  if (now - y_trend_sample_at < Y_TREND_SAMPLE_MS) return;

  float step = flt_y - y_trend_sample;
  y_step_filtered += (step - y_step_filtered) * Y_TREND_ALPHA;
  y_trend_sample = flt_y;
  y_trend_sample_at = now;
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

  // 连续方向：正常移动覆盖最大幅度的约 80%，极端移动再软扩展到 OLED 边缘。
  float magnitude = fabsf(delta);
  float normal_part = constrain(magnitude / LOOK_NORMAL_INPUT, 0.0f, 1.0f);
  float extreme_part = constrain(
    (magnitude - LOOK_NORMAL_INPUT) /
      (LOOK_EXTREME_INPUT - LOOK_NORMAL_INPUT), 0.0f, 1.0f);
  float look_magnitude = normal_part * LOOK_NORMAL_PX +
                         extreme_part * (LOOK_MAX_PX - LOOK_NORMAL_PX);
  model_a_direction = (delta < 0.0f ? 1.0f : -1.0f) * look_magnitude;
  if (fabsf(delta) < LOOK_DEADZONE_MM) {
    model_a_direction = 0.0f;
  } else if (model_a_position == POS_CENTER && fabsf(delta) < POSITION_EXIT_PX) {
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
    // v 噪声底：静态/微动时雷达 v 在 ±8 内抖（含 8 的倍数毛刺）。减掉噪声底后，
    // typing/head 微动不再把 mean_abs_v 顶过树的 LATERAL 门限；
    // 真实横移仍由 slope/std_X 位移特征驱动，快动作远高于噪声底不受影响。
    float afv = fabsf((float)s.v);
    afv = afv > V_NOISE_FLOOR ? afv - V_NOISE_FLOOR : 0.0f;
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
  updateYTrend(now);
  bool present = stableTargetPresent();

  if (present) {
    ev_cur = EV_NONE;   // 人回来了：睡前链等一次性事件立刻打断
    last_seen = now;
    if (!had_target || state == HM_SLEEP || state == HM_GOODBYE) {
      had_target = true;
      changeState(HM_WAKING);
    }
  } else if (had_target && now - last_seen < ABSENT_MS) {
    if (state != HM_GOODBYE && state != HM_SLEEP) changeState(HM_GOODBYE);
    // 睡前过渡链：打哈欠→揉眼→渐暗→入睡；期间有人回来随时打断
    uint32_t since = now - last_seen;
    uint32_t tR = BEDTIME_YAWN_AT + BEDTIME_YAWN_MS,
             tD = tR + BEDTIME_RUB_MS, tEnd = tD + BEDTIME_DIM_MS;
    if (since >= BEDTIME_YAWN_AT) {
      OneShotEv want = since < tR ? EV_YAWN : since < tD ? EV_RUB : EV_DIM;
      if (ev_cur != want) {
        ev_cur = want; ev_started = now;
        if (Serial && Serial.availableForWrite() > 64) {
          Serial.printf("EV,%lu,%s\n", millis(), evName(ev_cur));
        }
      }
      if (since >= tEnd) { had_target = false; ev_cur = EV_NONE; changeState(HM_SLEEP); }
    }
  } else if (had_target && now - last_seen >= ABSENT_MS) {
    had_target = false;
    ev_cur = EV_NONE;
    changeState(HM_SLEEP);
  }

  if (!present) return;
  if (state == HM_WAKING && now - state_since < WAKING_MS) return;

  // 事件状态最小保持时间：保证 APPROACH/RETREAT 至少能被看见。
  if ((state == HM_APPROACH || state == HM_RETREAT) &&
      now < transient_until) {
    return;
  }

  // 先处理经过滤波的 Y 趋势，再让普通状态模型接管，避免 APPROACH
  // 刚触发就被 ACTIVE/IDLE 覆盖。趋势事件带武装滞回：触发后必须等
  // 趋势回落到噪声带以内才重新武装，否则阈值附近徘徊会连环触发。
  y_delta = y_step_filtered;
  if (!y_trend_armed && fabsf(y_step_filtered) < Y_TREND_RELEASE_MM) {
    y_trend_armed = true;
  }
  if (r50_ok && y_trend_armed && now >= transient_cooldown_until &&
      y_step_filtered < -Y_TREND_TRIGGER_MM) {
    changeState(HM_APPROACH);
    y_trend_armed = false;
    transient_until = now + TRANSIENT_HOLD_MS;
    transient_cooldown_until = transient_until + TRANSIENT_COOLDOWN_MS;
    return;
  }
  if (r50_ok && y_trend_armed && now >= transient_cooldown_until &&
      y_step_filtered > Y_TREND_TRIGGER_MM) {
    changeState(HM_RETREAT);
    y_trend_armed = false;
    transient_until = now + TRANSIENT_HOLD_MS;
    transient_cooldown_until = transient_until + TRANSIENT_COOLDOWN_MS;
    return;
  }

  // T1b v0 先接管较稳定的 ABSENT/STILL/LATERAL；左右位置仍由
  // LD2450 规则控制，APPROACH/RETREAT 留给 Y 趋势规则兜底。
  if (ml_stable <= 4) {
    if (ml_stable == 2) {
      changeState(HM_ACTIVE);
      return;
    }
    // ML 判 STILL 时若规则运动积分器正在报警，放行给规则路径：
    // 慢速大幅横移（LATERAL 漏检时）仍能被兜底成 ACTIVE。
    if (ml_stable == 1 && !targetMoving() && now - state_since > STATE_HOLD_MS) {
      changeState(HM_IDLE);
      return;
    }
  }

  if (targetMoving()) changeState(HM_ACTIVE);
  else if (now - state_since > STATE_HOLD_MS) changeState(HM_IDLE);
}

void updateAnimation() {
  updateModelA();

  // 保留鼻子的左右方向性，但只切换底部横线，不翻转竖线。
  if (model_a_position == POS_LEFT) nose_direction = -1;
  else if (model_a_position == POS_RIGHT) nose_direction = 1;

  // 启动后以当前目标位置为中心，缓慢吸收 LD2450 的长期漂移。
  if (center_ok && !targetMoving() && state == HM_IDLE) {
    center_x = center_x * 0.995f + flt_x * 0.005f;
  }

  uint32_t now = millis();
  float phase = (now % 2600) / 2600.0f * 6.283185f;
  face_breath_target = (sinf(phase) + 1.0f) * 0.5f;

  // 人不动时偶尔做很小的注意力漂移；有明显运动时完全关闭。
  bool calm = state == HM_IDLE && r50_ok && !targetMoving() &&
              fabsf(model_a_direction) < 3.0f;
  if (calm) {
    if (attention_target == 0.0f &&
        (int32_t)(now - next_attention_at) >= 0) {
      attention_target = (attention_sequence++ & 1) ?
        ATTENTION_DRIFT_PX : -ATTENTION_DRIFT_PX;
      attention_until = now + 900;
    } else if (attention_target != 0.0f && now >= attention_until) {
      attention_target = 0.0f;
      next_attention_at = now + 3600 + (attention_sequence % 5) * 500;
    }
  } else {
    attention_target = 0.0f;
    next_attention_at = now + 2200;
  }
  attention_offset += (attention_target - attention_offset) * 0.05f;

  // 先平滑目标，再以限速方式移动整张脸，避免“到点式”跳转。
  float desired_look = constrain(model_a_direction + attention_offset,
                                 -LOOK_MAX_PX, LOOK_MAX_PX);
  if (state == HM_SLEEP) desired_look = 0.0f;
  face_look_target += (desired_look - face_look_target) * LOOK_TARGET_ALPHA;
  float look_step = (face_look_target - face_look) * LOOK_MOVE_ALPHA;
  face_look += constrain(look_step, -LOOK_MAX_STEP_PX, LOOK_MAX_STEP_PX);
  face_breath += (face_breath_target - face_breath) * 0.08f;

  // 独立的短眨眼：不再用长时间取模条件造成“闭眼卡住”。
  if (state == HM_SLEEP || ev_cur != EV_NONE) {
    blink_active = false;
    next_blink_at = now + 1200;
  } else if (!blink_active && (int32_t)(now - next_blink_at) >= 0) {
    blink_active = true;
    blink_started = now;
  } else if (blink_active && now - blink_started >= BLINK_DURATION_MS) {
    blink_active = false;
    next_blink_at = now + 3800 + (blink_sequence++ % 5) * 550;
  }
}

void drawEye(int cx, int cy, int open, bool sleepy) {
  // 麦金塔经典笑脸：眼睛宽度固定，只改变高度，不画瞳孔。
  if (sleepy || open <= 1) {
    oled.drawBox(cx - EYE_WIDTH_PX / 2, cy, EYE_WIDTH_PX, 2);
    return;
  }
  int h = constrain(open, 2, EYE_MAX_HEIGHT_PX);
  oled.drawRBox(cx - EYE_WIDTH_PX / 2, cy - h / 2,
               EYE_WIDTH_PX, h, 1);
}

void drawMouth(int cx, int mode) {
  if (mode == 2) {
    // 打哈欠的大 O 嘴。
    oled.drawRBox(cx - 6, 44, 12, 11, 2);
    return;
  }
  if (mode != 1) {
    // 平嘴：厚度保持和微笑嘴主体接近。
    oled.drawBox(cx - 14, 51, 28, 3);
    return;
  }

  // 参考图的微笑嘴：整体要宽，两个小方角只是两端，不是主体。
  // 横带约 32 px，整体宽约 44 px，接近截图中的比例。
  oled.drawRBox(cx - 22, 46, 5, 5, 1);
  oled.drawRBox(cx + 17, 46, 5, 5, 1);
  oled.drawBox(cx - 16, 51, 32, 4);
  oled.drawLine(cx - 17, 50, cx - 16, 51);
  oled.drawLine(cx + 16, 51, cx + 17, 50);
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

  // 几何视差：整张脸共同平移，再改变五官相对位置，而不是只改变速度。
  float look_ratio = constrain(face_look / LOOK_MAX_PX, -1.0f, 1.0f);
  float eye_separation = 52.0f - 14.0f * fabsf(look_ratio);
  int left_eye_x = (int)(64.0f + face_look - eye_separation * 0.5f);
  int right_eye_x = (int)(64.0f + face_look + eye_separation * 0.5f);
  int nose_x = (int)(64.0f + face_look + 7.0f * look_ratio);
  int mouth_x = (int)(64.0f + face_look + 4.0f * look_ratio);
  int eye_open = 12;
  bool sleepy = false;

  switch (state) {
    case HM_SLEEP:
      sleepy = true;
      eye_open = 1;
      break;
    case HM_GOODBYE:
      eye_open = 9;
      break;
    case HM_WAKING: {
      float p = constrain((now - state_since) / (float)WAKING_MS, 0.0f, 1.0f);
      eye_open = (int)(3 + p * 10);
      break;
    }
    case HM_ACTIVE:
      eye_open = 13;
      break;
    case HM_APPROACH:
      eye_open = 14;
      break;
    case HM_RETREAT:
      eye_open = 7;
      break;
    case HM_IDLE:
      eye_open = 11;
      break;
  }

  if (blink_active && ev_cur == EV_NONE) {
    uint32_t elapsed = now - blink_started;
    float blink_scale;
    if (elapsed < 55) blink_scale = 1.0f - elapsed / 55.0f;
    else if (elapsed < 105) blink_scale = 0.0f;
    else blink_scale = (elapsed - 105) / 75.0f;
    eye_open = (int)(eye_open * constrain(blink_scale, 0.0f, 1.0f));
  }

  // 一次性事件动画：帧内覆盖表情，播完回姿态层
  int eye_y = 23;
  if (ev_cur == EV_YAWN) {
    float p = constrain((now - ev_started) / (float)BEDTIME_YAWN_MS, 0.0f, 1.0f);
    eye_open = p < 0.35f ? max(1, (int)(12 * (1 - p / 0.35f)))
             : p < 0.6f  ? 1
             : max(1, (int)(12 * (p - 0.6f) / 0.4f));
  } else if (ev_cur == EV_RUB) {
    eye_open = 2;
    eye_y = 23 + ((now / 150) & 1 ? 1 : -1);   // 揉眼的上下蹭动
  } else if (ev_cur == EV_DIM) {
    eye_open = 1;
  }
  drawEye(left_eye_x, eye_y, eye_open, sleepy);
  drawEye(right_eye_x, eye_y, eye_open, sleepy);

  // 固定的 J 形鼻子，和整张脸一起平移。
  oled.drawBox(nose_x - 1, 26, 3, 13);
  if (nose_direction < 0) oled.drawBox(nose_x - 7, 37, 8, 3);
  else oled.drawBox(nose_x, 37, 8, 3);

  // 不是所有状态都笑：ACTIVE 使用参考图的微笑嘴，其余状态多数为平嘴。
  // 笑容有进入延迟（短暂误进 ACTIVE 不闪嘴）和最短保持时间（状态边界
  // 来回抖时嘴型停留足够久才收）；睡眠/目送/醒来属于表情转折，不保留笑容。
  if (state == HM_ACTIVE) {
    if (smile_pending_since == 0) smile_pending_since = now;
    mouth_mode = now - smile_pending_since >= SMILE_ENTER_MS ? 1 : 0;
    if (mouth_mode == 1) smile_until = now + SMILE_MIN_MS;
  } else {
    smile_pending_since = 0;
    // 保持期内的笑容继续显示；SLEEP/GOODBYE/WAKING 属表情转折，立即收起。
    mouth_mode = (state == HM_IDLE || state == HM_APPROACH || state == HM_RETREAT) &&
                 (int32_t)(now - smile_until) < 0 ? 1 : 0;
  }
  if (ev_cur == EV_YAWN) mouth_mode = 2;   // 打哈欠的大 O 嘴
  drawMouth(mouth_x, mouth_mode);

  // 睡眠呼吸：亮度保持很低，但不完全熄灭；渐暗事件期间做斜坡。
  int contrast = state == HM_SLEEP ? 35 + (int)(face_breath * 20) : 170;
  if (ev_cur == EV_DIM) {
    float p = constrain((now - ev_started) / (float)BEDTIME_DIM_MS, 0.0f, 1.0f);
    contrast = (int)(170 - p * (170 - 45));
  }
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
