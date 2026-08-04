// ============================================================
//  HappyMac — new_radar.ino
//  双雷达+红外验证固件（当前阶段：仅 LD2450）
//
//  功能：
//    - LD2450 毫米波雷达 X/Y/速度 解析（协议正确解码）
//    - EMA 滤波（α=0.3，平滑慢漂）
//    - 左/中/右 分区检测（LEFT / CENTER / RIGHT）
//    - 趋势箭头（← → ·）
//    - OLED 实用显示 + 串口 CSV 采集输出
//
//  硬件：ESP32-C3 SuperMini + SH1106 OLED + LD2450 + SR602
//  烧录：CDCOnBoot=cdc（必须！否则 USB 无串口输出）
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <HardwareSerial.h>

// ─── 引脚定义 ──────────────────────────────────────
#define PIN_SDA       8
#define PIN_SCL       9
#define PIN_IR        5       // SR602 红外传感器
#define PIN_2450_RX   20
#define PIN_2450_TX   21
#define RADAR_BAUD    256000  // LD2450 目标数据输出波特率

// ─── 滤波参数 ──────────────────────────────────────
#define EMA_ALPHA     0.3f    // EMA 系数（越大响应越快，越小越平滑）
#define ZONE_LEFT      -200   // X < -200mm → LEFT
#define ZONE_RIGHT     +200   // X > +200mm → RIGHT（中间为 CENTER）

// ─── 硬件对象 ──────────────────────────────────────
U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
HardwareSerial radar2450(0);

// ─── 雷达数据 ──────────────────────────────────────
struct Target {
  int16_t x, y, v;
  bool    valid;
  float   ema_x, ema_y;
  bool    ema_ok;
};

Target tg[3];           // LD2450 最多 3 目标
int     best_slot = 0;  // 当前选中槽位
float   trend_ema = 0;  // ΔX 的 EMA（用于趋势箭头）
int     last_x = 0;

uint8_t buf50[64];
int     bufIdx = 0;
uint32_t last_frame_ms = 0;

// ─── 帧解析 ────────────────────────────────────────
void parse2450() {
  while (radar2450.available()) {
    buf50[bufIdx++] = radar2450.read();
    if (bufIdx >= 30) {
      if (buf50[0]==0xAA && buf50[1]==0xFF &&
          buf50[2]==0x03 && buf50[3]==0x00 &&
          buf50[28]==0x55 && buf50[29]==0xCC) {

        for (int t = 0; t < 3; t++) tg[t].valid = false;

        for (int t = 0; t < 3; t++) {
          int o = 4 + t * 8;
          int16_t rx = buf50[o]   | buf50[o+1]<<8;
          int16_t ry = buf50[o+2] | buf50[o+3]<<8;
          int16_t rv = buf50[o+4] | buf50[o+5]<<8;

          // HLK-LD2450 协议：bit15=1→正，bit15=0→负，低 15 位为幅值
          // X/Y 单位 mm，速度单位 cm/s
          int x = (rx & 0x8000) ?  (rx & 0x7FFF) : -(rx & 0x7FFF);
          int y = (ry & 0x8000) ?  (ry & 0x7FFF) : -(ry & 0x7FFF);
          int v = (rv & 0x8000) ?  (rv & 0x7FFF) : -(rv & 0x7FFF);

          if (!(x==0 && y==0)) {
            tg[t].x = x; tg[t].y = y; tg[t].v = v; tg[t].valid = true;
            if (!tg[t].ema_ok) {
              tg[t].ema_x = x; tg[t].ema_y = y; tg[t].ema_ok = true;
            } else {
              tg[t].ema_x = EMA_ALPHA * x + (1.0f-EMA_ALPHA) * tg[t].ema_x;
              tg[t].ema_y = EMA_ALPHA * y + (1.0f-EMA_ALPHA) * tg[t].ema_y;
            }
          }
        }

        // 选择最佳槽位：优先选 Y 在 0.2~3m 且最近有速度的
        best_slot = 0;
        for (int t = 0; t < 3; t++) {
          if (tg[t].valid && tg[t].y > 0 && tg[t].y < 3000) {
            best_slot = t; break;  // 选第一个合理距离的目标
          }
        }

        last_frame_ms = millis();
        bufIdx = 0;
      } else {
        memmove(buf50, buf50+1, --bufIdx);
      }
    }
  }
}

// ─── setup ──────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(PIN_IR, INPUT);
  Wire.begin(PIN_SDA, PIN_SCL);

  oled.begin();
  oled.setContrast(200);
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(25, 28, "HappyMac");
  oled.drawStr(10, 44, "booting...");
  oled.sendBuffer();

  radar2450.begin(RADAR_BAUD, SERIAL_8N1, PIN_2450_RX, PIN_2450_TX);
  delay(1000);
  while (radar2450.available()) radar2450.read();

  Serial.println("[HappyMac] ready");
  Serial.println("CSV: ms,x_raw,y_raw,x_ema,y_ema,spd,zone,trend,ir");
}

// ─── loop ───────────────────────────────────────────
void loop() {
  parse2450();

  // 帧超时：500ms 无数据 → 所有目标失效
  if (millis() - last_frame_ms > 500) {
    for (int t = 0; t < 3; t++) tg[t].valid = false;
  }

  bool irHi = (digitalRead(PIN_IR) == HIGH);

  // 取最佳槽位
  Target &t0 = tg[best_slot];
  bool ok = t0.valid;

  // ─── 左中右分区 ─────────────────────────────────
  const char* zone = "--";
  if (ok) {
    int xf = (int)t0.ema_x;
    if      (xf < ZONE_LEFT)  zone = " L";
    else if (xf > ZONE_RIGHT) zone = " R";
    else                       zone = " C";
  }

  // ─── 趋势箭头 ───────────────────────────────────
  const char* trend = ".";
  if (ok) {
    int dx = t0.x - last_x;
    last_x = t0.x;
    trend_ema = 0.3f * dx + 0.7f * trend_ema;
    if      (trend_ema >  30) trend = ">>";
    else if (trend_ema >  10) trend = " >";
    else if (trend_ema < -30) trend = "<<";
    else if (trend_ema < -10) trend = " <";
  }

  // ─── OLED 显示（5Hz）─────────────────────────────
  static uint32_t oled_timer = 0;
  if (millis() - oled_timer >= 200) {
    oled_timer = millis();
    oled.clearBuffer();

    if (ok) {
      // 第一行：分区 + 趋势 + IR
      oled.setFont(u8g2_font_6x10_tr);
      char line[32];
      snprintf(line, sizeof(line), "%s %s IR:%s",
        zone, trend, irHi ? "ON" : "--");
      oled.drawStr(0, 8, line);

      // 中间大字：滤波坐标
      oled.setFont(u8g2_font_7x13B_tr);
      snprintf(line, sizeof(line), "X%+4d", (int)t0.ema_x);
      oled.drawStr(0, 26, line);
      snprintf(line, sizeof(line), "Y%4d", (int)t0.ema_y);
      oled.drawStr(64, 26, line);

      // 速度
      oled.setFont(u8g2_font_6x10_tr);
      snprintf(line, sizeof(line), "V%+4d cm/s", t0.v);
      oled.drawStr(0, 37, line);

      // 俯视图（雷达在底部中央，X±1000mm, Y 0-2000mm）
      oled.drawHLine(0, 55, 128);
      oled.drawVLine(63, 42, 22);
      oled.drawDisc(63, 62, 2);
      int px = map(constrain((int)t0.ema_x, -1000, 1000), -1000, 1000, 0, 127);
      int py = map(constrain((int)t0.ema_y, 0, 2000), 0, 2000, 63, 44);
      oled.drawDisc(px, py, 2);
      // 分区线
      int lx = map(ZONE_LEFT,  -1000, 1000, 0, 127);
      int rx = map(ZONE_RIGHT, -1000, 1000, 0, 127);
      oled.drawVLine(lx, 56, 7);
      oled.drawVLine(rx, 56, 7);
    } else {
      oled.setFont(u8g2_font_7x13B_tr);
      oled.drawStr(10, 28, "no target");
      oled.setFont(u8g2_font_6x10_tr);
      oled.drawStr(10, 44, "waiting...");
    }

    oled.sendBuffer();
  }

  // ─── 串口 CSV ─────────────────────────────────────
  if (ok) {
    Serial.printf("2450,%lu,%d,%d,%d,%d,%d,%s,%s,%d\n",
      millis(),
      t0.x, t0.y,
      (int)t0.ema_x, (int)t0.ema_y,
      t0.v,
      zone, trend,
      irHi ? 1 : 0);
  }

  delay(100);
}
