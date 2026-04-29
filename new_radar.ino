#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <ld2410.h>
#include <HardwareSerial.h>

// ─── 引脚定义 ───────────────────────────────────────
#define PIN_SDA       8
#define PIN_SCL       9
#define PIN_IR        5
#define PIN_2410_RX   4
#define PIN_2410_TX   3
#define PIN_2450_RX   20
#define PIN_2450_TX   21
#define RADAR_BAUD    256000

// ─── 硬件对象 ───────────────────────────────────────
U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
ld2410 radar2410;
HardwareSerial radar2450(0);  // UART0

// ─── LD2410C 数据 ────────────────────────────────────
bool  r10_present  = false;
int   r10_dist     = 0;
int   r10_energy_m = 0;
int   r10_energy_s = 0;

// ─── LD2450 数据 ─────────────────────────────────────
int16_t r50_x = 0, r50_y = 0, r50_speed = 0;
bool    r50_valid = false;

uint8_t buf50[64];
int     bufIdx = 0;

// ─── LD2450 帧解析 ────────────────────────────────────
void parse2450() {
  while (radar2450.available()) {
    buf50[bufIdx++] = radar2450.read();
    if (bufIdx >= 30) {
      if (buf50[0]==0xAA && buf50[1]==0xFF &&
          buf50[2]==0x03 && buf50[3]==0x00 &&
          buf50[28]==0x55 && buf50[29]==0xCC) {
        r50_valid = false;
        for (int t = 0; t < 3; t++) {
          int o = 4 + t * 8;
          int16_t raw_x = buf50[o]   | buf50[o+1]<<8;
          int16_t raw_y = buf50[o+2] | buf50[o+3]<<8;
          int16_t raw_v = buf50[o+4] | buf50[o+5]<<8;
          // 符号+数值解码
          int x = (raw_x & 0x8000) ? -(raw_x & 0x7FFF) : (raw_x & 0x7FFF);
          int y = (raw_y & 0x8000) ? -(raw_y & 0x7FFF) : (raw_y & 0x7FFF);
          int v = (raw_v & 0x8000) ? -(raw_v & 0x7FFF) : (raw_v & 0x7FFF);
          if (!(x==0 && y==0)) {
            r50_x = x; r50_y = y; r50_speed = v;
            r50_valid = true;
            break;
          }
        }
        bufIdx = 0;
      } else {
        memmove(buf50, buf50+1, --bufIdx);
      }
    }
  }
}
// ─── setup ───────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(PIN_IR, INPUT);

  // OLED
  Wire.begin(PIN_SDA, PIN_SCL);
  oled.begin();
  oled.setContrast(210);
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(20, 20, "HappyMac");
  oled.drawStr(10, 36, "initializing...");
  oled.sendBuffer();

  // LD2410C
  Serial1.begin(RADAR_BAUD, SERIAL_8N1, PIN_2410_RX, PIN_2410_TX);
  {
    unsigned long t = millis() + 2000;
    while (millis() < t) while (Serial1.available()) Serial1.read();
  }
  if (radar2410.begin(Serial1))
    Serial.println("[2410] OK");
  else
    Serial.println("[2410] WARN: not responding");

  // LD2450
  radar2450.begin(RADAR_BAUD, SERIAL_8N1, PIN_2450_RX, PIN_2450_TX);
  {
    unsigned long t = millis() + 2000;
    while (millis() < t) while (radar2450.available()) radar2450.read();
  }
  // 在 setup() 的 flush 之后加这段
// 发送"进入配置模式"指令
  uint8_t enableCmd[] = {0xFD,0xFC,0xFB,0xFA,0x04,0x00,0xFF,0x00,0x01,0x00,0x04,0x03,0x02,0x01};
  radar2450.write(enableCmd, sizeof(enableCmd));
  delay(100);

// 发送"恢复出厂设置"
  uint8_t resetCmd[] = {0xFD,0xFC,0xFB,0xFA,0x02,0x00,0xA2,0x00,0x04,0x03,0x02,0x01};
  radar2450.write(resetCmd, sizeof(resetCmd));
  delay(500);

// 退出配置模式
  uint8_t endCmd[] = {0xFD,0xFC,0xFB,0xFA,0x02,0x00,0xFE,0x00,0x04,0x03,0x02,0x01};
  radar2450.write(endCmd, sizeof(endCmd));
  delay(100);
  Serial.println("[2450] OK");

  Serial.println("[HappyMac] all components ready");
}

// ─── loop ────────────────────────────────────────────
void loop() {
  // 读 LD2410C
  radar2410.read();
  r10_present  = radar2410.movingTargetDetected() ||
                 radar2410.stationaryTargetDetected();
  if (radar2410.movingTargetDetected()) {
    r10_dist     = radar2410.movingTargetDistance();
    r10_energy_m = radar2410.movingTargetEnergy();
  } else if (radar2410.stationaryTargetDetected()) {
    r10_dist     = radar2410.stationaryTargetDistance();
    r10_energy_s = radar2410.stationaryTargetEnergy();
  } else {
    r10_dist = 0;
  }

  // 读 LD2450
  parse2450();

  // IR
  bool irHi = (digitalRead(PIN_IR) == HIGH);

  // ─── OLED 显示 ──────────────────────────────────
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);

  char line[32];

  // 行1: LD2410C 存在 + 距离
  if (r10_present)
    snprintf(line, sizeof(line), "2410: YES %dcm", r10_dist);
  else
    snprintf(line, sizeof(line), "2410: ---");
  oled.drawStr(0, 12, line);

  // 行2: LD2410C 能量
  snprintf(line, sizeof(line), "E: M=%d S=%d", r10_energy_m, r10_energy_s);
  oled.drawStr(0, 24, line);

  // 行3: LD2450 坐标
  if (r50_valid)
    snprintf(line, sizeof(line), "2450: X=%d Y=%d", r50_x, r50_y);
  else
    snprintf(line, sizeof(line), "2450: no target");
  oled.drawStr(0, 36, line);

  // 行4: LD2450 速度
  if (r50_valid)
    snprintf(line, sizeof(line), "Spd: %dmm/s", r50_speed);
  else
    snprintf(line, sizeof(line), "Spd: ---");
  oled.drawStr(0, 48, line);

  // 行5: IR 状态 + 分隔
  snprintf(line, sizeof(line), "IR: %s", irHi ? "HIGH" : "low");
  oled.drawStr(0, 60, line);

  // 右上角小点指示 LD2450 有无数据
  if (r50_valid) oled.drawDisc(124, 6, 3);
  else           oled.drawCircle(124, 6, 3);

  oled.sendBuffer();

  // 串口也输出一份方便调试
  Serial.printf("2410: %s %dcm | Em=%d Es=%d | 2450: X=%d Y=%d Spd=%d | IR=%s\n",
    r10_present ? "YES" : "NO",
    r10_dist,
    r10_energy_m, r10_energy_s,
    r50_x, r50_y, r50_speed,
    irHi ? "HIGH" : "low"
  );

  delay(100);
}