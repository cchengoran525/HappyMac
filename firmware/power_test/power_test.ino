// 临时测试固件：10s 开 / 10s 关 翻转雷达电源开关（GPIO1），验证 F5305S 模块。
// OLED 和串口同步显示当前状态。验证完请重新烧录 firmware/happymac_v0 正式固件。
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

#define PIN_PWR    1
#define PERIOD_MS  10000

U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

void show(const char *text) {
  Serial.println(text);
  oled.clearBuffer();
  oled.setFont(u8g2_font_10x20_tr);
  oled.drawStr(24, 36, text);
  oled.sendBuffer();
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PWR, OUTPUT);
  digitalWrite(PIN_PWR, HIGH);   // 开机上电
  Wire.begin(8, 9);
  oled.begin();
  show("PWR ON");
}

void loop() {
  delay(PERIOD_MS);
  digitalWrite(PIN_PWR, LOW);
  show("PWR OFF");
  delay(PERIOD_MS);
  digitalWrite(PIN_PWR, HIGH);
  show("PWR ON");
}
