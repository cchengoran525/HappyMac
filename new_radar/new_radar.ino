// ============================================================
//  HappyMac — new_radar.ino (TinyML 数据采集版)
//
//  功能：双雷达同时采样 + 串口 CSV 输出
//    - LD2450: X/Y 坐标 + 速度（10Hz 目标数据）
//    - LD2410C: 移动/静止能量 + 距离 + 存在标志
//    - SR602: 红外二值信号
//
//  输出格式（115200 baud, USB CDC）：
//    RADAR,<ms>,<x>,<y>,<v>,<em>,<es>,<d2410>,<pres>,<ir>
//      ms    = 毫秒时间戳
//      x,y,v = LD2450 坐标(mm) + 速度(cm/s)
//      em,es = LD2410C moving/stationary energy
//      d2410 = LD2410C 距离(cm)
//      pres  = LD2410C presence (1/0)
//      ir    = SR602 (1/0)
//
//  硬件：ESP32-C3 SuperMini + SH1106 OLED + LD2450 + LD2410C + SR602
//  烧录：CDCOnBoot=cdc（必须！v3 core 默认 USB CDC 关闭）
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <ld2410.h>
#include <HardwareSerial.h>

// ─── 引脚 ──────────────────────────────────────────
#define PIN_SDA       8
#define PIN_SCL       9
#define PIN_IR        5       // SR602
#define PIN_2410_RX   4
#define PIN_2410_TX   3
#define PIN_2450_RX   20
#define PIN_2450_TX   21
#define RADAR_BAUD    256000

// ─── EMA ────────────────────────────────────────────
#define EMA_A 0.3f

// ─── 硬件 ──────────────────────────────────────────
U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
ld2410 radar2410;
HardwareSerial radar2450(0);

// ─── LD2450 ─────────────────────────────────────────
int16_t  r50_x=0, r50_y=0, r50_v=0;
bool     r50_ok=false;
float    ema_x=0, ema_y=0;
bool     ema_init=false;
uint32_t r50_last=0;

// ─── LD2410C ────────────────────────────────────────
int  r10_em=0, r10_es=0, r10_dist=0;
bool r10_pres=false;

// ─── LD2450 帧解析 ──────────────────────────────────
uint8_t b[64]; int bi=0;
void parse2450(){
  while(radar2450.available()){
    b[bi++]=radar2450.read();
    if(bi>=30){
      if(b[0]==0xAA&&b[1]==0xFF&&b[2]==0x03&&b[3]==0x00&&b[28]==0x55&&b[29]==0xCC){
        r50_ok=false;
        for(int t=0;t<3;t++){
          int o=4+t*8;
          int16_t rx=b[o]|b[o+1]<<8, ry=b[o+2]|b[o+3]<<8, rv=b[o+4]|b[o+5]<<8;
          int x=(rx&0x8000)?(rx&0x7FFF):-(rx&0x7FFF);
          int y=(ry&0x8000)?(ry&0x7FFF):-(ry&0x7FFF);
          int v=(rv&0x8000)?(rv&0x7FFF):-(rv&0x7FFF);
          if(!(x==0&&y==0)){
            r50_x=x; r50_y=y; r50_v=v; r50_ok=true;
            if(!ema_init){ema_x=x;ema_y=y;ema_init=true;}
            else{ema_x=EMA_A*x+(1-EMA_A)*ema_x; ema_y=EMA_A*y+(1-EMA_A)*ema_y;}
            break;
          }
        }
        r50_last=millis(); bi=0;
      }else{memmove(b,b+1,--bi);}
    }
  }
}

// ─── setup ──────────────────────────────────────────
void setup(){
  Serial.begin(115200);
  pinMode(PIN_IR,INPUT);
  Wire.begin(PIN_SDA,PIN_SCL);
  oled.begin(); oled.setContrast(200);
  oled.clearBuffer(); oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(15,20,"HappyMac"); oled.drawStr(0,36,"2-radar collect");
  oled.sendBuffer();

  // LD2410C
  Serial1.begin(RADAR_BAUD,SERIAL_8N1,PIN_2410_RX,PIN_2410_TX);
  {unsigned long t=millis()+2000; while(millis()<t) while(Serial1.available())Serial1.read();}
  if(radar2410.begin(Serial1)) Serial.println("[2410] OK");
  else Serial.println("[2410] WARN");

  // LD2450
  radar2450.begin(RADAR_BAUD,SERIAL_8N1,PIN_2450_RX,PIN_2450_TX);
  {unsigned long t=millis()+2000; while(millis()<t) while(radar2450.available())radar2450.read();}
  Serial.println("[2450] OK");

  Serial.println("CSV: ms,x,y,v,em,es,d2410,pres,ir");
}

// ─── loop ───────────────────────────────────────────
void loop(){
  // ── 读 LD2410C ──
  radar2410.read();
  r10_pres=radar2410.movingTargetDetected()||radar2410.stationaryTargetDetected();
  if(radar2410.movingTargetDetected()){
    r10_em=radar2410.movingTargetEnergy(); r10_es=0; r10_dist=radar2410.movingTargetDistance();
  }else if(radar2410.stationaryTargetDetected()){
    r10_em=0; r10_es=radar2410.stationaryTargetEnergy(); r10_dist=radar2410.stationaryTargetDistance();
  }else{r10_em=0;r10_es=0;r10_dist=0;}

  // ── 读 LD2450 ──
  parse2450();
  if(r50_ok&&(millis()-r50_last)>500)r50_ok=false;

  // ── IR ──
  bool ir=(digitalRead(PIN_IR)==HIGH);

  // ── OLED（4Hz）──
  static uint32_t ot=0;
  if(millis()-ot>=250){
    ot=millis();
    oled.clearBuffer(); oled.setFont(u8g2_font_6x10_tr); char line[32];

    if(r50_ok){
      snprintf(line,sizeof(line),"X%+4d Y%+4d V%+3d",(int)ema_x,(int)ema_y,r50_v);
      oled.drawStr(0,8,line);
      snprintf(line,sizeof(line),"Em%3d Es%3d D%3d",r10_em,r10_es,r10_dist);
      oled.drawStr(0,20,line);
      snprintf(line,sizeof(line),"pres:%c ir:%c",
        r10_pres?'Y':'N',ir?'Y':'N');
      oled.drawStr(0,32,line);
      oled.setFont(u8g2_font_7x13B_tr);
      snprintf(line,sizeof(line),"%s",r10_pres?"SOMEONE":"EMPTY");
      oled.drawStr(10,52,line);
    }else{
      oled.setFont(u8g2_font_7x13B_tr);
      oled.drawStr(15,30,"no target");
    }
    oled.sendBuffer();
  }

  // ── 串口 CSV ──
  Serial.printf("RADAR,%lu,%d,%d,%d,%d,%d,%d,%d,%d\n",
    millis(),
    r50_ok?r50_x:0, r50_ok?r50_y:0, r50_ok?r50_v:0,
    r10_em, r10_es, r10_dist,
    r10_pres?1:0, ir?1:0);

  delay(100);
}
