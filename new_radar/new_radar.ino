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
#define EMA_A 0.55f   // 0.4→0.55, 响应~2帧(200ms)达63%

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

// ─── 串口命令（电脑 → C3 OLED 显示）─────────────────
char  cmd_phase[32] = "";
int   cmd_countdown  = 0;
int   cmd_state      = 0;  // 0=无, 1=倒计时, 2=开始, 3=完成
uint32_t cmd_expiry  = 0;

void checkCmd() {
  while (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (!s.startsWith("!")) continue;
    s = s.substring(1);
    if (s.startsWith("PHASE:")) {
      // !PHASE:静止基线,3
      int comma = s.indexOf(',');
      if (comma > 0) {
        String name = s.substring(6, comma);
        name.toCharArray(cmd_phase, sizeof(cmd_phase));
        cmd_countdown = s.substring(comma + 1).toInt();
        cmd_state = 1;
        cmd_expiry = millis() + 5000;
      }
    } else if (s == "GO") {
      cmd_state = 2; cmd_expiry = millis() + 3000;
    } else if (s == "DONE") {
      cmd_state = 3; cmd_expiry = millis() + 2000;
    } else if (s == "CLEAR") {
      cmd_state = 0; cmd_phase[0] = 0;
    }
  }
  if (cmd_state && millis() > cmd_expiry) {
    cmd_state = 0;
  }
}
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
  // ── 读电脑发来的 OLED 命令 ──
  checkCmd();

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

  // ── OLED（5Hz, 200ms）────────────────────────────
  static uint32_t ot=0;
  if(millis()-ot>=80){   // OLED 刷新 ~12Hz, 延迟 80ms
    ot=millis();
    oled.clearBuffer();
    char line[32];

    if(r50_ok){
      // 小字 raw 诊断
      oled.setFont(u8g2_font_5x8_tr);
      snprintf(line,sizeof(line),"raw X:%+4d Y:%4d V:%+3d", r50_x,r50_y,r50_v);
      oled.drawStr(0, 6, line);
      snprintf(line,sizeof(line),"Em:%3d Es:%3d D:%3d %s IR:%c",
        r10_em,r10_es,r10_dist, r10_pres?"P":"-", ir?'Y':'N');
      oled.drawStr(0, 13, line);

      // 大字 EMA
      oled.setFont(u8g2_font_7x13B_tr);
      snprintf(line,sizeof(line),"X%+4d",(int)ema_x); oled.drawStr(0,30,line);
      snprintf(line,sizeof(line),"Y%4d",(int)ema_y);  oled.drawStr(64,30,line);

      // 速度
      oled.setFont(u8g2_font_5x8_tr);
      int dx=r50_x-(int)ema_x, dy=r50_y-(int)ema_y;
      snprintf(line,sizeof(line),"V%+4d %s", r50_v,
        (abs(dx)>80||abs(dy)>60||abs(r50_v)>10)?"MOVING":"steady");
      oled.drawStr(0,40,line);

      // 阶段覆盖（电脑发指令）或俯视图
      if(cmd_state!=0){
        oled.setFont(u8g2_font_6x10_tr); oled.drawStr(0,50,cmd_phase);
        if(cmd_state==1){
          snprintf(line,sizeof(line),"%d",cmd_countdown);
          oled.setFont(u8g2_font_10x20_tn);
          oled.drawStr(64-strlen(line)*5,53,line);
        }else if(cmd_state==2){
          oled.setFont(u8g2_font_7x13B_tr); oled.drawStr(10,58,">>> GO <<<");
        }else if(cmd_state==3){
          oled.setFont(u8g2_font_7x13B_tr); oled.drawStr(20,58,"DONE");
        }
      }else{
        oled.drawHLine(0,53,128); oled.drawVLine(63,42,20); oled.drawDisc(63,62,2);
        int lx=map(-200,-800,800,0,127), rx=map(200,-800,800,0,127);
        oled.drawVLine(lx,56,6); oled.drawVLine(rx,56,6);
        int prx=map(constrain(r50_x,-800,800),-800,800,0,127);
        int pry=map(constrain(r50_y,0,2000),0,2000,63,44);
        oled.drawPixel(prx-1,pry); oled.drawPixel(prx+1,pry);
        oled.drawPixel(prx,pry-1); oled.drawPixel(prx,pry+1);
        int pex=map(constrain((int)ema_x,-800,800),-800,800,0,127);
        int pey=map(constrain((int)ema_y,0,2000),0,2000,63,44);
        oled.drawDisc(pex,pey,2);
      }
    }else{
      oled.setFont(u8g2_font_7x13B_tr); oled.drawStr(15,28,"no target");
    }
    oled.sendBuffer();
  }

  // ── 串口输出（非阻塞，缓冲区不足时自动跳过）──
  static uint8_t serial_skip=0;
  if(Serial && ++serial_skip>=2){  // 每 2 帧发一次，降低缓冲压力
    serial_skip=0;
    if(Serial.availableForWrite()>80){  // 确保有空间，避免阻塞
      Serial.printf("RADAR,%lu,%d,%d,%d,%d,%d,%d,%d,%d\n",
        millis(),
        r50_ok?r50_x:0, r50_ok?r50_y:0, r50_ok?r50_v:0,
        r10_em, r10_es, r10_dist,
        r10_pres?1:0, ir?1:0);
    }
  }
  delay(20);  // 30→20ms, 主循环 ~30Hz
}
