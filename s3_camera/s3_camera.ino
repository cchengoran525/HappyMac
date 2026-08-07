// ============================================================
//  HappyMac — s3_camera.ino
//  ESP32-S3 摄像头固件（Teacher 视觉源）
//
//  端点：
//    /           首页（设备信息 + 实时预览）
//    /stream     MJPEG 视频流（collect.py 主输入）
//    /capture    单张 JPEG（调试用，浏览器直接打开或 wget）
//    /status     JSON 状态（帧率/分辨率/WiFi 信号）
//    /control    参数调节（quality/framesize/brightness...）
//
//  使用：
//    1. 改下面的 WIFI_SSID / WIFI_PASS
//    2. 选板子（默认 ESP32-S3-EYE，其他板子改 BOARD 宏）
//    3. 烧录，打开串口看 IP，浏览器访问 http://<IP>
//
//  依赖：Arduino ESP32 Core + esp32-camera 库
// ============================================================

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

// ─── WiFi ──────────────────────────────────────────
const char* WIFI_SSID = "********";
const char* WIFI_PASS = "********";

// ─── 板子选择 ──────────────────────────────────────
// 取消注释你用的板子
#define BOARD_ESP32S3_EYE
// #define BOARD_XIAO_ESP32S3_SENSE
// #define BOARD_FREENOVE_ESP32S3_CAM

// ─── 摄像头参数 ────────────────────────────────────
#define CAM_FRAME_SIZE  FRAMESIZE_SVGA   // 800x600
#define CAM_JPEG_QUALITY 12              // 0-63, 越小画质越高
#define CAM_FB_COUNT     2               // 帧缓冲数

// ─── 服务端口 ──────────────────────────────────────
#define HTTP_PORT  80

// ╔══════════════════════════════════════════════════╗
// ║  板子引脚定义（一般不需要改）                       ║
// ╚══════════════════════════════════════════════════╝

#if defined(BOARD_ESP32S3_EYE)
  #define PWDN_GPIO_NUM  -1
  #define RESET_GPIO_NUM -1
  #define XCLK_GPIO_NUM  15
  #define SIOD_GPIO_NUM  4
  #define SIOC_GPIO_NUM  5
  #define Y9_GPIO_NUM    16
  #define Y8_GPIO_NUM    17
  #define Y7_GPIO_NUM    18
  #define Y6_GPIO_NUM    12
  #define Y5_GPIO_NUM    10
  #define Y4_GPIO_NUM    8
  #define Y3_GPIO_NUM    9
  #define Y2_GPIO_NUM    11
  #define VSYNC_GPIO_NUM 6
  #define HREF_GPIO_NUM  7
  #define PCLK_GPIO_NUM  13
  #define LED_GPIO_NUM   48   // 板载 LED（有些版本没有）

#elif defined(BOARD_XIAO_ESP32S3_SENSE)
  #define PWDN_GPIO_NUM  -1
  #define RESET_GPIO_NUM -1
  #define XCLK_GPIO_NUM  10
  #define SIOD_GPIO_NUM  40
  #define SIOC_GPIO_NUM  39
  #define Y9_GPIO_NUM    48
  #define Y8_GPIO_NUM    11
  #define Y7_GPIO_NUM    12
  #define Y6_GPIO_NUM    14
  #define Y5_GPIO_NUM    16
  #define Y4_GPIO_NUM    18
  #define Y3_GPIO_NUM    17
  #define Y2_GPIO_NUM    15
  #define VSYNC_GPIO_NUM 38
  #define HREF_GPIO_NUM  47
  #define PCLK_GPIO_NUM  13
  #define LED_GPIO_NUM   21

#elif defined(BOARD_FREENOVE_ESP32S3_CAM)
  #define PWDN_GPIO_NUM  -1
  #define RESET_GPIO_NUM -1
  #define XCLK_GPIO_NUM  15
  #define SIOD_GPIO_NUM  4
  #define SIOC_GPIO_NUM  5
  #define Y9_GPIO_NUM    16
  #define Y8_GPIO_NUM    17
  #define Y7_GPIO_NUM    18
  #define Y6_GPIO_NUM    12
  #define Y5_GPIO_NUM    10
  #define Y4_GPIO_NUM    8
  #define Y3_GPIO_NUM    9
  #define Y2_GPIO_NUM    11
  #define VSYNC_GPIO_NUM 6
  #define HREF_GPIO_NUM  7
  #define PCLK_GPIO_NUM  13
  #define LED_GPIO_NUM   -1   // 无板载 LED

#else
  #error "请选择一个板子（取消注释上面的 #define BOARD_xxx）"
#endif

// ╔══════════════════════════════════════════════════╗
// ║  HTTP 服务器 + MJPEG 推流                         ║
// ╚══════════════════════════════════════════════════╝

WebServer server(HTTP_PORT);

// 帧率统计
static unsigned long last_frame_ms = 0;
static float current_fps = 0;
static unsigned long frame_count = 0;

// ─── HTML 首页 ─────────────────────────────────────
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>HappyMac S3 Camera</title>
<style>
  body{font-family:system-ui;max-width:840px;margin:auto;padding:1em;background:#111;color:#eee}
  img{width:100%;max-width:800px;border-radius:8px}
  .info{display:flex;gap:2em;flex-wrap:wrap;margin:1em 0}
  .info span{background:#222;padding:.4em 1em;border-radius:6px;font-family:monospace}
  a{color:#4af}
</style>
</head>
<body>
<h1>📷 HappyMac S3 Camera</h1>
<div class="info">
  <span id="status">...</span>
  <span id="fps">-- fps</span>
</div>
<img id="stream" src="/stream" alt="MJPEG Stream">
<p>端点：<a href="/stream">/stream</a> · <a href="/capture">/capture</a> · <a href="/status">/status</a></p>
<script>
  fetch('/status').then(r=>r.json()).then(d=>{
    document.getElementById('status').textContent=d.resolution+' Q'+d.quality
  })
</script>
</body>
</html>
)rawliteral";

// ─── MJPEG 流 ──────────────────────────────────────
static const char* STREAM_BOUNDARY = "\r\n--frame\r\n";
static const char* STREAM_HEADER  = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

void handleStream() {
  WiFiClient client = server.client();
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println("Access-Control-Allow-Origin: *");
  client.println();

  unsigned long last = millis();
  while (client.connected()) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { delay(10); continue; }

    client.print(STREAM_BOUNDARY);
    client.printf(STREAM_HEADER, fb->len);
    client.write(fb->buf, fb->len);

    frame_count++;
    unsigned long now = millis();
    if (now - last >= 1000) {
      current_fps = frame_count * 1000.0f / (now - last);
      frame_count = 0;
      last = now;
    }

    esp_camera_fb_return(fb);
  }
}

// ─── 单张截图 ──────────────────────────────────────
void handleCapture() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

// ─── JSON 状态 ─────────────────────────────────────
void handleStatus() {
  sensor_t* s = esp_camera_sensor_get();
  char json[256];
  snprintf(json, sizeof(json),
    "{"
    "\"board\":\"%s\","
    "\"resolution\":\"%dx%d\","
    "\"quality\":%d,"
    "\"fps\":%.1f,"
    "\"wifi_rssi\":%d,"
    "\"free_heap\":%u,"
    "\"uptime_s\":%lu"
    "}",
    "ESP32-S3",
    s->status.framesize,
    s->status.framesize,
    s->status.quality,
    current_fps,
    WiFi.RSSI(),
    ESP.getFreeHeap(),
    millis() / 1000
  );
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

// ─── 参数调节 ──────────────────────────────────────
void handleControl() {
  if (!server.hasArg("var")) {
    server.send(400, "text/plain", "Missing 'var' parameter");
    return;
  }
  String var = server.arg("var");
  int val = server.hasArg("val") ? server.arg("val").toInt() : 0;

  sensor_t* s = esp_camera_sensor_get();

  if (var == "quality")      s->set_quality(s, val);
  else if (var == "framesize") {
    if (s->pixformat == PIXFORMAT_JPEG) s->set_framesize(s, (framesize_t)val);
  }
  else if (var == "contrast")    s->set_contrast(s, val);
  else if (var == "brightness")  s->set_brightness(s, val);
  else if (var == "saturation")  s->set_saturation(s, val);
  else if (var == "hmirror")     s->set_hmirror(s, val);
  else if (var == "vflip")       s->set_vflip(s, val);
  else if (var == "awb")         s->set_whitebal(s, val);
  else if (var == "aec")         s->set_exposure_ctrl(s, val);
  else if (var == "agc")         s->set_gain_ctrl(s, val);
  else {
    server.send(400, "text/plain", "Unknown parameter: " + var);
    return;
  }
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "OK: " + var + "=" + String(val));
}


// ╔══════════════════════════════════════════════════╗
// ║  setup / loop                                    ║
// ╚══════════════════════════════════════════════════╝

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n[HappyMac] S3 Camera booting...");

  // ── LED ──
  #if LED_GPIO_NUM >= 0
  pinMode(LED_GPIO_NUM, OUTPUT);
  digitalWrite(LED_GPIO_NUM, LOW);
  #endif

  // ── 摄像头 ──
  camera_config_t cfg;
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0  = Y2_GPIO_NUM;
  cfg.pin_d1  = Y3_GPIO_NUM;
  cfg.pin_d2  = Y4_GPIO_NUM;
  cfg.pin_d3  = Y5_GPIO_NUM;
  cfg.pin_d4  = Y6_GPIO_NUM;
  cfg.pin_d5  = Y7_GPIO_NUM;
  cfg.pin_d6  = Y8_GPIO_NUM;
  cfg.pin_d7  = Y9_GPIO_NUM;
  cfg.pin_xclk   = XCLK_GPIO_NUM;
  cfg.pin_pclk   = PCLK_GPIO_NUM;
  cfg.pin_vsync  = VSYNC_GPIO_NUM;
  cfg.pin_href   = HREF_GPIO_NUM;
  cfg.pin_sscb_sda = SIOD_GPIO_NUM;
  cfg.pin_sscb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn   = PWDN_GPIO_NUM;
  cfg.pin_reset  = RESET_GPIO_NUM;
  cfg.xclk_freq_hz  = 20000000;
  cfg.pixel_format   = PIXFORMAT_JPEG;
  cfg.frame_size     = CAM_FRAME_SIZE;
  cfg.jpeg_quality   = CAM_JPEG_QUALITY;
  cfg.fb_count       = CAM_FB_COUNT;
  cfg.grab_mode      = CAMERA_GRAB_WHEN_EMPTY;

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) {
    Serial.printf("[CAM] 初始化失败: 0x%x\n", err);
    #if LED_GPIO_NUM >= 0
    while (1) { digitalWrite(LED_GPIO_NUM, !digitalRead(LED_GPIO_NUM)); delay(200); }
    #else
    while (1) { Serial.println("HALT"); delay(1000); }
    #endif
  }
  Serial.println("[CAM] OK");

  // 关掉 LED 闪光灯（如果引脚被复用的话）
  sensor_t* s = esp_camera_sensor_get();
  s->set_vflip(s, 0);      // 不垂直翻转
  s->set_hmirror(s, 0);    // 不镜像（如果画面反了就改这个）

  // ── WiFi ──
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[WiFi] 连接 %s", WIFI_SSID);
  int wifi_retry = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_retry < 40) {
    delay(500);
    Serial.print(".");
    wifi_retry++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] 已连接 IP: %s\n", WiFi.localIP().toString().c_str());
    #if LED_GPIO_NUM >= 0
    digitalWrite(LED_GPIO_NUM, HIGH);
    #endif

    // mDNS（可选，浏览器输入 http://happymac.local）
    if (MDNS.begin("happymac")) {
      Serial.println("[mDNS] http://happymac.local");
    }
  } else {
    Serial.println("\n[WiFi] 连接失败，启动 AP 模式");
    WiFi.softAP("HappyMac-S3", "12345678");
    Serial.printf("[AP] IP: %s\n", WiFi.softAPIP().toString().c_str());
  }

  // ── HTTP 服务 ──
  server.on("/",        HTTP_GET, [](){ server.send_P(200, "text/html", INDEX_HTML); });
  server.on("/stream",  HTTP_GET, handleStream);
  server.on("/capture", HTTP_GET, handleCapture);
  server.on("/status",  HTTP_GET, handleStatus);
  server.on("/control", HTTP_GET, handleControl);

  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found. Try / /stream /capture /status /control");
  });

  server.begin();
  Serial.printf("[HTTP] 端口 %d 已启动\n", HTTP_PORT);
  Serial.printf("[INFO] 访问 http://%s/\n",
    WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : WiFi.softAPIP().toString().c_str());
  Serial.println("[INFO] ready");
}

void loop() {
  server.handleClient();
  delay(1);
}
