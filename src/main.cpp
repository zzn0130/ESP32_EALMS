#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>

// ================= 配置 =================
const char *WIFI_SSID     = "vivo X100 Pro";
const char *WIFI_PASSWORD = "66666666";

// 上位机（调试台）在同一热点下的 IP
const char *SERVER_HOST = "10.13.132.226";
const int   SERVER_PORT = 8080;
// =======================================

#define DEVICE_NO      "DM001"
#define DOOR_PATH      "/api/device/door/event"
#define HEARTBEAT_PATH "/api/device/heartbeat"

#define DETECT_PIN 15            // 门磁 DO：低=门关，高=门开
#define LED_PIN    2             // 板载 LED
#define LED_ON_LEVEL  HIGH       // 本板 HIGH=亮；若是低电平板改成 LOW
#define LED_OFF_LEVEL LOW

#define LED_WIFI_BLINK 100UL     // 未连 WiFi 快闪半周期(ms)：100ms → 约 5Hz
#define LED_TX_ON      60UL      // 发送提示：亮时长(ms)
#define LED_TX_GAP     60UL      // 发送提示：灭间隔(ms)
#define LED_TX_PULSES  2         // 发送提示闪几下

#define DEBOUNCE_TIME      200UL     // 去抖时间(ms)：门磁信号干净，200ms 足够
#define WIFI_DOT_INTERVAL  500UL     // 找网打点间隔(ms)
#define SERVER_TIMEOUT     1500      // 单次 HTTP 超时(ms)
#define RETRY_MAX          3         // 失败重发次数
#define RETRY_DELAY        1000UL    // 重发间隔(ms)
#define HEARTBEAT_INTERVAL 30000UL   // 心跳周期(ms)，30~60s
#define PENDING_MAX        4         // 待上报队列长度

// ---- 运行状态 ----
static bool door_open = false;               // 当前门状态
static bool wifiConnecting   = false;
static bool wifiWasConnected = false;
static unsigned long lastWifiDotTime   = 0;
static unsigned long lastHeartbeatTime = 0;
static String deviceMacSuffix = "0000";
static unsigned long eventSeq = 0;

// 事件唯一 ID：设备号-MAC后4位-millis-序号（重发时保持不变，服务端据此去重）
static String makeEventId() {
  return String(DEVICE_NO) + "-" + deviceMacSuffix + "-" +
         String(millis()) + "-" + String(eventSeq++);
}

// 读门磁电平：高=门开
static bool readDoorOpen() { return digitalRead(DETECT_PIN) == HIGH; }

// 同步门状态到变量（LED 已改由 updateLed() 统一管理，不再反映门状态）
static void applyDoorState(bool open) {
  door_open = open;
}

// ---- LED 指示 ----
// 未连 WiFi：一直快闪；已连 WiFi：常灭；每发送一次：快闪 2 下
static unsigned long ledNextAt   = 0;      // 下次动作时刻
static bool ledBlinkOn           = false;  // 快闪当前相位
static int  txPulsesLeft         = 0;      // 剩余待闪次数
static bool txPulseOn            = false;  // 发送提示当前是否处于“亮”

// 请求一次“发送提示”（快闪 LED_TX_PULSES 下）
static void triggerTxBlink() {
  txPulsesLeft = LED_TX_PULSES;
  txPulseOn    = false;
  ledNextAt    = millis();                 // 立即开始
}

// 由 loop() 驱动，统一决定 LED 亮灭
static void updateLed() {
  unsigned long now = millis();

  // 1) 发送提示优先：一亮一灭算闪一下
  if (txPulsesLeft > 0) {
    if ((long)(now - ledNextAt) < 0) return;
    txPulseOn = !txPulseOn;
    digitalWrite(LED_PIN, txPulseOn ? LED_ON_LEVEL : LED_OFF_LEVEL);
    if (!txPulseOn) txPulsesLeft--;        // 每次灭代表闪完一下
    ledNextAt = now + (txPulseOn ? LED_TX_ON : LED_TX_GAP);
    return;
  }

  // 2) 未连 WiFi：一直快闪
  if (!wifiWasConnected) {
    if ((long)(now - ledNextAt) < 0) return;
    ledBlinkOn = !ledBlinkOn;
    digitalWrite(LED_PIN, ledBlinkOn ? LED_ON_LEVEL : LED_OFF_LEVEL);
    ledNextAt = now + LED_WIFI_BLINK;
    return;
  }

  // 3) 已连 WiFi 且空闲：不亮
  digitalWrite(LED_PIN, LED_OFF_LEVEL);
}

// ---- 待上报队列：事件先入队，由 loop() 控制节奏发送，不用 delay() 阻塞 ----
struct Pending {
  bool   used;
  String path;
  String body;
  int    attempts;          // 已尝试次数
  unsigned long nextTryAt;  // 下次可尝试的时刻
};

static Pending pending[PENDING_MAX];

// 入队一条请求
static bool enqueueRequest(const char *path, const String &body) {
  for (int i = 0; i < PENDING_MAX; i++) {
    if (!pending[i].used) {
      pending[i].used      = true;
      pending[i].path      = path;
      pending[i].body      = body;
      pending[i].attempts  = 0;
      pending[i].nextTryAt = millis();      // 立即尝试
      return true;
    }
  }
  Serial.println("上报队列已满，丢弃该事件");
  return false;
}

// 单次 HTTP POST，只看 HTTP 层是否成功
static bool postOnce(const String &url, const String &body) {
  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(SERVER_TIMEOUT);

  int code = http.POST(body);
  http.end();

  return (code >= 200 && code < 300);
}

// 由 loop() 驱动：只处理到期的请求，发完立即返回
static void processPending() {
  if (WiFi.status() != WL_CONNECTED) return;

  for (int i = 0; i < PENDING_MAX; i++) {
    Pending &ev = pending[i];
    if (!ev.used) continue;
    if ((long)(millis() - ev.nextTryAt) < 0) continue;   // 还没到重发时刻

    String url = String("http://") + SERVER_HOST + ":" + SERVER_PORT + ev.path;
    ev.attempts++;

    triggerTxBlink();                    // 发送提示：LED 快闪两下

    if (postOnce(url, ev.body)) {
      Serial.printf("上报成功  %s\n", ev.body.c_str());
      ev.used = false;
    } else if (ev.attempts >= RETRY_MAX) {
      Serial.printf("上报失败(已试 %d 次)，放弃  %s\n", ev.attempts, ev.body.c_str());
      ev.used = false;
    } else {
      ev.nextTryAt = millis() + RETRY_DELAY;             // 1s 后再试，不阻塞
    }
  }
}

// 上报门事件
static void sendDoorEvent(bool open) {
  String body = String("{\"deviceNo\":\"") + DEVICE_NO +
                "\",\"eventType\":\"" + (open ? "OPEN" : "CLOSE") +
                "\",\"eventId\":\"" + makeEventId() + "\"}";
  enqueueRequest(DOOR_PATH, body);
}

// 上报心跳
static void sendHeartbeat() {
  enqueueRequest(HEARTBEAT_PATH, String("{\"deviceNo\":\"") + DEVICE_NO + "\"}");
}

// 非阻塞 WiFi 连接维护：寻找中每 0.5s 打点，掉线自动重连
static void handleWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiWasConnected) {
      wifiWasConnected = true;
      wifiConnecting   = false;
      Serial.printf("\n连接成功  IP=%s  服务器=%s:%d\n",
                    WiFi.localIP().toString().c_str(), SERVER_HOST, SERVER_PORT);

      // 上电（或重连）后补报一次当前门状态，让上位机对齐
      applyDoorState(readDoorOpen());
      Serial.printf("当前门状态: %s\n", door_open ? "打开" : "关闭");
      sendDoorEvent(door_open);
      lastHeartbeatTime = millis();
    }
    return;
  }

  if (wifiWasConnected) {
    wifiWasConnected = false;
    Serial.println("WiFi 断开，重新寻找...");
  }

  if (!wifiConnecting) {
    wifiConnecting = true;
    Serial.println("寻找WIFI中...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    lastWifiDotTime = millis();
    return;
  }

  if (millis() - lastWifiDotTime >= WIFI_DOT_INTERVAL) {
    lastWifiDotTime = millis();
    Serial.println(".");
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF_LEVEL); // 默认灭，之后交给 updateLed() 控制

  pinMode(DETECT_PIN, INPUT);           // 门磁 DO：门关(磁铁靠近)=低，门开=高

  Serial.begin(115200);
  Serial.printf("Door node %s started (DETECT_PIN=%d)\n", DEVICE_NO, DETECT_PIN);

  uint8_t mac[6];
  WiFi.macAddress(mac);
  char buf[5];
  snprintf(buf, sizeof(buf), "%02X%02X", mac[4], mac[5]);
  deviceMacSuffix = String(buf);

  handleWiFi();
}

void loop() {
  handleWiFi();

  // ---- 门状态：去抖 + 跳变沿检测 ----
  static int lastRawLevel = digitalRead(DETECT_PIN);
  static int stableLevel  = lastRawLevel;
  static unsigned long lastChangeTime = 0;

  int level = digitalRead(DETECT_PIN);

  if (level != lastRawLevel) {
    lastRawLevel = level;
    lastChangeTime = millis();
  }

  if ((millis() - lastChangeTime >= DEBOUNCE_TIME) && (stableLevel != lastRawLevel)) {
    stableLevel = lastRawLevel;
    applyDoorState(stableLevel == HIGH);
    Serial.printf("门事件: %s\n", door_open ? "OPEN" : "CLOSE");
    sendDoorEvent(door_open);
  }

  // ---- 周期心跳（非阻塞）----
  if (wifiWasConnected && (millis() - lastHeartbeatTime >= HEARTBEAT_INTERVAL)) {
    lastHeartbeatTime = millis();
    sendHeartbeat();
  }

  // ---- 驱动上报队列（非阻塞）----
  processPending();

  // ---- LED 指示 ----
  updateLed();
}
