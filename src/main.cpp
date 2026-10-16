#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>

// 板上 LED 引脚（开发板丝印 D2 = GPIO2）
#define LED_PIN 2

// 门开关状态检测引脚（ESP32 开发板 D15 = GPIO15）
#define DETECT_PIN 15

#define DEBOUNCE_TIME 50  //去抖时间长度

// ===== WiFi 配置（改成你自己的 WiFi 名称和密码）=====
const char *WIFI_SSID     = "你的WiFi名称";
const char *WIFI_PASSWORD = "你的WiFi密码";

// 寻找 WiFi 时每 0.5S 打印一个点
#define WIFI_DOT_INTERVAL 500

// ===== 服务器配置（门状态上报地址）=====
const char *SERVER_URL = "http://10.10.128.166:8080/door";

//门状态：0为关闭，1为打开
static bool door_status = 0;

// WiFi 连接状态
static bool wifiConnecting    = false;  // 是否正在寻找/连接中
static bool wifiWasConnected  = false;  // 上一次是否已连接（用于检测掉线）
static unsigned long lastWifiDotTime = 0;  // 上一次打印 "." 的时刻

// 在串口打印当前门状态
static void printDoorStatus() {
  Serial.print("当前门状态: ");
  Serial.println(door_status == 1 ? "打开" : "关闭");
}

// 通过 HTTP POST 把门开关状态发送到服务器
static void sendDoorStatus(bool open) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi 未连接，发送失败");
    return;
  }

  HTTPClient http;
  http.begin(SERVER_URL);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(3000);

  String body = String("{\"door\":\"") + (open ? "open" : "closed") +
                "\",\"door_status\":" + (open ? 1 : 0) + "}";

  int code = http.POST(body);
  if (code > 0) {
    Serial.print("已发送门状态: ");
    Serial.print(open ? "open" : "closed");
    Serial.print("，HTTP ");
    Serial.println(code);
  } else {
    Serial.print("发送门状态失败: ");
    Serial.println(http.errorToString(code));
  }

  http.end();
}

// 非阻塞地发起 WiFi 连接：未连接时每 0.5S 打印一个 "."，连上后打印提示
static void handleWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiWasConnected) {
      // 刚连接成功
      wifiWasConnected = true;
      wifiConnecting   = false;
      Serial.println();
      Serial.println("连接成功");
      Serial.print("IP 地址: ");
      Serial.println(WiFi.localIP());
      printDoorStatus();
    }
    return;
  }

  // 未连接：检测是否为掉线
  if (wifiWasConnected) {
    wifiWasConnected = false;
    Serial.println("WiFi 断开，重新寻找...");
  }

  // 第一次（或重新）发起连接
  if (!wifiConnecting) {
    wifiConnecting = true;
    Serial.println("寻找WIFI中...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    lastWifiDotTime = millis();
    return;
  }

  // 寻找过程中每 0.5S 打印一个点
  if (millis() - lastWifiDotTime >= WIFI_DOT_INTERVAL) {
    lastWifiDotTime = millis();
    Serial.println(".");
  }
}

void setup() {
  // LED 引脚配置为输出模式
  // 板载 LED 默认为低电平，LED 引脚初始化为高电平（LED 亮）
  pinMode(LED_PIN, OUTPUT);

  // D15 配置为输入模式（检测外部电平）
  //检测干簧管传感器数据，门关（磁铁靠近）为低电平，门开（磁铁远离离）为高电平
  pinMode(DETECT_PIN, INPUT);

  // 串口初始化，方便调试（波特率 115200）
  Serial.begin(115200);
  Serial.println("D15 level detect started (GPIO15)");
  Serial.println("HIGH -> LED OFF, LOW -> LED ON");

  // 上电后搜寻 WiFi 并连接（非阻塞，在 loop 里持续推进）
  handleWiFi();
}

void loop() {
  // 维护 WiFi 连接：寻找中每 0.5S 打印一个点，掉线自动重连
  handleWiFi();

  // 读取 D15 外部电平
  int level = digitalRead(DETECT_PIN);

  // 去抖：电平变化后需保持稳定超过 DEBOUNCE_TIME 才认为有效
  static int lastRawLevel = level;             // 上一次读到的原始电平
  static int stableLevel = level;              // 已确认的稳定电平
  static unsigned long lastChangeTime = 0;     // 最近一次电平变化的时刻

  if (level != lastRawLevel) {
    // 原始电平发生变化，重置计时
    lastRawLevel = level;
    lastChangeTime = millis();
  }

  // 已稳定且与当前有效状态不同，才执行状态切换
  if ((millis() - lastChangeTime >= DEBOUNCE_TIME) && (stableLevel != lastRawLevel)) {
    stableLevel = lastRawLevel;

    if (stableLevel == HIGH) {
      // 高电平：门开（磁铁远离）→ 灯灭
      door_status = 1;
      digitalWrite(LED_PIN, LOW);
      Serial.println("Door ON");
      printDoorStatus();
      // 检测到开门动作，通过 WiFi 上报
      sendDoorStatus(true);
    } else {
      // 低电平：门关（磁铁靠近）→ 灯亮
      door_status = 0;
      digitalWrite(LED_PIN, HIGH);
      Serial.println("Door OFF");
      printDoorStatus();
      // 检测到关门动作，通过 WiFi 上报
      sendDoorStatus(false);
    }
  }
  

}
