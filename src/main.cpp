/**
 * ============================================================================
 *  ESP32 门磁检测节点 (DM001) —— 老人检测系统 / 门开合上报
 * ============================================================================
 *
 *  功能概览
 *    1. 上电后自动连接指定 WiFi（热点）；
 *    2. 通过干簧管（门磁）检测门的开/合，去抖后判定为一次门事件；
 *    3. 门事件与周期心跳通过 HTTP POST（JSON）上报到上位机；
 *    4. 上报失败自动重发（最多 3 次），非阻塞，不影响门检测实时性；
 *    5. 板载 LED 指示联网状态与发送动作。
 *
 *  硬件接线
 *    门磁 DO  -> GPIO15   模块输出：磁铁靠近/门关 = 低电平；磁铁远离/门开 = 高电平
 *    板载 LED -> GPIO2    本开发板 HIGH = 亮，LOW = 灭
 *
 *  网络拓扑
 *    ESP32 与上位机（PC）必须连在【同一个】热点或路由器下，
 *    否则双方的私有 IP（10.x / 192.168.x）互相不可达，上报必失败。
 *    手机热点由手机做 DHCP，重启后分配到的 IP 可能变化，
 *    SERVER_HOST 需要改成 PC 实际拿到的 IP。
 *
 *  上报协议
 *    门事件  POST /api/device/door/event
 *            {"deviceNo":"DM001","eventType":"OPEN|CLOSE","eventId":"<唯一ID>"}
 *    心跳    POST /api/device/heartbeat
 *            {"deviceNo":"DM001"}
 *    上位机成功时返回 HTTP 200，响应体形如：
 *            {"code": 0, "msg": "成功", "data": null}
 *    注意：本代码只判断 HTTP 状态码是否为 2xx，【不解析响应体】。
 *    因为服务端 JSON 里的空格、字段顺序一旦变化，字符串匹配就会误判为失败，
 *    进而触发无意义的重发（表现为上位机收到大量“重复帧”）。
 *
 *  设计要点
 *    - 全程非阻塞：不使用 delay()，靠 millis() 时间戳推进各状态机，
 *    因此门事件的端到端延迟 ≈ 去抖时间 + 一次 HTTP 往返（约 300ms）。
 *    - 单一 LED 仲裁：所有 LED 行为集中在 updateLed() 里按优先级裁决，
 *    避免多处 digitalWrite() 互相覆盖。
 *    - 事件去重：同一个事件在重发时 eventId 保持不变，上位机可据此识别重复帧。
 * ============================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>

// ============================ 用户配置区 ============================
// 下面这几项需要按现场环境修改
// ====================================================================

// ---- WiFi 凭据 ----
const char *WIFI_SSID     = "vivo X100 Pro";
const char *WIFI_PASSWORD = "66666666";

// ---- 上位机（调试台）地址 ----
// 必须是“上位机在 ESP32 所在热点下”的 IP，而不是任何公网/校园网地址。
// 若上位机就是本机 PC：用手机热点时，在 PC 上执行 ipconfig 查看 WLAN 的 IPv4 地址。
const char *SERVER_HOST = "10.13.132.226";
const int   SERVER_PORT = 8080;

// ========================== 设备标识与接口 ==========================

#define DEVICE_NO      "Door"                // 设备编号，随每条信息上报一起发送
#define DOOR_PATH      "/api/device/door/event"   // 门事件上报路径
#define HEARTBEAT_PATH "/api/device/heartbeat"    // 心跳上报路径

// ============================== 引脚定义 ==============================

#define DETECT_PIN 15            // 门磁 DO 输入引脚
#define LED_PIN    2             // 板载 LED 引脚

// LED 有效电平：本开发板 HIGH=亮。
// 若换成低电平点亮的板子，把下面两行的 HIGH/LOW 对调即可，其余代码无需改动。
#define LED_ON_LEVEL  HIGH
#define LED_OFF_LEVEL LOW

// ============================== 时间参数 ==============================
// 单位统一为 ms；加 UL 后缀是为了让运算全程保持无符号类型。

#define LED_WIFI_BLINK 100UL     // 未联网时的快闪半周期：100ms 翻转一次 → 约 5Hz
#define LED_TX_ON      60UL      // 发送信息提示：每次点亮的持续时长
#define LED_TX_GAP     60UL      // 发送信息提示：两次点亮之间的熄灭时长
#define LED_TX_PULSES  2         // 发送提示闪几下

#define DEBOUNCE_TIME      200UL     // 去抖：电平跳变后需持续稳定这么久才认定为有效事件
                                     // 越小延迟越低，越大越抗抖动；门磁信号干净，200ms 足够
#define WIFI_DOT_INTERVAL  500UL     // 连接 WiFi 期间，串口打 “.” 的间隔（表示连接中）
#define SERVER_TIMEOUT     1500      // 单次 HTTP 超时（含建连与读响应），失败时会阻塞这么久
#define RETRY_MAX          3         // 单条上报最多尝试次数（第 1 次 + 2 次重发）
#define RETRY_DELAY        1000UL    // 两次尝试之间的等待（非阻塞，仅推迟允许发送的时刻）
#define HEARTBEAT_INTERVAL 30000UL   // 心跳周期：上位机据此判断设备是否在线
#define PENDING_MAX        4         // 待上报队列容量

// ============================== 运行状态 ==============================
// 以下变量由各个状态机读写；用 static 限制在本编译单元内可见。

static bool door_open = false;               // 当前门状态：true=开，false=关

// ---- WiFi 状态机 ----
static bool wifiConnecting   = false;        // 是否已调用过 WiFi.begin()（避免重复发起）
static bool wifiWasConnected = false;        // 上一轮是否处于“已连接”，用于识别连上/掉线瞬间
static unsigned long lastWifiDotTime   = 0;  // 上一次打印 “.” 的时刻

// ---- 心跳 ----
static unsigned long lastHeartbeatTime = 0;  // 上一次发送心跳的时刻

// ---- 事件编号 ----
static String deviceMacSuffix = "0000";      // MAC 地址后 4 位（十六进制），用于区分多台设备
static unsigned long eventSeq = 0;           // 事件序号，单调递增，避免同一毫秒内 ID 重复

/**
 * 生成事件唯一 ID，格式：设备号-MAC后4位-millis-序号
 * 例如：DM001-B348-9210-1
 *
 * 为什么需要它：
 *   HTTP 上报失败会重发，上位机可能收到内容完全相同的两条报文。
 *   带上稳定的 eventId 后，上位机可以据此去重（识别“重复帧”）。
 *   因此同一个事件在多次重发时必须使用【同一个】ID —— 本函数只在
 *   事件产生的瞬间调用一次，重发时直接复用已入队的 body。
 */
static String makeEventId() {
  return String(DEVICE_NO) + "-" + deviceMacSuffix + "-" +
         String(millis()) + "-" + String(eventSeq++);
}

// 读取门磁当前电平：高电平 = 门打开（磁铁远离）
static bool readDoorOpen() { return digitalRead(DETECT_PIN) == HIGH; }

/**
 * 更新“当前门状态”变量。
 * 注意：LED 已不再反映门状态（改由 updateLed() 统一指示联网/发送），
 * 这里只做状态记录，供上电补报时使用。
 */
static void applyDoorState(bool open) {
  door_open = open;
}

// ============================== LED 指示 ==============================
// 优先级（从高到低）：
//   1) 发送提示   —— 每发起一次 HTTP 上报，快闪 LED_TX_PULSES 下
//   2) 未联网     —— 持续快闪，提示“还没连上 WiFi”
//   3) 已联网空闲 —— 常灭
//
// 因为板上只有一个 LED，多种指示必须由同一个函数仲裁，否则会互相覆盖。
// ======================================================================

static unsigned long ledNextAt = 0;      // 下一次需要翻转 LED 的时刻
static bool ledBlinkOn         = false;  // 联网快闪的当前相位（true=亮）
static int  txPulsesLeft       = 0;      // 发送提示还剩几下没闪
static bool txPulseOn          = false;  // 发送提示当前是否处于“亮”的半周期

/**
 * 请求一次“发送提示”：让 LED 快闪 LED_TX_PULSES 下。
 * 只负责登记请求（置计数、设起始时刻），实际的亮灭由 updateLed() 按时间片推进，
 * 所以本函数是瞬间返回的，不会阻塞调用者。
 */
static void triggerTxBlink() {
  txPulsesLeft = LED_TX_PULSES;
  txPulseOn    = false;
  ledNextAt    = millis();                 // 让更新逻辑立即开始动作
}

/**
 * LED 总调度：必须在 loop() 中每轮调用。
 * 非阻塞实现 —— 只判断“是否到点”，不做任何 delay。
 */
static void updateLed() {
  unsigned long now = millis();

  // 1) 发送提示优先级最高：亮 60ms → 灭 60ms 视为“闪一下”
  if (txPulsesLeft > 0) {
    if ((long)(now - ledNextAt) < 0) return;   // 用带符号差值判断，天然抗 millis() 溢出
    txPulseOn = !txPulseOn;
    digitalWrite(LED_PIN, txPulseOn ? LED_ON_LEVEL : LED_OFF_LEVEL);
    if (!txPulseOn) txPulsesLeft--;        // 每次“灭”代表完整闪完一下
    ledNextAt = now + (txPulseOn ? LED_TX_ON : LED_TX_GAP);
    return;
  }

  // 2) 未连接 WiFi：持续快闪，一半时间亮、一半时间灭
  if (!wifiWasConnected) {
    if ((long)(now - ledNextAt) < 0) return;
    ledBlinkOn = !ledBlinkOn;
    digitalWrite(LED_PIN, ledBlinkOn ? LED_ON_LEVEL : LED_OFF_LEVEL);
    ledNextAt = now + LED_WIFI_BLINK;
    return;
  }

  // 3) 已联网且没有待发送的提示：常灭
  digitalWrite(LED_PIN, LED_OFF_LEVEL);
}

// ============================== 上报队列 ==============================
// 为什么要用队列而不是检测到就立即发：
//   如果在 loop() 里直接发 HTTP，一旦网络慢或服务器无响应，
//   loop() 会被卡住最多 SERVER_TIMEOUT 毫秒，导致这段时间内门动作漏检。
//   改成“先入队、再由 loop() 按时间片发送”后，门检测始终能及时响应。
//
// 重发策略：
//   每次尝试失败后不 delay()，只把 nextTryAt 推迟 RETRY_DELAY 毫秒，
//   期间 loop() 照常运转；累计尝试 RETRY_MAX 次仍失败才放弃并丢弃。
// ======================================================================

struct Pending {
  bool   used;              // 该槽位是否被占用
  String path;              // 请求路径（DOOR_PATH / HEARTBEAT_PATH）
  String body;              // 已序列化好的 JSON 请求体（重发时原样重发，保证 eventId 不变）
  int    attempts;          // 已经尝试发送的次数
  unsigned long nextTryAt;  // 允许下一次尝试的时刻
};

static Pending pending[PENDING_MAX];   // 静态数组，全部初始化为 used=false

/**
 * 把一条请求放进待发队列。
 * @param path 请求路径
 * @param body JSON 请求体
 * @return true=入队成功；false=队列已满（该条上报被丢弃）
 */
static bool enqueueRequest(const char *path, const String &body) {
  for (int i = 0; i < PENDING_MAX; i++) {
    if (!pending[i].used) {
      pending[i].used      = true;
      pending[i].path      = path;
      pending[i].body      = body;
      pending[i].attempts  = 0;
      pending[i].nextTryAt = millis();      // 立即可发送
      return true;
    }
  }
  Serial.println("上报队列已满，丢弃该事件");
  return false;
}

/**
 * 执行一次 HTTP POST。
 * 只根据 HTTP 状态码判断成败（2xx 视为成功），不解析响应体 ——
 * 因为服务端返回的 JSON 可能带空格或字段顺序不同，做字符串匹配极易误判。
 * @return true=HTTP 层成功
 */
static bool postOnce(const String &url, const String &body) {
  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(SERVER_TIMEOUT);

  int code = http.POST(body);
  http.end();                          // 必须调用，否则底层连接不会释放

  return (code >= 200 && code < 300);
}

/**
 * 队列调度：由 loop() 每轮调用。
 * 只处理“已到期”的请求，处理完立即返回，绝不 delay。
 */
static void processPending() {
  if (WiFi.status() != WL_CONNECTED) return;   // 未联网时不消耗重试次数，等联网后再发

  for (int i = 0; i < PENDING_MAX; i++) {
    Pending &ev = pending[i];
    if (!ev.used) continue;
    if ((long)(millis() - ev.nextTryAt) < 0) continue;   // 还没到重发时刻，跳过

    String url = String("http://") + SERVER_HOST + ":" + SERVER_PORT + ev.path;
    ev.attempts++;

    triggerTxBlink();                    // 只要有实际发送动作，LED 就快闪两下

    if (postOnce(url, ev.body)) {
      Serial.printf("上报成功  %s\n", ev.body.c_str());
      ev.used = false;                                   // 成功，释放槽位
    } else if (ev.attempts >= RETRY_MAX) {
      Serial.printf("上报失败(已试 %d 次)，放弃  %s\n", ev.attempts, ev.body.c_str());
      ev.used = false;                                   // 重试用尽，丢弃
    } else {
      ev.nextTryAt = millis() + RETRY_DELAY;             // 稍后再试（非阻塞）
    }
  }
}

/**
 * 生成并提交一条门事件上报。
 * eventId 在这里生成一次即固定，重发时沿用，便于上位机去重。
 * @param open true=门打开(OPEN)，false=门关闭(CLOSE)
 */
static void sendDoorEvent(bool open) {
  String body = String("{\"deviceNo\":\"") + DEVICE_NO +
                "\",\"eventType\":\"" + (open ? "OPEN" : "CLOSE") +
                "\",\"eventId\":\"" + makeEventId() + "\"}";
  enqueueRequest(DOOR_PATH, body);
}

/**
 * 提交一条心跳上报。
 * 心跳不携带状态，只表示“设备还活着”。
 * 它的作用是让上位机能够区分下面两种“收不到数据”的情况：
 *   1) 设备已离线（断电/掉线）—— 心跳也会消失；
 *   2) 设备正常但门一直没人开 —— 心跳照常到来。
 */
static void sendHeartbeat() {
  enqueueRequest(HEARTBEAT_PATH, String("{\"deviceNo\":\"") + DEVICE_NO + "\"}");
}

// ============================ WiFi 连接维护 ============================
// 非阻塞状态机，每个 loop() 轮次推进一小步。四个分支互斥：
//   A. 已连接          -> 只在“刚连上”的那一轮做一次性初始化
//   B. 检测到掉线      -> 归位标志位，下面会自动重新发起连接
//   C. 尚未发起连接    -> 打印提示并 WiFi.begin()（只调一次）
//   D. 等待连接中      -> 每 WIFI_DOT_INTERVAL 打一个 “.”
// ========================================================================

static void handleWiFi() {
  // ---------- 分支 A：当前已连接 ----------
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiWasConnected) {
      // 仅在“刚刚连上”的这一轮进入，做一次性初始化
      wifiWasConnected = true;
      wifiConnecting   = false;
      Serial.printf("\n连接成功  IP=%s  服务器=%s:%d\n",
                    WiFi.localIP().toString().c_str(), SERVER_HOST, SERVER_PORT);

      // 上电（或断线重连）后补报一次当前门状态，
      // 让上位机立刻知道门此刻是开还是关，不必等到下一次动作。
      applyDoorState(readDoorOpen());
      Serial.printf("当前门状态: %s\n", door_open ? "打开" : "关闭");
      sendDoorEvent(door_open);
      lastHeartbeatTime = millis();       // 重置心跳计时，避免刚连上就补发心跳
    }
    return;
  }

  // ---------- 分支 B：检测到掉线 ----------
  if (wifiWasConnected) {
    wifiWasConnected = false;             // LED 会因此切回“快闪”状态
    Serial.println("WiFi 断开，重新寻找...");
  }

  // ---------- 分支 C：发起连接（只发一次） ----------
  if (!wifiConnecting) {
    wifiConnecting = true;
    Serial.println("寻找WIFI中...");
    WiFi.mode(WIFI_STA);                  // 工作模式：STA（去连热点），不是 AP 模式
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // 非阻塞，实际连接在后台完成
    lastWifiDotTime = millis();
    return;
  }

  // ---------- 分支 D：等待连接，定时打点 ----------
  if (millis() - lastWifiDotTime >= WIFI_DOT_INTERVAL) {
    lastWifiDotTime = millis();
    Serial.println(".");
  }
}

// ================================ 初始化 ================================
// 只在复位/上电后执行一次：配置引脚、启动串口、取 MAC 后四位、发起 WiFi 连接。
// 耗时操作都不放在这里，setup() 会尽快返回，让 loop() 开始跑。
// ========================================================================
void setup() {
  // LED 先熄灭，之后完全交给 updateLed() 控制
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF_LEVEL);

  // 门磁 DO 作为数字输入；模块内部已把磁簧管状态转换成逻辑电平
  pinMode(DETECT_PIN, INPUT);

  Serial.begin(115200);
  Serial.printf("Door node %s started (DETECT_PIN=%d)\n", DEVICE_NO, DETECT_PIN);

  // 取 MAC 的后两个字节拼成 4 位十六进制字符串，
  // 作为事件 ID 中的设备区分位（多台同型号设备共用同一台上位机时不会撞号）
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char buf[5];
  snprintf(buf, sizeof(buf), "%02X%02X", mac[4], mac[5]);
  deviceMacSuffix = String(buf);

  // 立刻启动 WiFi 连接流程（后续由 loop() 持续推进，不在这里死等）
  handleWiFi();
}

// ================================= 主循环 =================================
// 每一轮都很短（毫秒级），各个功能以“时间片”方式并行推进，互不阻塞。
// 注意：loop() 里【不允许出现 delay()】，否则会拖慢门检测与 LED 闪烁。
// =========================================================================
void loop() {
  // 1) 维护 WiFi：连接中打点、连上后补报、掉线自动重连
  handleWiFi();

  // 2) 门状态检测：去抖 + 跳变沿识别
  //    static 局部变量只在第一次进入时初始化，之后一直保留上一轮的值。
  static int lastRawLevel = digitalRead(DETECT_PIN);   // 上一次读到的原始电平
  static int stableLevel  = lastRawLevel;              // 已确认稳定的电平
  static unsigned long lastChangeTime = 0;             // 最近一次电平跳变的时刻

  int level = digitalRead(DETECT_PIN);

  // 电平一旦变化就重置计时器；
  // 接触抖动会让计时不断重来，只有持续稳定超过 DEBOUNCE_TIME 才会被采纳。
  if (level != lastRawLevel) {
    lastRawLevel = level;
    lastChangeTime = millis();
  }

  // 稳定时间已到，且与上次采纳的电平不同 → 判定为一次真实的门动作
  if ((millis() - lastChangeTime >= DEBOUNCE_TIME) && (stableLevel != lastRawLevel)) {
    stableLevel = lastRawLevel;
    applyDoorState(stableLevel == HIGH);
    Serial.printf("门事件: %s\n", door_open ? "OPEN" : "CLOSE");
    sendDoorEvent(door_open);
  }

  // 3) 周期心跳（非阻塞，到点才发）
  if (wifiWasConnected && (millis() - lastHeartbeatTime >= HEARTBEAT_INTERVAL)) {
    lastHeartbeatTime = millis();
    sendHeartbeat();
  }

  // 4) 驱动上报队列：发送待发请求、处理失败重发
  processPending();

  // 5) 刷新 LED 指示（放最后，让本轮新触发的“发送提示”能立刻开始）
  updateLed();
}
