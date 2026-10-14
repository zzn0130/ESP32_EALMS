#include <Arduino.h>

// 板上 LED 引脚（开发板丝印 D2 = GPIO2）
#define LED_PIN 2

// D15 检测引脚（ESP32 开发板 D15 = GPIO15）
#define DETECT_PIN 15

#define DEBOUNCE_TIME 50  //去抖时间长度

//门状态：0为关闭，1为打开
static bool door_status = 0;

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
}

void loop() {
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
    } else {
      // 低电平：门关（磁铁靠近）→ 灯亮
      door_status = 0;
      digitalWrite(LED_PIN, HIGH);
      Serial.println("Door OFF");
    }
  }
  

}
