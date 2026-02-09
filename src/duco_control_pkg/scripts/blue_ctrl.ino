#include <WiFi.h>

// WiFi配置
const char* ssid = "BEKJ";
const char* password = "bekj123456";

// GPIO输出引脚定义（继电器等）
const int GPIO_PINS[] = {13, 12, 14, 27};  // 根据实际需要修改
const int PIN_COUNT = 4;

// GPIO输入引脚定义（开关量查询用）
// 这里先占位使用 4 个引脚，请根据实际接线修改
const int SWITCH_PINS[] = {33, 32, 35, 34};
const int SWITCH_PIN_COUNT = 4;

// 步进电机结构体
struct StepperMotor {
  int stepPin;              // 脉冲引脚
  int dirPin;               // 方向引脚
  int enablePin;            // 使能引脚
  bool running;             // 运行状态
  uint32_t pulseWidth;      // 脉冲宽度（微秒）
  uint32_t pulseInterval;   // 脉冲间隔（微秒）
  int32_t stepsRemaining;   // 剩余步数
  bool direction;           // 方向
  unsigned long lastStepTime; // 上次步进时间
};

// 定义多个步进电机（根据实际需要修改）
const int MOTOR_COUNT = 3;  // 电机数量
StepperMotor motors[MOTOR_COUNT] = {
  // 电机0: STEP_PIN, DIR_PIN, ENABLE_PIN
  {25, 26, 33, false, 5, 500, 0, false, 0},
  // 电机1
  {32, 35, 34, false, 5, 500, 0, false, 0},
  // 电机2
  {19, 18, 5, false, 5, 500, 0, false, 0}
};

// TCP服务器配置
WiFiServer server(45678);  // 监听8080端口
WiFiClient client;

// 重连间隔
const unsigned long RECONNECT_INTERVAL = 10000;  // 10秒
unsigned long lastReconnectAttempt = 0;

// CRC8校验表（多项式0x07）
const uint8_t crc8_table[256] = {
  0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
  0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
  0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
  0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
  0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
  0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
  0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
  0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
  0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
  0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
  0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
  0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
  0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
  0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
  0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
  0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

// 计算CRC8校验值
uint8_t calculateCRC8(uint8_t* data, int len) {
  uint8_t crc = 0x00;
  for (int i = 0; i < len; i++) {
    crc = crc8_table[crc ^ data[i]];
  }
  return crc;
}

// 验证CRC8
bool verifyCRC8(uint8_t* data, int len) {
  if (len < 2) return false;  // 至少需要1字节数据+1字节CRC
  uint8_t receivedCRC = data[len - 1];
  uint8_t calculatedCRC = calculateCRC8(data, len - 1);
  return (receivedCRC == calculatedCRC);
}

void setup() {
  Serial.begin(115200);
  
  // 初始化GPIO输出引脚
  for (int i = 0; i < PIN_COUNT; i++) {
    pinMode(GPIO_PINS[i], OUTPUT);
    digitalWrite(GPIO_PINS[i], LOW);
  }
  
  // 初始化GPIO输入引脚（开关量）
  for (int i = 0; i < SWITCH_PIN_COUNT; i++) {
    pinMode(SWITCH_PINS[i], INPUT_PULLUP);  // 默认上拉，根据硬件电路可调整
  }
  
  // 初始化所有步进电机引脚
  for (int i = 0; i < MOTOR_COUNT; i++) {
    pinMode(motors[i].stepPin, OUTPUT);
    pinMode(motors[i].dirPin, OUTPUT);
    pinMode(motors[i].enablePin, OUTPUT);
    digitalWrite(motors[i].stepPin, LOW);
    digitalWrite(motors[i].dirPin, LOW);
    digitalWrite(motors[i].enablePin, HIGH);  // 高电平禁用（根据驱动器调整）
  }
  
  Serial.print("已初始化 ");
  Serial.print(MOTOR_COUNT);
  Serial.println(" 个步进电机");
  
  // 连接WiFi
  connectWiFi();
  
  // 启动服务器
  server.begin();
  Serial.println("服务器已启动");
}

void loop() {
  // 检查WiFi连接状态
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastReconnectAttempt >= RECONNECT_INTERVAL) {
      lastReconnectAttempt = millis();
      connectWiFi();
    }
    return;
  }
  
  // 控制所有步进电机
  unsigned long currentTime = micros();
  for (int i = 0; i < MOTOR_COUNT; i++) {
    if (motors[i].running && motors[i].stepsRemaining > 0) {
      // 检查是否到了该步进的时间
      if (currentTime - motors[i].lastStepTime >= (motors[i].pulseWidth + motors[i].pulseInterval)) {
        digitalWrite(motors[i].stepPin, HIGH);
        delayMicroseconds(motors[i].pulseWidth);
        digitalWrite(motors[i].stepPin, LOW);
        
        motors[i].stepsRemaining--;
        motors[i].lastStepTime = currentTime;
        
        if (motors[i].stepsRemaining == 0) {
          motors[i].running = false;
          Serial.print("电机 ");
          Serial.print(i);
          Serial.println(" 运行完成");
        }
      }
    }
  }
  
  // 处理客户端连接
  if (!client || !client.connected()) {
    client = server.available();
  }
  
  if (client && client.connected()) {
    // 检查是否有数据可读
    if (client.available()) {
      handleIncomingData();
    }
  }
}

// WiFi连接函数
void connectWiFi() {
  Serial.println("正在连接WiFi...");
  Serial.print("SSID: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi连接成功!");
    Serial.print("IP地址: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi连接失败，将在10秒后重试");
  }
}

// 处理接收到的数据
void handleIncomingData() {
  uint8_t buffer[32];  // 接收缓冲区
  int bytesRead = client.readBytes(buffer, sizeof(buffer));
  
  if (bytesRead < 2) {  // 至少需要1字节命令+1字节CRC
    Serial.println("数据长度不足");
    sendErrorResponse(0xFE);  // 0xFE = 数据长度错误
    return;
  }
  
  // CRC校验
  if (!verifyCRC8(buffer, bytesRead)) {
    Serial.println("CRC校验失败！");
    sendErrorResponse(0xFF);  // 0xFF = CRC校验失败
    return;
  }
  
  Serial.print("收到数据，长度: ");
  Serial.print(bytesRead);
  Serial.println("，CRC校验通过");
  
  // 处理命令类型（去除CRC字节）
  uint8_t cmdType = buffer[0];
  
  switch (cmdType) {
    case 0x00:  // 急停命令
      handleEmergencyStop();
      break;
    
    case 0x01:  // GPIO控制命令
      handleGPIOCommand(buffer, bytesRead - 1);  // 去除CRC
      break;
      
    case 0x02:  // 步进电机控制命令
      handleMotorCommand(buffer, bytesRead - 1);  // 去除CRC
      break;
      
    case 0x03:  // 电机停止/查询命令
      handleExtensionCommand(buffer, bytesRead - 1);  // 去除CRC
      break;
      
    default:
      Serial.print("未知命令类型: 0x");
      Serial.println(cmdType, HEX);
      sendErrorResponse(0xFD);  // 0xFD = 未知命令
      break;
  }
}

// 发送错误响应
void sendErrorResponse(uint8_t errorCode) {
  uint8_t response[3];
  response[0] = 0xEE;  // 错误标识
  response[1] = errorCode;
  response[2] = calculateCRC8(response, 2);
  client.write(response, 3);
  client.flush();
}

// 急停函数
void handleEmergencyStop() {
  Serial.println("!!! 急停命令触发 !!!");
  
  // 停止所有电机
  for (int i = 0; i < MOTOR_COUNT; i++) {
    motors[i].running = false;
    motors[i].stepsRemaining = 0;
    digitalWrite(motors[i].enablePin, HIGH);  // 禁用电机
    digitalWrite(motors[i].stepPin, LOW);
    digitalWrite(motors[i].dirPin, LOW);
  }
  
  // 所有GPIO拉低
  for (int i = 0; i < PIN_COUNT; i++) {
    digitalWrite(GPIO_PINS[i], LOW);
  }
  
  Serial.println("所有GPIO已拉低，所有电机已停止");
  
  // 返回确认（带CRC）
  uint8_t response[3];
  response[0] = 0x00;
  response[1] = 0x01;  // 0x01表示急停成功
  response[2] = calculateCRC8(response, 2);
  client.write(response, 3);
  client.flush();
}

// GPIO控制命令处理
void handleGPIOCommand(uint8_t* data, int len) {
  if (len < 5) return;
  
  // 第一个元素(data[0])是命令类型，已在上层处理
  // data[1-4]对应四个GPIO的控制值
  
  // 根据接收到的值设置GPIO
  for (int i = 0; i < PIN_COUNT; i++) {
    if (data[i + 1] == 0) {
      digitalWrite(GPIO_PINS[i], LOW);
      Serial.print("GPIO ");
      Serial.print(GPIO_PINS[i]);
      Serial.println(" -> LOW");
    } else if (data[i + 1] == 1) {
      digitalWrite(GPIO_PINS[i], HIGH);
      Serial.print("GPIO ");
      Serial.print(GPIO_PINS[i]);
      Serial.println(" -> HIGH");
    }
    // 0xFF表示不改变该引脚状态
  }
  // 读取当前GPIO状态
  uint8_t gpioStates[4];
  for (int i = 0; i < PIN_COUNT; i++) {
    gpioStates[i] = digitalRead(GPIO_PINS[i]);
  }
  
  // 立即通过同一TCP连接返回之前的GPIO状态（带CRC）
  if (client.connected()) {
    uint8_t responseWithCRC[5];
    memcpy(responseWithCRC, gpioStates, 4);
    responseWithCRC[4] = calculateCRC8(responseWithCRC, 4);
    client.write(responseWithCRC, 5);
    client.flush();  // 确保数据立即发送
    Serial.println("已通过同一连接返回GPIO状态");
  } else {
    Serial.println("警告：客户端已断开，无法返回状态");
  }
}

// 步进电机控制命令处理
void handleMotorCommand(uint8_t* data, int len) {
  // 数据格式：
  // data[0]: 命令类型 0x02
  // data[1]: 电机编号 (0 ~ MOTOR_COUNT-1)
  // data[2]: 方向 (0=正转, 1=反转)
  // data[3-4]: 脉冲宽度（微秒），高字节在前
  // data[5-6]: 脉冲间隔（微秒），高字节在前
  // data[7-10]: 步数（可选），4字节，高字节在前
  
  if (len < 7) {
    Serial.println("步进电机命令数据不足");
    uint8_t response[] = {0x02, 0xFF, 0x00};  // 0xFF=无效电机, 0x00=失败
    client.write(response, 3);
    return;
  }
  
  // 解析电机编号
  uint8_t motorID = data[1];
  if (motorID >= MOTOR_COUNT) {
    Serial.print("无效的电机编号: ");
    Serial.println(motorID);
    uint8_t response[] = {0x02, motorID, 0x00};  // 0x00表示失败
    client.write(response, 3);
    return;
  }
  
  // 解析方向
  motors[motorID].direction = data[2];
  digitalWrite(motors[motorID].dirPin, motors[motorID].direction);
  
  // 解析脉冲宽度（微秒）
  motors[motorID].pulseWidth = (data[3] << 8) | data[4];
  if (motors[motorID].pulseWidth < 1) motors[motorID].pulseWidth = 1;
  
  // 解析脉冲间隔（微秒）
  motors[motorID].pulseInterval = (data[5] << 8) | data[6];
  if (motors[motorID].pulseInterval < 1) motors[motorID].pulseInterval = 1;
  
  // 解析步数（如果提供）
  if (len >= 11) {
    motors[motorID].stepsRemaining = ((uint32_t)data[7] << 24) | 
                                      ((uint32_t)data[8] << 16) | 
                                      ((uint32_t)data[9] << 8) | 
                                      data[10];
  } else {
    motors[motorID].stepsRemaining = 0;  // 0表示持续运行
  }
  
  // 使能电机
  digitalWrite(motors[motorID].enablePin, LOW);  // 低电平使能（根据驱动器调整）
  motors[motorID].running = true;
  motors[motorID].lastStepTime = micros();
  
  Serial.print("电机 ");
  Serial.print(motorID);
  Serial.println(" 参数:");
  Serial.print("  方向: ");
  Serial.println(motors[motorID].direction ? "反转" : "正转");
  Serial.print("  脉冲宽度: ");
  Serial.print(motors[motorID].pulseWidth);
  Serial.println(" μs");
  Serial.print("  脉冲间隔: ");
  Serial.print(motors[motorID].pulseInterval);
  Serial.println(" μs");
  Serial.print("  步数: ");
  Serial.println(motors[motorID].stepsRemaining == 0 ? "持续运行" : String(motors[motorID].stepsRemaining));
  
  // 返回确认（带CRC）
  uint8_t response[4];
  response[0] = 0x02;
  response[1] = motorID;
  response[2] = 0x01;  // 0x01表示成功
  response[3] = calculateCRC8(response, 3);
  client.write(response, 4);
  client.flush();
}

// 扩展命令处理（电机停止/查询等）
void handleExtensionCommand(uint8_t* data, int len) {
  if (len < 2) return;
  
  uint8_t subCmd = data[1];
  
  switch (subCmd) {
    case 0x01:  // 停止指定电机
      if (len >= 3) {
        uint8_t motorID = data[2];
        if (motorID < MOTOR_COUNT) {
          motors[motorID].running = false;
          motors[motorID].stepsRemaining = 0;
          digitalWrite(motors[motorID].enablePin, HIGH);  // 禁用电机
          Serial.print("电机 ");
          Serial.print(motorID);
          Serial.println(" 已停止");
          
          // 返回确认（带CRC）
          uint8_t response[5];
          response[0] = 0x03;
          response[1] = 0x01;
          response[2] = motorID;
          response[3] = 0x01;
          response[4] = calculateCRC8(response, 4);
          client.write(response, 5);
          client.flush();
        } else {
          Serial.println("无效的电机编号");
          uint8_t response[5];
          response[0] = 0x03;
          response[1] = 0x01;
          response[2] = motorID;
          response[3] = 0x00;
          response[4] = calculateCRC8(response, 4);
          client.write(response, 5);
        }
      }
      break;
      
    case 0x02:  // 查询指定电机状态
      if (len >= 3) {
        uint8_t motorID = data[2];
        if (motorID < MOTOR_COUNT) {
          uint8_t status[8];
          status[0] = 0x03;
          status[1] = 0x02;
          status[2] = motorID;
          status[3] = motors[motorID].running ? 0x01 : 0x00;
          status[4] = (motors[motorID].stepsRemaining >> 16) & 0xFF;
          status[5] = (motors[motorID].stepsRemaining >> 8) & 0xFF;
          status[6] = motors[motorID].stepsRemaining & 0xFF;
          status[7] = calculateCRC8(status, 7);
          client.write(status, 8);
          client.flush();
          Serial.print("已返回电机 ");
          Serial.print(motorID);
          Serial.println(" 状态");
        }
      }
      break;
      
    case 0x03:  // 查询所有电机状态
      {
        uint8_t allStatus[3 + MOTOR_COUNT];
        allStatus[0] = 0x03;
        allStatus[1] = 0x03;
        for (int i = 0; i < MOTOR_COUNT; i++) {
          allStatus[2 + i] = motors[i].running ? 0x01 : 0x00;
        }
        allStatus[2 + MOTOR_COUNT] = calculateCRC8(allStatus, 2 + MOTOR_COUNT);
        client.write(allStatus, 3 + MOTOR_COUNT);
        client.flush();
        Serial.println("已返回所有电机状态");
      }
      break;
      
    case 0x04:  // 查询开关量GPIO状态
      {
        if (!client.connected()) {
          Serial.println("客户端未连接，无法返回GPIO开关状态");
          break;
        }

        // 读取开关 GPIO 状态
        uint8_t swStatus[7];
        swStatus[0] = 0x03;   // 主命令 0x03
        swStatus[1] = 0x04;   // 子命令 0x04 表示 GPIO 查询

        for (int i = 0; i < SWITCH_PIN_COUNT && i < 4; i++) {
          // 将数字读到的电平直接返回（0 或 1）
          swStatus[2 + i] = digitalRead(SWITCH_PINS[i]) ? 0x01 : 0x00;
        }

        // 当 SWITCH_PIN_COUNT < 4 时，未使用的位填 0
        for (int i = SWITCH_PIN_COUNT; i < 4; i++) {
          swStatus[2 + i] = 0x00;
        }

        // 计算 CRC（前 6 字节）
        swStatus[6] = calculateCRC8(swStatus, 6);

        client.write(swStatus, 7);
        client.flush();
        Serial.println("已返回 GPIO 开关状态 (subCmd 0x04)");
      }
      break;
      
    default:
      Serial.println("未知扩展命令");
      break;
  }
}