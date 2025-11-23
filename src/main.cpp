#include <DHT.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <ArduinoJson.h>
#include <PubSubClient.h> // Thư viện MQTT
#include <WiFiManager.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

// --- CẤU HÌNH THINGSBOARD ---
// Thay đổi TOKEN bằng "Access Token" của thiết bị trên ThingsBoard
#define TB_SERVER "eu.thingsboard.cloud" // Hoặc IP server ThingsBoard của bạn (VD: 192.168.1.100)
#define TB_TOKEN "nSFoTxXKtTbbjknjUU9Z"
#define TB_PORT 1883

// --- CẤU HÌNH PHẦN CỨNG (GPIO ESP32) ---
#define DHTPIN 4
#define DHTTYPE DHT11
#define SOIL_PIN 34
#define RELAY2_PIN 5
#define LED_BUG 2
#define TRIG_PIN 18
#define ECHO_PIN 19

#define I2C_SDA 21
#define I2C_SCL 22

// --- KHỞI TẠO ĐỐI TƯỢNG ---
Adafruit_INA219 ina219;
DHT dht(DHTPIN, DHTTYPE);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000);

WiFiClient espClient;
PubSubClient client(espClient); // Khởi tạo MQTT Client

// --- BIẾN TOÀN CỤC ---
unsigned long lastUpdateData = 0;
unsigned long lastLedCheck = 0;

// Trạng thái máy bơm
bool pumpOn = false;
bool manualOverride = false; // Cờ báo bơm đang chạy do thủ công (nút bấm/RPC)
unsigned long pumpStartTime = 0;
unsigned long pumpDuration = 10000;
bool autoPump = true; // Mặc định bật chế độ tự động

// Trạng thái Lỗi
int typeError = 0;
bool sendNotiLowPower = true;
bool sendNotiDhtError = true;
bool sendNotiSoilMoisError = true;

// Biến lưu dữ liệu
float temperature = 0;
float humidity = 0;
float percentSoilMoisture = 0;
float batteryLevel = 0;
float tankWaterLevel = 0;
float heightTankWater = 100;

// --- KHAI BÁO HÀM (FORWARD DECLARATIONS) ---
void turnOnPump(String reason);
void turnOffPump();
void checkPump();
void ledBug(int type);
void sendTelemetry(float t, float h, float soil, float bat, float tank);
float checkLevelWater();
float checkLevelBattery();
void readSensorsAndUpload();
void reconnect();

// --- HÀM CALLBACK MQTT (XỬ LÝ LỆNH TỪ SERVER) ---
void onMessage(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Nhan tin hieu MQTT [");
  Serial.print(topic);
  Serial.print("]: ");

  String message;
  for (int i = 0; i < length; i++)
  {
    message += (char)payload[i];
  }
  Serial.println(message);

  // Parse JSON
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, message);

  if (error)
  {
    Serial.print("Loi parse JSON: ");
    Serial.println(error.c_str());
    return;
  }

  // 1. Xử lý RPC (Remote Procedure Call) - Điều khiển bơm
  // Topic mẫu: v1/devices/me/rpc/request/1
  String topicStr = String(topic);
  if (topicStr.startsWith("v1/devices/me/rpc/request/"))
  {
    String method = doc["method"];

    if (method == "setPump")
    {
      bool params = doc["params"]; // true = BẬT, false = TẮT
      if (params)
      {
        turnOnPump("RPC Command");
      }
      else
      {
        turnOffPump();
      }
    }
    else if (method == "reset")
    {
      Serial.println("Reset ESP32 via RPC...");
      ESP.restart();
    }
  }

  // 2. Xử lý Shared Attributes (Cấu hình hệ thống)
  // Topic: v1/devices/me/attributes
  if (topicStr.equals("v1/devices/me/attributes"))
  {
    if (doc.containsKey("pumpDuration"))
    {
      pumpDuration = doc["pumpDuration"].as<unsigned long>();
      Serial.print("Cap nhat pumpDuration: ");
      Serial.println(pumpDuration);
    }
    if (doc.containsKey("autoPump"))
    {
      autoPump = doc["autoPump"].as<bool>();
      Serial.print("Cap nhat autoPump: ");
      Serial.println(autoPump);
    }
    if (doc.containsKey("heightWaterTank"))
    {
      heightTankWater = doc["heightWaterTank"].as<float>();
      Serial.print("Cap nhat heightWaterTank: ");
      Serial.println(heightTankWater);
    }
  }
}

void configModeCallback(WiFiManager *myWiFiManager)
{
  Serial.println(F("Vao che do cau hinh (Config Mode)"));
  Serial.println(WiFi.softAPIP());
}

// --- SETUP ---
void setup()
{
  Serial.begin(115200);
  Serial.println(F("\nDang khoi dong ESP32 (ThingsBoard MQTT)..."));

  analogReadResolution(10);

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!ina219.begin())
  {
    Serial.println(F("Khong tim thay INA219"));
    typeError = 3;
  }

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  digitalWrite(RELAY2_PIN, LOW);
  pinMode(SOIL_PIN, INPUT);
  pinMode(LED_BUG, OUTPUT);

  WiFiManager wifiManager;
  wifiManager.setAPCallback(configModeCallback);
  if (!wifiManager.autoConnect("ESP32_SmartGarden"))
  {
    ESP.restart();
  }

  Serial.println(F("WiFi Connected!"));
  digitalWrite(LED_BUG, LOW);

  timeClient.begin();
  dht.begin();

  // Cấu hình MQTT Server
  client.setServer(TB_SERVER, TB_PORT);
  client.setCallback(onMessage); // Đăng ký hàm xử lý khi có tin nhắn tới
  client.setBufferSize(1024);    // Tăng buffer để nhận JSON dài
}

// --- HÀM KẾT NỐI LẠI MQTT ---
void reconnect()
{
  // Lặp cho đến khi kết nối được
  while (!client.connected())
  {
    Serial.print("Dang ket noi ThingsBoard MQTT...");
    // Kết nối với Client ID (tự sinh) và Username là Access Token
    if (client.connect("ESP32_Garden_Client", TB_TOKEN, NULL))
    {
      Serial.println("Da ket noi!");

      // 1. Subscribe topic RPC để nhận lệnh điều khiển
      client.subscribe("v1/devices/me/rpc/request/+");

      // 2. Subscribe topic Attributes để nhận thay đổi cấu hình
      client.subscribe("v1/devices/me/attributes");

      // 3. Gửi yêu cầu lấy cấu hình hiện tại từ Server (Shared Attributes)
      client.publish("v1/devices/me/attributes/request/1", "{\"clientKeys\":\"pumpDuration,autoPump,heightWaterTank\"}");
    }
    else
    {
      Serial.print("Loi, rc=");
      Serial.print(client.state());
      Serial.println(" thu lai sau 5s");
      typeError = 6; // Lỗi kết nối MQTT
      ledBug(typeError);
      delay(5000);
    }
  }
  if (typeError == 6)
    typeError = 0;
}

// --- LOOP ---
void loop()
{
  ledBug(typeError);

  if (WiFi.status() != WL_CONNECTED)
  {
    typeError = 5;
    WiFi.reconnect();
    return;
  }
  else
  {
    if (typeError == 5)
      typeError = 0;
  }

  // Kiểm tra kết nối MQTT
  if (!client.connected())
  {
    reconnect();
  }
  client.loop(); // Hàm quan trọng để duy trì kết nối MQTT và nhận tin nhắn

  timeClient.update();
  checkPump();

  unsigned long currentMillis = millis();
  if (currentMillis - lastUpdateData >= 2000)
  {
    readSensorsAndUpload();
    lastUpdateData = currentMillis + 4000;
  }

  // Logic tự động
  int hour = timeClient.getHours();
  if (!manualOverride && (hour == 6 || hour == 18) && percentSoilMoisture <= 50 && temperature <= 40 && autoPump)
  {
    turnOnPump("Auto Pump");
  }
}

// --- HELPER FUNCTIONS ---

void readSensorsAndUpload()
{
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();

  int rawSoil = analogRead(SOIL_PIN);
  percentSoilMoisture = 100 - map(constrain(rawSoil, 350, 1024), 350, 1024, 0, 100);

  batteryLevel = checkLevelBattery();
  tankWaterLevel = checkLevelWater();

  // Xử lý lỗi cảm biến (giữ nguyên logic cũ)
  if (isnan(temperature) || isnan(humidity))
  {
    typeError = 1;
  }
  else if (rawSoil <= 20 || rawSoil >= 1023)
  {
    typeError = 2;
  }
  else
  {
    if (typeError == 1 || typeError == 2)
      typeError = 0;
  }

  // Gửi dữ liệu lên ThingsBoard (Telemetry)
  sendTelemetry(temperature, humidity, percentSoilMoisture, batteryLevel, tankWaterLevel);
}

void sendTelemetry(float t, float h, float soil, float bat, float tank)
{
  DynamicJsonDocument doc(256);
  doc["temperature"] = t;
  doc["humidity"] = h;
  doc["soilMoisture"] = soil;
  doc["batteryLevel"] = bat;
  doc["tankWaterLevel"] = tank;
  doc["pumpState"] = pumpOn; // Gửi thêm trạng thái bơm để Dashboard hiển thị
  doc["error"] = typeError;

  char payload[256];
  serializeJson(doc, payload);

  // Topic Telemetry mặc định của ThingsBoard
  client.publish("v1/devices/me/telemetry", payload);
}

void turnOnPump(String reason)
{
  if (!pumpOn)
  {
    pumpOn = true;
    manualOverride = true;
    digitalWrite(RELAY2_PIN, HIGH);
    pumpStartTime = millis();
    Serial.println("Bom BAT: " + reason);

    // Gửi sự kiện bật bơm lên server (tùy chọn)
    client.publish("v1/devices/me/telemetry", "{\"pumpState\": true, \"action\": \"Pump ON\"}");
  }
}

void turnOffPump()
{
  if (pumpOn)
  {
    pumpOn = false;
    manualOverride = false;
    digitalWrite(RELAY2_PIN, LOW);
    Serial.println("Bom TAT");
    client.publish("v1/devices/me/telemetry", "{\"pumpState\": false, \"action\": \"Pump OFF\"}");
  }
}

void checkPump()
{
  if (pumpOn && (millis() - pumpStartTime >= pumpDuration))
  {
    turnOffPump();
    Serial.println(F("Bom Tu Dong TAT (Het gio)"));
  }
}

float checkLevelWater()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2;
  // Tránh chia cho 0
  if (heightTankWater <= 0)
    return 0;
  float level = distance * 100.0 / heightTankWater;
  return level;
}

float checkLevelBattery()
{
  float loadVoltage = ina219.getBusVoltage_V() + (ina219.getShuntVoltage_mV() / 1000.0);
  float percent = (loadVoltage - 9.0) / (12.6 - 9.0) * 100.0;
  return constrain(percent, 0, 100);
}

void ledBug(int type)
{
  unsigned long currentMillis = millis();
  unsigned long interval = 0;
  bool useDelay = false;
  bool alwaysOn = false;

  switch (type)
  {
  case 1:
    interval = 500;
    break;
  case 2:
    interval = 1000;
    break;
  case 3:
    interval = 2000;
    useDelay = true;
    break;
  case 4:
    interval = 4000;
    useDelay = true;
    break;
  case 5:
    alwaysOn = true;
    break;
  case 6:
    interval = 250;
    useDelay = true;
    break;
  default:
    digitalWrite(LED_BUG, LOW);
    return;
  }

  if (alwaysOn)
  {
    digitalWrite(LED_BUG, HIGH);
    return;
  }

  if (currentMillis - lastLedCheck >= interval)
  {
    lastLedCheck = currentMillis;
    digitalWrite(LED_BUG, HIGH);
    if (useDelay)
      delay(50);
  }
  else
  {
    digitalWrite(LED_BUG, LOW);
  }
}