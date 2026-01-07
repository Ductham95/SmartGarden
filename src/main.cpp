#include <DHT.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// --- CẤU HÌNH ---
#define TB_SERVER "eu.thingsboard.cloud"
#define TB_TOKEN "50FjeUVAMhq00TRZhBfw"
#define TB_PORT 1883

#define DHTPIN 4
#define DHTTYPE DHT11
#define SOIL_PIN 34
#define RELAY2_PIN 5
#define LED_BUG 2
#define TRIG_PIN 18
#define ECHO_PIN 19
#define I2C_SDA 21
#define I2C_SCL 22

// --- ĐỐI TƯỢNG ---
Adafruit_INA219 ina219;
DHT dht(DHTPIN, DHTTYPE);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000);
WiFiClient espClient;
PubSubClient client(espClient);

// --- SHARED DATA (DỮ LIỆU DÙNG CHUNG) ---
// Dùng Struct để gom nhóm dữ liệu, dễ quản lý hơn biến rời rạc
struct SystemState
{
  float temp = 0;
  float hum = 0;
  float soil = 0;
  float bat = 0;
  float tankLevel = 0;

  // Config
  float heightTankWater = 100;
  unsigned long pumpDuration = 10000;
  bool autoPump = true;
  String wateringSchedulesJson = "[]";

  // Status
  bool pumpOn = false;
  bool manualOverride = false;
  int typeError = 0;
};

SystemState sysState;     // Biến toàn cục lưu trạng thái
SemaphoreHandle_t xMutex; // Khóa bảo vệ dữ liệu (Mutex)

// Biến hỗ trợ logic bơm
unsigned long pumpStartTime = 0;

// --- PROTOTYPES ---
void taskComm(void *parameter);
void taskSensor(void *parameter);
void taskControl(void *parameter);
void taskLed(void *parameter);
void saveConfigCallback(WiFiManager *myWiFiManager);
void onMessage(char *topic, byte *payload, unsigned int length);
float checkLevelWater();
float checkLevelBattery();

// --- SETUP ---
void setup()
{
  Serial.begin(115200);

  // 1. Khởi tạo Hardware
  analogReadResolution(10);
  Wire.begin(I2C_SDA, I2C_SCL);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  digitalWrite(RELAY2_PIN, LOW); // Đảm bảo tắt bơm khi khởi động
  pinMode(SOIL_PIN, INPUT);
  pinMode(LED_BUG, OUTPUT);
  dht.begin();

  if (!ina219.begin())
  {
    Serial.println("Warning: INA219 not found");
  }

  // 2. Tạo Mutex
  xMutex = xSemaphoreCreateMutex();

  // 3. Kết nối WiFi (Blocking - Chặn cho đến khi có WiFi mới chạy tiếp)
  WiFiManager wifiManager;
  wifiManager.setAPCallback(saveConfigCallback);
  if (!wifiManager.autoConnect("ESP32_SmartGarden"))
  {
    ESP.restart();
  }
  Serial.println("WiFi Connected!");

  timeClient.begin();
  client.setServer(TB_SERVER, TB_PORT);
  client.setCallback(onMessage);
  client.setBufferSize(2048); // Tăng buffer cho JSON lịch tưới

  // 4. Khởi tạo Tasks (FreeRTOS)
  xTaskCreatePinnedToCore(taskComm, "TaskComm", 8192, NULL, 1, NULL, 1);       // Core 1, Ưu tiên 1
  xTaskCreatePinnedToCore(taskSensor, "TaskSensor", 4096, NULL, 1, NULL, 1);   // Core 1, Ưu tiên 1
  xTaskCreatePinnedToCore(taskControl, "TaskControl", 4096, NULL, 2, NULL, 1); // Core 1, Ưu tiên 2
  xTaskCreatePinnedToCore(taskLed, "TaskLed", 1024, NULL, 0, NULL, 1);         // Core 1, Ưu tiên thấp

  Serial.println("System Started with FreeRTOS!");
}

void loop()
{
  vTaskDelete(NULL);
}

// --- TASK 1: COMMUNICATION (WIFI / MQTT / TELEMETRY) ---
void taskComm(void *parameter)
{
  unsigned long lastTelemetry = 0;
  const unsigned long TELEMETRY_INTERVAL = 2000;

  for (;;)
  {
    // 1. Check WiFi
    if (WiFi.status() != WL_CONNECTED)
    {
      // Logic reconnect WiFi nếu cần, hoặc để WiFiManager tự xử lý nhưng nên hạn chế block lâu
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    // 2. Check MQTT & Reconnect
    if (!client.connected())
    {
      Serial.print("Connecting MQTT...");
      if (client.connect("ESP32_Garden_Client", TB_TOKEN, NULL))
      {
        Serial.println("Connected!");
        client.subscribe("v1/devices/me/rpc/request/+");
        client.subscribe("v1/devices/me/attributes");
        client.publish("v1/devices/me/attributes/request/1", "{\"clientKeys\":\"pumpDuration,autoPump,heightWaterTank,wateringSchedules\"}");
      }
      else
      {
        Serial.print("Failed rc=");
        Serial.println(client.state());
        vTaskDelay(5000 / portTICK_PERIOD_MS); // Chờ 5s không block các task khác
      }
    }

    // 3. Maintain connection
    client.loop();

    // 4. Update Time
    timeClient.update();

    // 5. Send Telemetry
    if (millis() - lastTelemetry >= TELEMETRY_INTERVAL)
    {
      lastTelemetry = millis();

      // Lấy dữ liệu an toàn từ Struct
      SystemState snapshot;
      if (xSemaphoreTake(xMutex, (TickType_t)100) == pdTRUE)
      {
        snapshot = sysState; // Copy dữ liệu ra biến tạm để gửi
        xSemaphoreGive(xMutex);

        // Tạo JSON gửi đi
        DynamicJsonDocument doc(512);
        doc["temperature"] = snapshot.temp;
        doc["humidity"] = snapshot.hum;
        doc["soilMoisture"] = snapshot.soil;
        doc["batteryLevel"] = snapshot.bat;
        doc["tankWaterLevel"] = snapshot.tankLevel;
        doc["pumpState"] = snapshot.pumpOn;
        doc["error"] = snapshot.typeError;

        char payload[512];
        serializeJson(doc, payload);
        client.publish("v1/devices/me/telemetry", payload);
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); // Yield cho CPU nghỉ
  }
}

// --- TASK 2: SENSORS (READ HARDWARE) ---
void taskSensor(void *parameter)
{
  for (;;)
  {
    float t = dht.readTemperature();
    float h = dht.readHumidity();

    int rawSoil = analogRead(SOIL_PIN);
    float soil = 100 - map(constrain(rawSoil, 350, 1024), 350, 1024, 0, 100);

    float bat = checkLevelBattery();

    // Cần lấy chiều cao bể từ config (cần Mutex để đọc an toàn)
    float tankHeight = 100;
    if (xSemaphoreTake(xMutex, (TickType_t)100) == pdTRUE)
    {
      tankHeight = sysState.heightTankWater;
      xSemaphoreGive(xMutex);
    }

    // Đo khoảng cách
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH);
    float distance = duration * 0.034 / 2;
    float tankLevel = (tankHeight > 0) ? (distance * 100.0 / tankHeight) : 0;

    // Phân tích lỗi
    int err = 0;
    if (isnan(t) || isnan(h))
      err = 1;
    else if (rawSoil <= 20 || rawSoil >= 1023)
      err = 2;

    // CẬP NHẬT VÀO STRUCT CHUNG (Thread Safe)
    if (xSemaphoreTake(xMutex, (TickType_t)100) == pdTRUE)
    {
      sysState.temp = t;
      sysState.hum = h;
      sysState.soil = soil;
      sysState.bat = bat;
      sysState.tankLevel = tankLevel;

      // Chỉ cập nhật lỗi nếu không phải lỗi mạng (lỗi mạng do TaskComm quản lý)
      if (sysState.typeError != 5 && sysState.typeError != 6)
      {
        sysState.typeError = err;
      }
      xSemaphoreGive(xMutex);
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS); // Đọc mỗi 2 giây
  }
}

// --- TASK 3: CONTROL (LOGIC BƠM & LỊCH) ---
void taskControl(void *parameter)
{
  unsigned long lastScheduleCheck = 0;

  for (;;)
  {
    // 1. Lấy dữ liệu snapshot để xử lý logic
    SystemState current;
    bool dataReady = false;

    if (xSemaphoreTake(xMutex, (TickType_t)100) == pdTRUE)
    {
      current = sysState;
      dataReady = true;
      xSemaphoreGive(xMutex);
    }

    if (dataReady)
    {
      unsigned long now = millis();
      bool shouldPump = false;
      String reason = "";

      // Logic ngắt bơm theo thời gian
      if (current.pumpOn && (now - pumpStartTime >= current.pumpDuration))
      {
        // Tắt bơm
        digitalWrite(RELAY2_PIN, LOW);
        Serial.println("Auto OFF Pump");

        // Update state
        if (xSemaphoreTake(xMutex, (TickType_t)100) == pdTRUE)
        {
          sysState.pumpOn = false;
          sysState.manualOverride = false;
          xSemaphoreGive(xMutex);
        }
      }

      // Logic Tự động theo cảm biến (Chỉ chạy khi không Override và Auto mode ON)
      int h = timeClient.getHours();
      if (!current.pumpOn && !current.manualOverride && current.autoPump)
      {
        if ((h == 6 || h == 18) && current.soil <= 50 && current.temp <= 40)
        {
          shouldPump = true;
          reason = "Auto Sensor Logic";
        }
      }

      // Logic Lịch Tưới (Mỗi 60s check 1 lần)
      if (!current.pumpOn && !current.manualOverride && (now - lastScheduleCheck > 60000))
      {
        lastScheduleCheck = now;
        // Parse JSON lịch tưới (đã lưu trong struct)
        DynamicJsonDocument schedDoc(2048);
        DeserializationError err = deserializeJson(schedDoc, current.wateringSchedulesJson);

        if (!err)
        {
          JsonArray schedules = schedDoc.as<JsonArray>();
          int cDay = timeClient.getDay(); // 0=Sun
          if (cDay == 0)
            cDay = 7;
          int cHour = timeClient.getHours();
          int cMin = timeClient.getMinutes();

          for (JsonObject sch : schedules)
          {
            if (!(sch["isEnabled"] | false))
              continue;

            bool dayMatch = false;
            for (int d : sch["daysOfWeek"].as<JsonArray>())
            {
              if (d == cDay)
              {
                dayMatch = true;
                break;
              }
            }

            if (dayMatch && (sch["hour"] | 0) == cHour && (sch["minute"] | 0) == cMin)
            {
              shouldPump = true;
              reason = "Schedule: " + sch["name"].as<String>();
              // Update duration tạm thời cho lần tưới này
              if (xSemaphoreTake(xMutex, (TickType_t)100) == pdTRUE)
              {
                sysState.pumpDuration = sch["duration"] | 10000;
                xSemaphoreGive(xMutex);
              }
              break;
            }
          }
        }
      }

      // THỰC THI BẬT BƠM
      if (shouldPump)
      {
        digitalWrite(RELAY2_PIN, HIGH);
        pumpStartTime = now;
        Serial.println("Pump ON: " + reason);

        if (xSemaphoreTake(xMutex, (TickType_t)100) == pdTRUE)
        {
          sysState.pumpOn = true;
          sysState.manualOverride = true; // Đánh dấu đang chạy
          xSemaphoreGive(xMutex);
        }
      }
    }

    vTaskDelay(500 / portTICK_PERIOD_MS); // Check logic mỗi 0.5s
  }
}

// --- TASK 4: LED INDICATOR ---
void taskLed(void *parameter)
{
  for (;;)
  {
    int errCode = 0;
    if (xSemaphoreTake(xMutex, (TickType_t)100) == pdTRUE)
    {
      errCode = sysState.typeError;
      xSemaphoreGive(xMutex);
    }

    // Logic nháy đèn đơn giản hóa
    if (errCode == 0)
    {
      digitalWrite(LED_BUG, LOW);
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    else
    {
      // Nháy theo mã lỗi
      digitalWrite(LED_BUG, HIGH);
      vTaskDelay(200 / portTICK_PERIOD_MS);
      digitalWrite(LED_BUG, LOW);
      // Thời gian nghỉ tùy lỗi
      int delayTime = (errCode == 5 || errCode == 6) ? 200 : 1000;
      vTaskDelay(delayTime / portTICK_PERIOD_MS);
    }
  }
}

// HELPER & CALLBACKS

void onMessage(char *topic, byte *payload, unsigned int length)
{
  String message;
  for (int i = 0; i < length; i++)
    message += (char)payload[i];
  Serial.print("MQTT: ");
  Serial.println(message);

  DynamicJsonDocument doc(1024);
  deserializeJson(doc, message);
  String topicStr = String(topic);

  // Cần Mutex để cập nhật config an toàn
  if (xSemaphoreTake(xMutex, (TickType_t)500) == pdTRUE)
  {

    // 1. RPC CONTROL
    if (topicStr.startsWith("v1/devices/me/rpc/request/"))
    {
      String method = doc["method"];
      if (method == "setPump")
      {
        bool status = doc["params"];
        if (status)
        {
          digitalWrite(RELAY2_PIN, HIGH);
          sysState.pumpOn = true;
          sysState.manualOverride = true;
          pumpStartTime = millis();
        }
        else
        {
          digitalWrite(RELAY2_PIN, LOW);
          sysState.pumpOn = false;
          sysState.manualOverride = false;
        }
      }
      else if (method == "reset")
      {
        ESP.restart();
      }
    }

    // 2. SHARED ATTRIBUTES
    if (topicStr.equals("v1/devices/me/attributes"))
    {
      if (doc.containsKey("pumpDuration"))
        sysState.pumpDuration = doc["pumpDuration"];
      if (doc.containsKey("autoPump"))
        sysState.autoPump = doc["autoPump"];
      if (doc.containsKey("heightWaterTank"))
        sysState.heightTankWater = doc["heightWaterTank"];
      if (doc.containsKey("wateringSchedules"))
        sysState.wateringSchedulesJson = doc["wateringSchedules"].as<String>();
    }

    xSemaphoreGive(xMutex);
  }
}

void saveConfigCallback(WiFiManager *myWiFiManager)
{
  Serial.println("Enter Config Mode");
}

float checkLevelBattery()
{
  float loadVoltage = ina219.getBusVoltage_V() + (ina219.getShuntVoltage_mV() / 1000.0);
  return constrain((loadVoltage - 9.0) / (12.6 - 9.0) * 100.0, 0, 100);
}