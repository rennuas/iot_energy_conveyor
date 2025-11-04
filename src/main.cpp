#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Stepper.h>

// ==============================
// WiFi & MQTT CONFIG
// ==============================
const char* ssid       = "";
const char* password   = "";

const char* mqttServer = "";
const uint16_t mqttPort = 8883;
const char* mqttUser   = "";
const char* mqttPass   = "";

WiFiClientSecure espClient;
PubSubClient client(espClient);

// ==============================
// STEPPER MOTOR CONFIG (28BYJ-48 + ULN2003)
// ==============================
// 28BYJ-48 biasanya memiliki 2048 steps per revolution (Half-Step mode)
const int STEPS_PER_REV = 2048; 
const int STEP1_IN1 = 2; 
const int STEP1_IN2 = 4; 
const int STEP1_IN3 = 5; 
const int STEP1_IN4 = 18; 

const int STEP2_IN1 = 19; 
const int STEP2_IN2 = 21; 
const int STEP2_IN3 = 22;
const int STEP2_IN4 = 23; 

const int STEP3_IN1 = 13; 
const int STEP3_IN2 = 14; 
const int STEP3_IN3 = 27; 
const int STEP3_IN4 = 32; 

// Perhatian: Urutan pin pada motor 28BYJ-48 seringkali IN1, IN3, IN2, IN4
// Pastikan urutan pin ini benar untuk motor Anda. Jika tidak, motor akan bergetar/tidak berputar.
Stepper myStepper1(STEPS_PER_REV, STEP1_IN1, STEP1_IN3, STEP1_IN2, STEP1_IN4);
Stepper myStepper2(STEPS_PER_REV, STEP2_IN1, STEP2_IN3, STEP2_IN2, STEP2_IN4);
Stepper myStepper3(STEPS_PER_REV, STEP3_IN1, STEP3_IN3, STEP3_IN2, STEP3_IN4);

// Variabel Kontrol Motor
int motorSpeedRPM = 10; // Kecepatan awal 5 RPM
bool motor1IsRunning = false;
bool motor2IsRunning = false;
bool motor3IsRunning = false;
long lastMotorStep = 0;
const long MOTOR_STEP_INTERVAL = 10; // Interval untuk langkah motor (ms)

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  // Ambil payload sebagai string
  String msg;
  for (int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  Serial.println(msg);

  // Parsing JSON
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, msg);

  if (error) {
    Serial.print("JSON Parsing failed: ");
    Serial.println(error.c_str());
    client.publish("iot_energy/status/conveyor", "{\"commandId\":0,\"status\":\"failed\"}");
    return;
  }

  String commandId = doc["commandId"];
  int setTurnOn = doc["cmd"]["setTurnOn"];
  int setTurnOff  = doc["cmd"]["setTurnOff"];

  bool success = true;
  
  if(strcmp(topic, "iot_energy/trigger/conveyor") == 0){
    if(setTurnOn != 0){
      switch (setTurnOn)
      {
      case 1:
        motor1IsRunning = true;
        break;
      case 2:
        motor2IsRunning = true;
        break;
      case 3:
        motor3IsRunning = true;
        break;
      
      default:
        break;
      }
      
    }
  }

  if(strcmp(topic, "iot_energy/trigger/conveyor/shutdown") == 0){
    motor1IsRunning = false;
    motor2IsRunning = false;
    motor3IsRunning = false;
  }

  // Publish status
  StaticJsonDocument<200> response;
  response["commandId"] = commandId;
  response["status"] = success ? "success" : "failed";

  char buffer[200];
  serializeJson(response, buffer);

  client.publish("iot_energy/status/conveyor", buffer);

  Serial.println(success ? "CONVEYOR ON success" : "CONVEYOR ON failed");
}

// ==============================
// WiFi & MQTT Connect
// ==============================
void ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) return;

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("WiFi connect");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nWiFi OK. IP: %s\n", WiFi.localIP().toString().c_str());

  espClient.setInsecure();

}

void ensureMqtt() {
  if (client.connected()) return;

  Serial.printf("MQTT connect -> %s:%u ...\n", mqttServer, mqttPort);

  String clientId = "ESP32_CNYR_" + String((uint32_t)ESP.getEfuseMac(), HEX);

  client.connect(clientId.c_str(), mqttUser, mqttPass);

  if (!client.connected()) {
    Serial.println("[ERR] MQTT connect gagal");
    return;
  }

  client.subscribe("iot_energy/trigger/conveyor");
  client.subscribe("iot_energy/trigger/conveyor/shutdown");

  Serial.println("MQTT connected & subscribed.");
}

void runMotorLoop() {
    // Motor Stepper terus berputar satu arah (maju 1 langkah setiap panggilan)
    // Motor akan melangkah maju dengan kecepatan yang telah di-set oleh myStepper.setSpeed()
    // Motor.step(1) akan memindahkan 1 langkah, dan myStepper.run() akan memajukan motor
    // dengan kecepatan yang telah ditetapkan. Karena kita hanya perlu 'berputar' 
    // saat LED 3 ON, kita pakai myStepper.step(1) di loop berkala.
    
    if (millis() - lastMotorStep >= MOTOR_STEP_INTERVAL) {
        lastMotorStep = millis();
        // Berputar 1 langkah (putar searah jarum jam)
        if (motor1IsRunning) myStepper1.step(1);
        if (motor2IsRunning) myStepper1.step(2);
        if (motor3IsRunning) myStepper1.step(3);
    }
}


// ==============================
// Setup & Loop
// ==============================
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("\n=== ESP32 MQTT Conveyor Controller (JSON) ===");

  ensureWifi();

  client.setServer(mqttServer, mqttPort);
  client.setCallback(mqttCallback);

  myStepper1.setSpeed(motorSpeedRPM); 
  myStepper2.setSpeed(motorSpeedRPM);
  myStepper3.setSpeed(motorSpeedRPM);
  
}

void loop() {
  ensureWifi();
  ensureMqtt();
  client.loop();
  runMotorLoop();
}
  