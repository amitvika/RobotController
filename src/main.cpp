#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_Sensor.h>
#include <HTTPClient.h>
#include <math.h> // for roundf()
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

//INSTRUCTIONS
//based on the motor direction, switch around the motor control pins
//based on the actuator number, change the string key

// ------------------------- PIN DEFINITIONS -------------------------
#define SDA_PIN_SENSOR   21
#define SCL_PIN_SENSOR   22
#define MOTOR_CTRL_PIN1  14
#define MOTOR_CTRL_PIN2  13
#define MOTOR_ENABLE_PIN 12 //12 on 5 off
#define AS5600_ADDRESS   0x36
#define ACTUATOR_CENTER 3661 //actuator 6

// ---------------------- CONTROL PARAMETERS -------------------------
// Initial motor setpoint (will be updated via received ESP-NOW data)
uint16_t desiredPosition = ACTUATOR_CENTER;
float kp = 0.3;
int maxPWM = 255;
int positionErrorAllowance = 1;
int deadZone = 20;

// --------------------- READ AS5600 ANGLE ---------------------------
// Reads the 12-bit angle from the AS5600 sensor.
uint16_t readAS5600Angle() {
Wire.beginTransmission(AS5600_ADDRESS);
Wire.write(0x0C);
Wire.endTransmission(false);
Wire.requestFrom(AS5600_ADDRESS, (uint8_t)2);
if (Wire.available() == 2) {
  uint8_t highByte = Wire.read();
  uint8_t lowByte  = Wire.read();
  uint16_t angle   = ((uint16_t)highByte << 8) | lowByte;
  return (angle & 0x0FFF);
}
return 0;
}

// ------------------ WRAP-AROUND DIFFERENCE FUNCTION ----------------
// Ensures angular difference is in [-2048, 2047].
int angleDifference(uint16_t currentAngle, uint16_t targetAngle) {
int diff = (int)targetAngle - (int)currentAngle;

// Since AS5600 is 12-bit => 4096 counts per revolution
// Wrap to range [-2048, 2047]:
if (diff > 2047) {
  diff -= 4096; 
} else if (diff < -2048) {
  diff += 4096;
}
return diff;
}

// ------------------ MOTOR CONTROL FUNCTIONS ------------------------
// Compute a PWM output based on the error between current and desired positions.
int computeTargetPWM(uint16_t currentPosition, uint16_t desiredPosition) {
// Use the wrapped difference:
int error = angleDifference(currentPosition, desiredPosition);

if (abs(error) <= positionErrorAllowance) {
  return 0;
}

int pwmValue = kp * error;
pwmValue = constrain(pwmValue, -maxPWM, maxPWM);

// Ensure the output exceeds the dead zone
if (pwmValue > 0) {
  pwmValue = max(pwmValue, deadZone);
} else if (pwmValue < 0) {
  pwmValue = min(pwmValue, -deadZone);
}
return pwmValue;
}

// Apply the computed PWM to the motor using two LEDC channels.
void applyPWM(int pwmValue) {
if (pwmValue > 0) {
  ledcWrite(0, pwmValue);
  ledcWrite(1, 0);
} else if (pwmValue < 0) {
  ledcWrite(0, 0);
  ledcWrite(1, -pwmValue);
} else {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}
}

// ------------------- Setup PWM Pins and Channels -------------------
void setupPinsAndPWMs() {
// Enable the motor driver.
pinMode(MOTOR_ENABLE_PIN, OUTPUT);
digitalWrite(MOTOR_ENABLE_PIN, HIGH);

// Set up control pins as outputs.
pinMode(MOTOR_CTRL_PIN1, OUTPUT);
pinMode(MOTOR_CTRL_PIN2, OUTPUT);

// Configure two LEDC channels for motor control (20kHz, 8-bit resolution).
ledcSetup(0, 20000, 8); // Channel 0 for one motor direction.
ledcSetup(1, 20000, 8); // Channel 1 for the other direction.
ledcAttachPin(MOTOR_CTRL_PIN1, 0);
ledcAttachPin(MOTOR_CTRL_PIN2, 1);

// Initially stop the motor.
ledcWrite(0, 0);
ledcWrite(1, 0);
}

// ------------------- ESP-NOW RECEIVE CODE --------------------------
#define MAX_BUFFER_SIZE 256

void parseActuatorValueLine(char* line);

// Callback when an ESP‑NOW packet is received.
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
if (len <= 0) return;

char buffer[MAX_BUFFER_SIZE];
int copyLen = (len < MAX_BUFFER_SIZE - 1) ? len : MAX_BUFFER_SIZE - 1;
memcpy(buffer, incomingData, copyLen);
buffer[copyLen] = '\0';

// Debug: print the raw received string.
Serial.print("Received via ESP-NOW: ");
Serial.println(buffer);

// Parse the string for actuator values; in particular, look for "h1X".
parseActuatorValueLine(buffer);
}

// Parse a semicolon-separated key/value string and update desiredPosition if "h1X" is found.
void parseActuatorValueLine(char* line) {
char* token = strtok(line, ";");
while (token != NULL) {
  char* eqPos = strchr(token, '=');
  if (eqPos) {
    *eqPos = '\0';   // Terminate the key token.
    char* key = token;
    char* val = eqPos + 1;
    // Look for the key "h1X" (you can add more keys if needed).
    if (strcmp(key, "h1y") == 0) {
      desiredPosition = (uint16_t)atoi(val);
      Serial.print("Updated desiredPosition (h1y): ");
      Serial.println(desiredPosition);
    }
  }
  token = strtok(NULL, ";");
}
}

// ------------------- ESP-NOW SETUP ---------------------------------
void initESPNOW() {
WiFi.mode(WIFI_STA);
WiFi.disconnect();
if (esp_now_init() != ESP_OK) {
  Serial.println("Error initializing ESP-NOW");
}
// Register the receive callback.
esp_now_register_recv_cb(onDataRecv);
}

// ------------------------------ SETUP ------------------------------
void setup() {
Serial.begin(115200);
delay(100); // Allow time for the serial connection to initialize
Serial.println("Starting ESP-NOW Motor Controller Receiver...");

setupPinsAndPWMs();
Wire.begin(SDA_PIN_SENSOR, SCL_PIN_SENSOR);

initESPNOW();

Serial.print("Receiver MAC Address: ");
Serial.println(WiFi.macAddress());
}

// ------------------------------ LOOP -------------------------------
void loop() {
uint16_t angle = readAS5600Angle();
int targetPWM  = computeTargetPWM(angle, desiredPosition);
applyPWM(targetPWM);

// Debug output: sensor reading, current setpoint, and PWM value.
Serial.print("Angle: ");
Serial.print(angle);
Serial.print(" | Setpoint: ");
Serial.print(desiredPosition);
Serial.print(" | PWM: ");
Serial.println(targetPWM);

delay(10);  // Adjust delay as needed for stable control loop timing.
}
