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
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_Sensor.h>

//----------------------------------------------------------------------
// Uncomment the following line to use UART communication (e.g. with an external
// device connected to Serial1). Comment it out to use the USB serial (Serial)
// for communication with a computer.
//----------------------------------------------------------------------
//#define COMM_MODE_UART

// ------------------- Communication Config -------------------
#ifdef COMM_MODE_UART
  #define UART_BAUD_RATE   115200  // Adjust as needed
  #define MAX_LINE_LENGTH  256
  // Define UART1 TX/RX pins (adjust according to your wiring)
  #define UART_TX_PIN 17
  #define UART_RX_PIN 16
#endif

// Create a pointer for the communication port (either Serial1 or Serial)
Stream* commPort;

// ------------------- SLAVE MAC ADDRESSES --------------------
// (Replace these MAC addresses as needed)
uint8_t slave1MAC[] = {0x78, 0xEE, 0x4C, 0x11, 0x31, 0x8C}; // wheels
uint8_t slave2MAC[] = {0xF8, 0xB3, 0xB7, 0xD2, 0x3B, 0x00}; // neck
uint8_t slave3MAC[] = {0xF8, 0xB3, 0xB7, 0xD2, 0x3B, 0x64}; // neck
uint8_t slave4MAC[] = {0xF8, 0xB3, 0xB7, 0xD2, 0x3B, 0x68}; // IK arm 1, actuator 6
uint8_t slave5MAC[] = {0xF8, 0xB3, 0xB7, 0xD2, 0x3B, 0x54}; // IK arm 1, actuator 2
uint8_t slave6MAC[] = {0x08, 0xA6, 0xF7, 0xB0, 0x72, 0x1C};
uint8_t slave7MAC[] = {0x08, 0xA6, 0xF7, 0xB0, 0x72, 0x1C};
uint8_t slave8MAC[] = {0x08, 0xA6, 0xF7, 0xB0, 0x72, 0x1C};
uint8_t slave9MAC[] = {0x08, 0xA6, 0xF7, 0xB0, 0x72, 0x1C};
uint8_t slave10MAC[] = {0x08, 0xA6, 0xF7, 0xB0, 0x72, 0x1C};
uint8_t slave11MAC[] = {0x08, 0xA6, 0xF7, 0xB0, 0x72, 0x1C};
uint8_t slave12MAC[] = {0x08, 0xA6, 0xF7, 0xB0, 0x72, 0x1C};
uint8_t slave13MAC[] = {0x08, 0xA6, 0xF7, 0xB0, 0x72, 0x1C};
uint8_t slave14MAC[] = {0x08, 0xA6, 0xF7, 0xB0, 0x72, 0x1C};

uint8_t* slaves[] = { slave1MAC, slave2MAC, slave3MAC, slave4MAC,
                      slave5MAC, slave6MAC, slave7MAC, slave8MAC,
                      slave9MAC, slave10MAC, slave11MAC, slave12MAC,
                      slave13MAC, slave14MAC };
static const int numSlaves = 14;

// Track which peers were successfully added
bool peerAdded[numSlaves] = { false };

// ------------------- ESPNOW CALLBACK ----------------
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Optionally, add code here if you need to track success or failure.
}

// ------------------- SETUP --------------------------
void setup() {
  // Initialize the chosen communication port:
  #ifdef COMM_MODE_UART
    Serial1.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    commPort = &Serial1;
  #else
    Serial.begin(115200);
    commPort = &Serial;
  #endif

  // Allow some time for the serial connection to initialize.
  delay(100);

  // Set Wi-Fi mode to STA and disable power saving for low-latency ESPNOW.
  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.disconnect();

  // Initialize ESPNOW.
  if (esp_now_init() == ESP_OK) {
    esp_now_register_send_cb(onDataSent);

    // Add each slave as an ESPNOW peer.
    for (int i = 0; i < numSlaves; i++) {
      esp_now_peer_info_t peerInfo = {};
      memcpy(peerInfo.peer_addr, slaves[i], 6);
      peerInfo.channel = 0;   // Use the same channel as the master.
      peerInfo.encrypt = false;

      if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        peerAdded[i] = true;
      }
    }
  } else {
    commPort->println("Error initializing ESPNOW");
  }
}

// ------------------- MAIN LOOP ----------------------
void loop() {
  // Define a buffer for incoming data.
  #ifdef COMM_MODE_UART
    static char buffer[MAX_LINE_LENGTH];
  #else
    static char buffer[256];
  #endif
  static int idx = 0;

  // Continuously check if data is available on the chosen communication port.
  while (commPort->available() > 0) {
    char c = (char)commPort->read();

    if (c == '\n') {
      // End-of-line received. Terminate the string.
      buffer[idx] = '\0';

      // Echo the received string back so the sender can see it.
      commPort->print(buffer);
      commPort->print("\n");

      // Send the received data via ESPNOW to all configured peers.
      if (idx > 0) {
        for (int i = 0; i < numSlaves; i++) {
          if (peerAdded[i]) {
            esp_now_send(slaves[i], (uint8_t*)buffer, idx + 1);
          }
        }
      }
      // Reset the buffer index for the next message.
      idx = 0;
    } 
    else {
      // Accumulate incoming characters until a newline is encountered.
      if (idx < (int)(sizeof(buffer) - 1)) {
        buffer[idx++] = c;
      } else {
        // If the buffer overflows, reset the index.
        idx = 0;
      }
    }
  }
  // Loop continuously without delay.
}
