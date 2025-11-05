/*
 * Arduino NANO ESP32-S3 Fish Tank Primary Control System v3.6
 * Manages up to 25 fish breeding tanks via ESP-NOW and web interface
 *
 * 
 * 
 * NEW IN v3.6:
 * - OLED goes off at night
 * - fixed the grey of the sleeping tank devices
 * - hostname instead of IP address, now: http://fishbreedtanks.local
 * - V.36
 *  got rid of fish logo, it was crowding the OLED for the hostname
 *
 * Required Libraries:
 * - ArduinoJson v6.21.3+ : https://github.com/bblanchon/ArduinoJson
 * - AsyncTCP v1.1.4+ : https://github.com/me-no-dev/AsyncTCP
 * - ESPAsyncWebServer v1.2.4+ : https://github.com/me-no-dev/ESPAsyncWebServer
 * - Adafruit GFX Library v1.11.9+ : https://github.com/adafruit/Adafruit-GFX-Library
 * - Adafruit SSD1306 v2.5.9+ : https://github.com/adafruit/Adafruit_SSD1306
 *
 * Folder Structure:
 * - 
 */

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESPmDNS.h>

// OLED params
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3D
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Fish graphic
static const unsigned char PROGMEM fish_bmp[] = {
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b01100000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000001, 0b11110000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00001111, 0b11000000, 0b00000111, 0b11111000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b01111111, 0b11110000, 0b00011111, 0b11111100, 0b00000000,
  0b00000000, 0b00000000, 0b00000001, 0b11111111, 0b11111000, 0b01111111, 0b11111110, 0b00000000,
  0b00000000, 0b00000000, 0b00000111, 0b11111111, 0b11111100, 0b11111111, 0b11111111, 0b00000000,
  0b00000000, 0b00000000, 0b00001111, 0b11111111, 0b11111110, 0b11111111, 0b11111111, 0b10000000,
  0b00000000, 0b00000000, 0b00011111, 0b11100111, 0b11111111, 0b11111111, 0b11111111, 0b11000000,
  0b00000000, 0b00000000, 0b00111111, 0b11000011, 0b11111111, 0b11111111, 0b11111111, 0b11100000,
  0b00000000, 0b00000000, 0b01111111, 0b10000001, 0b11111111, 0b11111111, 0b11111111, 0b11110000,
  0b00000000, 0b00000000, 0b01111111, 0b00000000, 0b11111111, 0b11111111, 0b11111111, 0b11110000,
  0b00000000, 0b00000000, 0b11111110, 0b00000000, 0b01111111, 0b11111111, 0b11111111, 0b11111000,
  0b00000000, 0b00000000, 0b11111100, 0b00000000, 0b00111111, 0b11111111, 0b11111111, 0b11111000,
  0b00000000, 0b00000000, 0b11111100, 0b00000000, 0b00111111, 0b11111111, 0b11111111, 0b11111000,
  0b00000000, 0b00000000, 0b11111110, 0b00000000, 0b01111111, 0b11111111, 0b11111111, 0b11111000,
  0b00000000, 0b00000000, 0b01111111, 0b00000000, 0b11111111, 0b11111111, 0b11111111, 0b11110000,
  0b00000000, 0b00000000, 0b01111111, 0b10000001, 0b11111111, 0b11111111, 0b11111111, 0b11110000,
  0b00000000, 0b00000000, 0b00111111, 0b11000011, 0b11111111, 0b11111111, 0b11111111, 0b11100000,
  0b00000000, 0b00000000, 0b00011111, 0b11100111, 0b11111111, 0b11111111, 0b11111111, 0b11000000,
  0b00000000, 0b00000000, 0b00001111, 0b11111111, 0b11111110, 0b11111111, 0b11111111, 0b10000000,
  0b00000000, 0b00000000, 0b00000111, 0b11111111, 0b11111100, 0b11111111, 0b11111111, 0b00000000,
  0b00000000, 0b00000000, 0b00000001, 0b11111111, 0b11111000, 0b01111111, 0b11111110, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b01111111, 0b11110000, 0b00011111, 0b11111100, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00001111, 0b11000000, 0b00000111, 0b11111000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000001, 0b11110000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b01100000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000
};

// ===== CONFIGURATION =====
const char* ssid = "BHN-Guest";
const char* password = "N/A";
const char* slackWebhook = "N/A";

uint8_t secondaryMACAddresses[25][6] = {
  {0x30, 0xED, 0xA0, 0xAC, 0xAB, 0xE8}, // secondary1
  {0x94, 0xA9, 0x90, 0x17, 0x84, 0x70}, // secondary2
  {0x30, 0xED, 0xA0, 0xAC, 0xAC, 0xC0}, // secondary3
  {0x30, 0xED, 0xA0, 0xAC, 0xAE, 0x78}, // secondary4
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x05}, // secondary5
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x06}, // secondary6
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x07}, // secondary7
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x08}, // secondary8
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x09}, // secondary9
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x0A}, // secondary10
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x0B}, // secondary11
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x0C}, // secondary12
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x0D}, // secondary13
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x0E}, // secondary14
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x0F}, // secondary15
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x10}, // secondary16
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x11}, // secondary17
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x12}, // secondary18
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x13}, // secondary19
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x14}, // secondary20
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x15}, // secondary21
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x16}, // secondary22
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x17}, // secondary23
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x18}, // secondary24
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x19}  // secondary25
};

// ===== GLOBAL VARIABLES =====
const int MAX_TANKS = 25;
AsyncWebServer server(80);

bool tankConnected[MAX_TANKS] = {false};        // REF1
int batteryLevels[MAX_TANKS] = {0};            // REF5
bool lowBatteryNotified[MAX_TANKS] = {false};   // NEW: prevent repeated low-battery Slack spam
String errorMessages[MAX_TANKS] = {""};        // REF4
String tankMACs[MAX_TANKS] = {""};             // REF8
bool tankStatus[MAX_TANKS] = {false};          // REF9
bool delayRunning[MAX_TANKS] = {false};        // REF11
unsigned long delayStartTime[MAX_TANKS] = {0};
int delayDuration[MAX_TANKS] = {0};
unsigned long lastHeartbeat[MAX_TANKS] = {0};
bool tankInSleep[MAX_TANKS] = {false};         // REF10
unsigned long sleepStartTime[MAX_TANKS] = {0};
int sleepDuration[MAX_TANKS] = {0};

// clock time
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -8 * 3600;
const int daylightOffset_sec = 3600;

// display off times
// Add these variables near the top with other configuration
// ===== OLED DISPLAY TIME CONTROL =====
const int OLED_OFF_HOUR = 19;  // 7pm (24-hour format)
const int OLED_ON_HOUR = 6;    // 6am (24-hour format)
bool oledIsOn = true;          // Track OLED state


// Add at top with global variables, slack non blocking
#include <queue>
std::queue<String> slackQueue;
unsigned long lastSlackSend = 0;

typedef struct {
    int tankId;
    char command[32];
    int value;
    char message[128];
    int messageType;
} ESPNowMessage;

// ===== INITIALIZATION =====

/**
 * Initialize WiFi connection
 * Inputs: None
 * Outputs: None (connects to WiFi network)
 */
void initWiFi() {
    Serial.println("FUNCTION: initWiFi() starting");
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    // after WiFi begins / connects (in initWiFi or setup) this is PLUS addon 2 lines:
    WiFi.setSleep(false);              // Arduino API
    esp_wifi_set_ps(WIFI_PS_NONE);     // extra belt-and-suspenders


    Serial.print("Connecting to WiFi");
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(1000);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.print("WiFi connected! IP address: ");
        Serial.println(WiFi.localIP());
        Serial.println("Primary WiFi Channel: " + String(WiFi.channel()));
        Serial.println("FUNCTION: initWiFi() completed successfully");
    } else {
        Serial.println();
        Serial.println("ERROR: Failed to connect to WiFi");
        Serial.println("FUNCTION: initWiFi() failed");
    }
}

/**
 * Initialize NTP time synchronization
 * Inputs: None
 * Outputs: None (configures time sync)
 */
void initTime() {
    Serial.println("FUNCTION: initTime() starting");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    Serial.println("FUNCTION: initTime() completed");
}

/**
 * Add all secondary devices as ESP-NOW peers
 * Inputs: None
 * Outputs: None (adds peers to ESP-NOW)
 */
void addESPNowPeers() {
    Serial.println("FUNCTION: addESPNowPeers() starting");
    int currentChannel = WiFi.channel();
    
    for (int i = 0; i < MAX_TANKS; i++) {
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, secondaryMACAddresses[i], 6);
        peerInfo.channel =  0; // follow current Wi-Fi channel (auto)
        peerInfo.encrypt = false;
        
        if (esp_now_add_peer(&peerInfo) == ESP_OK) {
            Serial.println("Added Tank " + String(i + 1) + " as peer");
        } else {
            Serial.println("ERROR: Failed to add Tank " + String(i + 1));
        }
    }
    Serial.println("FUNCTION: addESPNowPeers() completed");
}

/**
 * Initialize ESP-NOW protocol
 * Inputs: None
 * Outputs: None (initializes ESP-NOW)
 */
void initESPNow() {
    Serial.println("FUNCTION: initESPNow() starting");
    
    int currentChannel = WiFi.channel();
    Serial.println("WiFi channel before ESP-NOW: " + String(currentChannel));
    
    WiFi.mode(WIFI_STA);
    
    // esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);  // commented out: primary follows AP channel
    delay(500);
    
    int verifiedChannel = WiFi.channel();
    Serial.println("WiFi channel after mode switch: " + String(verifiedChannel));
    
    if (verifiedChannel != currentChannel) {
        Serial.println("WARNING: Channel mismatch detected! Forcing again...");
        // esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);  // commented out: primary follows AP channel
        delay(500);
        verifiedChannel = WiFi.channel();
        Serial.println("WiFi channel after second force: " + String(verifiedChannel));
    }

    // ADD: Extra verification before ESP-NOW init
    delay(200);
    verifiedChannel = WiFi.channel();
    if (verifiedChannel != currentChannel) {
        Serial.println("ERROR: Cannot stabilize channel! Expected " + String(currentChannel) + " got " + String(verifiedChannel));
    }
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("ERROR: Failed to initialize ESP-NOW");
        Serial.println("FUNCTION: initESPNow() failed");
        return;
    }
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("ERROR: Failed to initialize ESP-NOW");
        Serial.println("FUNCTION: initESPNow() failed");
        return;
    }
    
    esp_now_register_send_cb(onESPNowDataSent);
    esp_now_register_recv_cb(onESPNowDataReceived);
    
    addESPNowPeers();
    Serial.println("FUNCTION: initESPNow() completed successfully");
}

// ===== TIME FUNCTIONS =====

/**
 * Get current time as formatted string
 * Inputs: None
 * Outputs: String - formatted time "YYYY-MM-DD HH:MM:SS PST"
 */
String getCurrentTime() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        return "Time unavailable";
    }
    
    char timeStr[100];
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S PST", &timeinfo);
    return String(timeStr);
}

/**
 * Calculate wake-up time for sleeping tank (REF13)
 * Inputs: arrayIndex - int (0-24)
 * Outputs: String - wake time "HH:MM PST"
 */
String getWakeUpTime(int arrayIndex) {
    if (!tankInSleep[arrayIndex]) {
        return "";
    }
    
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        return "Time unavailable";
    }
    
    time_t now;
    time(&now);
    unsigned long elapsed = millis() - sleepStartTime[arrayIndex];
    unsigned long remaining = ((unsigned long)sleepDuration[arrayIndex] * 3600000) - elapsed;
    time_t wakeTime = now + (remaining / 1000);
    
    struct tm* wakeTimeInfo = localtime(&wakeTime);
    
    char timeStr[20];
    strftime(timeStr, sizeof(timeStr), "%H:%M PST", wakeTimeInfo);
    return String(timeStr);
}

// ===== ESP-NOW CALLBACKS =====

/**
 * Callback when ESP-NOW data is sent
 * Inputs: mac_addr (uint8_t*), status (esp_now_send_status_t)
 * Outputs: None
 */
void onESPNowDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Non-blocking - just track status
}

/**
 * Callback when ESP-NOW data is received
 * Inputs: mac (uint8_t*), incomingData (uint8_t*), len (int)
 * Outputs: None (updates tank arrays)
 */
void onESPNowDataReceived(const uint8_t *mac, const uint8_t *incomingData, int len) {
    ESPNowMessage message;
    memcpy(&message, incomingData, sizeof(message));
    
    int tankId = message.tankId;
    if (tankId < 1 || tankId > MAX_TANKS) {
        Serial.println("ERROR: Invalid tank ID: " + String(tankId));
        return;
    }
    
    int arrayIndex = tankId - 1;
    
    tankConnected[arrayIndex] = true;
    lastHeartbeat[arrayIndex] = millis();
    
    Serial.println("FUNCTION: onESPNowDataReceived() - Tank " + String(tankId) + " type " + String(message.messageType));
    
    switch (message.messageType) {
        case 1:
            Serial.println("Status update from Tank " + String(tankId));
            break;
            
        case 2: // REF4: Error
            errorMessages[arrayIndex] = String(message.message);
            Serial.println("ERROR: Tank " + String(tankId) + ": " + String(message.message));
            /* Slack disabled for generic errors to avoid noise */
            break;
            
        case 3: // REF5: Battery
            batteryLevels[arrayIndex] = message.value;
            Serial.println("Tank " + String(tankId) + " battery: " + String(message.value) + "%");
            // Notify once when battery first drops below 20%, reset once back above 25%
            if (message.value < 20 && !lowBatteryNotified[arrayIndex]) {
                queueSlackMessage("Tank" + String(tankId) + " battery low (" + String(message.value) + "%)");
                lowBatteryNotified[arrayIndex] = true;
            } else if (message.value >= 25 && lowBatteryNotified[arrayIndex]) {
                lowBatteryNotified[arrayIndex] = false;
            }
            break;
            
        case 4: // REF8: MAC
            tankMACs[arrayIndex] = String(message.message);
            Serial.println("Tank " + String(tankId) + " MAC: " + String(message.message));
            break;
            
        case 5: // REF9: Door status
            {
                bool prev = tankStatus[arrayIndex];
                tankStatus[arrayIndex] = (message.value == 1);
                Serial.println("Tank " + String(tankId) + " status: " + (tankStatus[arrayIndex] ? "Open" : "Closed"));
                if (tankStatus[arrayIndex] != prev) {
                    if (tankStatus[arrayIndex]) {
                        queueSlackMessage("Tank" + String(tankId) + " opened the slot/door");
                    } else {
                        queueSlackMessage("Tank" + String(tankId) + " closed the slot/door");
                    }
                }
            }
            break;
            
        case 6: // Heartbeat
            break;
            
        default:
            Serial.println("WARNING: Unknown message type: " + String(message.messageType));
            break;
    }
}

// ===== ESP-NOW COMMANDS =====

/**
 * Send command to specific tank
 * Inputs: tankId (int 1-25), command (String), value (int, optional)
 * Outputs: None (sends ESP-NOW message)
 */
void sendCommandToTank(int tankId, String command, int value = 0) {
    if (tankId < 1 || tankId > MAX_TANKS) {
        return;
    }
    
    Serial.println("FUNCTION: sendCommandToTank() - Tank " + String(tankId) + " cmd: " + command);
    
    ESPNowMessage message;
    message.tankId = tankId;
    strncpy(message.command, command.c_str(), sizeof(message.command) - 1);
    message.command[sizeof(message.command) - 1] = '\0';
    message.value = value;
    message.messageType = 0;
    
    int arrayIndex = tankId - 1;
    esp_err_t result = esp_now_send(secondaryMACAddresses[arrayIndex], (uint8_t *)&message, sizeof(message));
    
    if (result != ESP_OK) {
        Serial.println("ERROR: Send failed to Tank " + String(tankId));
    }
}

/**
 * Send command to all tanks
 * Inputs: command (String)
 * Outputs: None (sends to all 25 tanks)
 */
void sendCommandToAllTanks(String command) {
    Serial.println("FUNCTION: sendCommandToAllTanks() - " + command);
    for (int i = 1; i <= MAX_TANKS; i++) {
        sendCommandToTank(i, command);
        delay(50);
    }
}

// ===== TANK CONTROL =====

/**
 * Open all tanks (REF2)
 * Inputs: None
 * Outputs: None
 */
void openAllTanks() {
    Serial.println("FUNCTION: openAllTanks() executed");
    sendCommandToAllTanks("open_slot");
}

/**
 * Close all tanks (REF3)
 * Inputs: None
 * Outputs: None
 */
void closeAllTanks() {
    Serial.println("FUNCTION: closeAllTanks() executed");
    sendCommandToAllTanks("close_slot");
}

/**
 * Test close specific tank (REF6)
 * Inputs: tankId (int 1-25)
 * Outputs: None
 */
void testCloseTank(int tankId) {
    Serial.println("FUNCTION: testCloseTank() - Tank " + String(tankId));
    sendCommandToTank(tankId, "close_slot");
}

/**
 * Test open specific tank (REF7)
 * Inputs: tankId (int 1-25)
 * Outputs: None
 */
void testOpenTank(int tankId) {
    Serial.println("FUNCTION: testOpenTank() - Tank " + String(tankId));
    sendCommandToTank(tankId, "open_slot");
}

/**
 * Ping all tanks for connectivity test
 * Inputs: None
 * Outputs: None
 */
void pingAllTanks() {
    Serial.println("FUNCTION: pingAllTanks() executed");
    sendCommandToAllTanks("ping_test");
}

// ===== TIMER FUNCTIONS =====

/**
 * Start delay timer for tank (REF11)
 * Inputs: tankId (int 1-25), hours (int 1-24)
 * Outputs: None
 */
void startDelayTimer(int tankId, int hours) {
    if (tankId < 1 || tankId > MAX_TANKS || hours < 1 || hours > 24) {
        Serial.println("ERROR: Invalid delay parameters");
        return;
    }
    
    Serial.println("FUNCTION: startDelayTimer() - Tank " + String(tankId) + " for " + String(hours) + "h");
    
    int arrayIndex = tankId - 1;
    delayRunning[arrayIndex] = true;
    delayStartTime[arrayIndex] = millis();
    delayDuration[arrayIndex] = hours;
    
    sendCommandToTank(tankId, "start_delay", hours);
    queueSlackMessage("Tank" + String(tankId) + " has timed open enabled");
}

/**
 * Cancel all timed openings (REF11)
 * Inputs: None
 * Outputs: None
 */
void cancelAllTimedOpenings() {
    Serial.println("FUNCTION: cancelAllTimedOpenings() executed");
    
    for (int i = 0; i < MAX_TANKS; i++) {
        if (delayRunning[i]) {
            delayRunning[i] = false;
            delayStartTime[i] = 0;
            delayDuration[i] = 0;
            
            int tankId = i + 1;
            sendCommandToTank(tankId, "cancel_delay");
            queueSlackMessage("Tank" + String(tankId) + " canceled timed open");
        }
    }
}

/**
 * Cancel delay timer for specific tank (REF12)
 * Inputs: tankId (int 1-25)
 * Outputs: None
 */
void cancelDelayTimer(int tankId) {
    if (tankId < 1 || tankId > MAX_TANKS) {
        return;
    }
    
    Serial.println("FUNCTION: cancelDelayTimer() - Tank " + String(tankId));
    
    int arrayIndex = tankId - 1;
    
    if (delayRunning[arrayIndex]) {
        delayRunning[arrayIndex] = false;
        delayStartTime[arrayIndex] = 0;
        delayDuration[arrayIndex] = 0;
        
        sendCommandToTank(tankId, "cancel_delay");
        queueSlackMessage("Tank" + String(tankId) + " canceled timed open");
    }
}

/**
 * Send sleep command to tank (REF10, REF13)
 * Inputs: tankId (int 1-25), hours (int 1-24)
 * Outputs: None
 */
void sendSleepCommand(int tankId, int hours) {
    if (tankId < 1 || tankId > MAX_TANKS || hours < 1 || hours > 24) {
        Serial.println("ERROR: Invalid sleep parameters");
        return;
    }
    
    Serial.println("FUNCTION: sendSleepCommand() - Tank " + String(tankId) + " for " + String(hours) + "h");
    
    int arrayIndex = tankId - 1;
    
    tankInSleep[arrayIndex] = true;
    sleepStartTime[arrayIndex] = millis();
    sleepDuration[arrayIndex] = hours;
    tankConnected[arrayIndex] = false;
    
    sendCommandToTank(tankId, "go_sleep", hours);
    queueSlackMessage("Tank" + String(tankId) + " has gone into low power sleep for " + String(hours) + " hours");
}

/**
 * v2.6: Clear all error messages (REF4)
 * Inputs: None
 * Outputs: None (clears error array)
 */
void clearAllErrorMessages() {
    Serial.println("FUNCTION: clearAllErrorMessages() executed");
    for (int i = 0; i < MAX_TANKS; i++) {
        errorMessages[i] = "";
    }
}

/**
 * Check delay timers
 * Inputs: None
 * Outputs: None
 */
void checkDelayTimers() {
    for (int i = 0; i < MAX_TANKS; i++) {
        if (delayRunning[i]) {
            unsigned long elapsed = millis() - delayStartTime[i];
            unsigned long duration = (unsigned long)delayDuration[i] * 3600000;
            
            if (elapsed >= duration) {
                delayRunning[i] = false;
                Serial.println("Delay timer expired for Tank " + String(i + 1));
            }
        }
    }
}

/**
 * Check sleep timers
 * Inputs: None
 * Outputs: None
 */
void checkSleepTimers() {
    for (int i = 0; i < MAX_TANKS; i++) {
        if (tankInSleep[i]) {
            unsigned long elapsed = millis() - sleepStartTime[i];
            unsigned long duration = (unsigned long)sleepDuration[i] * 3600000;
            
            if (elapsed >= duration) {
                tankInSleep[i] = false;
                sleepStartTime[i] = 0;
                sleepDuration[i] = 0;
                Serial.println("Sleep period ended for Tank " + String(i + 1));
                queueSlackMessage("Tank" + String(i + 1) + " awoke from sleep");
            }
        }
    }
}

/**
 * Check connection status of all tanks
 * Inputs: None
 * Outputs: None
 */
void checkConnectionStatus() {
    unsigned long currentTime = millis();
    for (int i = 0; i < MAX_TANKS; i++) {
        // Don't mark sleeping tanks as disconnected
        if (tankConnected[i] && !tankInSleep[i] && (currentTime - lastHeartbeat[i] > 180000)) {
            tankConnected[i] = false;
            delayRunning[i] = false;
            // Don't clear tankInSleep here
            Serial.println("Tank " + String(i + 1) + " connection timeout");
        }
    }
}

// ===== SLACK =====



// Replace sendSlackMessage with:
void queueSlackMessage(String message) {
    slackQueue.push(message);
    Serial.println("QUEUED Slack: " + message);
}
/**
 * Send message to Slack webhook
 * Inputs: message (String)
 * Outputs: None
 */


/**
 * Send test Slack notification
 * Inputs: None
 * Outputs: None
 */
void sendTestSlackMessage() {
    Serial.println("FUNCTION: sendTestSlackMessage() executed");
    queueSlackMessage("Testing Slack notification");
}

// ===== WEB PAGE GENERATION =====

/**
 * Get tank display information
 * Inputs: arrayIndex (int 0-24)
 * Outputs: String "cssClass|statusText"
 */
String getTankDisplayInfo(int arrayIndex) {
    // Check sleep status first, before connection status
    if (tankInSleep[arrayIndex]) {
        return "tank-sleeping|Sleeping";
    }
    
    if (!tankConnected[arrayIndex]) {
        return "tank-disconnected|Disconnected";
    }
    
    if (delayRunning[arrayIndex]) {
        return "tank-timer|Timer Active";
    } else {
        return "tank-connected|Connected";
    }
}

/**
 * Generate home page HTML
 * Inputs: None
 * Outputs: String (complete HTML page)
 */
String generateHomePage() {
    String currentTime = getCurrentTime();

    String html;
    html.reserve(14000);

    html += F("<!DOCTYPE html><html><head>");
    html += F("<title>Fish breeding tank control interface</title>");
    html += F("<meta name='viewport' content='width=device-width, initial-scale=1'>");
    html += F("<style>");
    html += F("body { font-family: 'Segoe UI', sans-serif; margin: 0; padding: 20px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); min-height: 100vh; }");
    html += F(".container { max-width: 1200px; margin: 0 auto; background: rgba(255,255,255,0.95); border-radius: 20px; padding: 30px; box-shadow: 0 20px 40px rgba(0,0,0,0.1); }");
    html += F("h1 { color: #2c3e50; text-align: center; font-size: 2.5rem; margin-bottom: 10px; }");
    html += F(".time { text-align: center; font-size: 1.3rem; color: #7f8c8d; margin-bottom: 40px; }");
    html += F(".tank-grid { display: grid; grid-template-columns: repeat(5, 1fr); gap: 15px; margin: 30px 0; }");
    html += F(".tank-indicator { padding: 20px; border-radius: 12px; text-align: center; font-weight: bold; font-size: 0.9rem; transition: transform 0.3s; }");
    html += F(".tank-connected { background: linear-gradient(135deg, #2ecc71, #27ae60); color: white; }");
    html += F(".tank-disconnected { background: linear-gradient(135deg, #e74c3c, #c0392b); color: white; }");
    html += F(".tank-sleeping { background: linear-gradient(135deg, #95a5a6, #7f8c8d); color: white; }");
    html += F(".tank-timer { background: linear-gradient(135deg, #f39c12, #e67e22); color: white; animation: pulse 2s infinite; }");
    html += F("@keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.7; } }");
    html += F(".tank-indicator:hover { transform: translateY(-5px); }");
    html += F(".nav-link { display: inline-block; background: linear-gradient(135deg, #3498db, #2980b9); color: white; padding: 15px 30px; text-decoration: none; border-radius: 25px; font-weight: bold; margin: 20px auto; }");
    html += F(".nav-container { text-align: center; margin-top: 40px; }");
    html += F(".legend { display: flex; justify-content: center; gap: 20px; margin: 20px 0; flex-wrap: wrap; }");
    html += F(".legend-item { display: flex; align-items: center; gap: 8px; padding: 8px 12px; border-radius: 8px; background: rgba(255,255,255,0.8); }");
    html += F(".legend-color { width: 20px; height: 20px; border-radius: 4px; }");
    html += F(".legend-connected { background: linear-gradient(135deg, #2ecc71, #27ae60); }");
    html += F(".legend-disconnected { background: linear-gradient(135deg, #e74c3c, #c0392b); }");
    html += F(".legend-sleeping { background: linear-gradient(135deg, #95a5a6, #7f8c8d); }");
    html += F(".legend-timer { background: linear-gradient(135deg, #f39c12, #e67e22); }");
    html += F("</style>");
    html += F("<script>setInterval(function(){ location.reload(); }, 30000);</script>");
    html += F("</head><body>");
    html += F("<div class='container'>");
    html += F("<h1>Fish breeding tank control interface</h1>");
    html += F("<div class='time'>Current Time: ");
    html += currentTime;
    html += F("</div>");
    html += F("<div class='legend'>");
    html += F("<div class='legend-item'><div class='legend-color legend-connected'></div><span>Connected</span></div>");
    html += F("<div class='legend-item'><div class='legend-color legend-disconnected'></div><span>Disconnected</span></div>");
    html += F("<div class='legend-item'><div class='legend-color legend-sleeping'></div><span>Sleep Mode</span></div>");
    html += F("<div class='legend-item'><div class='legend-color legend-timer'></div><span>Timer Active</span></div>");
    html += F("</div>");
    html += F("<h2 style='text-align: center;'>Tank Status Indicators</h2>");
    html += F("<div class='tank-grid'>");

    
    for (int i = 0; i < MAX_TANKS; i++) {
        String displayInfo = getTankDisplayInfo(i);
        int separatorPos = displayInfo.indexOf("|");
        String cssClass = displayInfo.substring(0, separatorPos);
        String statusText = displayInfo.substring(separatorPos + 1);
        
        html += "<div class='tank-indicator " + cssClass + "'>";
        html += "Tank " + String(i + 1) + "<br>" + statusText;
        
        if (tankConnected[i]) {
            if (tankInSleep[i]) {
                String wakeTime = getWakeUpTime(i);
                html += "<br><small>Wake: " + wakeTime + "</small>";
            } else if (delayRunning[i]) {
                unsigned long elapsed = millis() - delayStartTime[i];
                unsigned long remaining = ((unsigned long)delayDuration[i] * 3600000) - elapsed;
                int hoursLeft = remaining / 3600000;
                int minutesLeft = (remaining % 3600000) / 60000;
                html += "<br><small>" + String(hoursLeft) + "h " + String(minutesLeft) + "m</small>";
            }
        }
        
        html += "</div>";
    }
    
    html += F("</div>");
    html += F("<div class='nav-container'>");
    html += F("<a href='/tank-settings' class='nav-link'>Tank Settings</a>");
    html += F("</div>");
    html += F("</div></body></html>");
    
    return html;
}

/**
 * Generate tank settings page HTML - v3.3 optimized
 * Inputs: None
 * Outputs: String (complete HTML page)
 */
String generateTankSettingsPage() {
    String html;
    html.reserve(18000);  // Reserve memory for better performance
    
    html += F("<!DOCTYPE html><html><head>");
    html += F("<title>Tank Settings</title>");
    html += F("<meta name='viewport' content='width=device-width, initial-scale=1'>");
    html += F("<style>");
    html += F("body { font-family: 'Segoe UI', sans-serif; margin: 0; padding: 20px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); min-height: 100vh; }");
    html += F(".container { max-width: 1400px; margin: 0 auto; background: rgba(255,255,255,0.95); border-radius: 20px; padding: 30px; box-shadow: 0 20px 40px rgba(0,0,0,0.1); }");
    html += F("h1 { color: #2c3e50; text-align: center; margin-bottom: 30px; }");
    html += F(".control-section { margin: 30px 0; padding: 20px; background: rgba(52, 152, 219, 0.1); border-radius: 15px; border-left: 5px solid #3498db; }");
    html += F(".btn { padding: 12px 25px; margin: 10px; border: none; border-radius: 8px; cursor: pointer; font-weight: bold; transition: all 0.3s; }");
    html += F(".btn-success { background: linear-gradient(135deg, #27ae60, #229954); color: white; }");
    html += F(".btn-danger { background: linear-gradient(135deg, #e74c3c, #c0392b); color: white; }");
    html += F(".btn-warning { background: linear-gradient(135deg, #f39c12, #d68910); color: white; }");
    html += F(".btn-info { background: linear-gradient(135deg, #3498db, #2980b9); color: white; }");
    html += F(".btn-secondary { background: linear-gradient(135deg, #95a5a6, #7f8c8d); color: white; }");
    html += F(".btn:hover { transform: translateY(-2px); }");
    html += F(".error-box { background: #f8d7da; color: #721c24; padding: 15px; border-radius: 8px; margin: 15px 0; max-height: 200px; overflow-y: auto; }");
    html += F(".tank-grid { display: grid; grid-template-columns: repeat(5, 1fr); gap: 15px; margin: 30px 0; }");
    html += F(".tank-box { padding: 15px; border-radius: 8px; text-align: center; font-weight: bold; text-decoration: none; color: white; transition: transform 0.3s; }");
    html += F(".tank-box.connected { background: linear-gradient(135deg, #2ecc71, #27ae60); cursor: pointer; }");
    html += F(".tank-box.connected:hover { transform: translateY(-3px); }");
    html += F(".tank-box.disconnected { background: linear-gradient(135deg, #e74c3c, #c0392b); cursor: not-allowed; opacity: 0.7; }");
    html += F(".tank-box.sleeping { background: linear-gradient(135deg, #95a5a6, #7f8c8d); cursor: pointer; }");
    html += F(".tank-box.timer { background: linear-gradient(135deg, #f39c12, #e67e22); cursor: pointer; animation: pulse 2s infinite; }");
    html += F("@keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.7; } }");
    html += F(".back-link { display: inline-block; margin-bottom: 20px; color: #3498db; text-decoration: none; font-weight: bold; }");
    html += F("</style>");
    html += F("</head><body>");
    html += F("<div class='container'>");
    html += F("<a href='/' class='back-link'>Back to Home</a>");
    html += F("<h1>Tank Settings Control Panel</h1>");
    
    // Global controls
    html += F("<div class='control-section'>");
    html += F("<h2>Global Tank Controls</h2>");
    html += F("<button class='btn btn-success' onclick=\"sendCommand('open_all')\">Open all tanks</button>");
    html += F("<button class='btn btn-danger' onclick=\"sendCommand('close_all')\">Close all tanks</button>");
    html += F("<button class='btn btn-warning' onclick=\"sendCommand('cancel_all_timers')\">Cancel all timed openings</button>");
    html += F("<button class='btn btn-info' onclick=\"sendCommand('test_slack')\">Test Slack Notify</button>");
    html += F("<button class='btn btn-secondary' onclick=\"sendCommand('ping_all')\">Ping All Tanks</button>");
    html += F("</div>");
    
    // Error messages (REF4) with Clear Messages button
    html += F("<div class='control-section'>");
    html += F("<h2>System Error Messages</h2>");
    html += F("<div class='error-box'>");
    
    bool hasErrors = false;
    for (int i = 0; i < MAX_TANKS; i++) {
        if (errorMessages[i] != "") {
            html += "Tank " + String(i + 1) + ": " + errorMessages[i] + "<br>";
            hasErrors = true;
        }
    }
    
    if (!hasErrors) {
        html += F("No errors reported from any tanks.");
    }
    
    html += F("</div>");
    html += F("<button class='btn btn-warning' onclick=\"sendCommand('clear_messages')\">Clear Messages</button>");
    html += F("</div>");
    
    // Tank status boxes
    html += F("<div class='control-section'>");
    html += F("<h2>Individual Tank Controls</h2>");
    html += F("<div class='tank-grid'>");
    
    for (int i = 1; i <= MAX_TANKS; i++) {
        int arrayIndex = i - 1;
        String displayInfo = getTankDisplayInfo(arrayIndex);
        int separatorPos = displayInfo.indexOf("|");
        String statusText = displayInfo.substring(separatorPos + 1);
        
        // FIXED: Check sleep status first, before connection status
        String boxClass = "disconnected";
        if (tankInSleep[arrayIndex]) {
            boxClass = "sleeping";
        } else if (tankConnected[arrayIndex]) {
            if (delayRunning[arrayIndex]) {
                boxClass = "timer";
            } else {
                boxClass = "connected";
            }
        }
        
        // Allow clicking on sleeping tanks to view their status
        if (tankConnected[arrayIndex] || tankInSleep[arrayIndex]) {
            html += "<a href='/tank" + String(i) + "' class='tank-box " + boxClass + "'>";
        } else {
            html += "<div class='tank-box " + boxClass + "'>";
        }
        
        html += "Tank" + String(i) + "<br><small>" + statusText + "</small>";
        
        if (tankConnected[arrayIndex] || tankInSleep[arrayIndex]) {
            html += F("</a>");
        } else {
            html += F("</div>");
        }
    }
    
    html += F("</div></div>");
    
    // JavaScript
    html += F("<script>");
    html += F("function sendCommand(cmd) {");
    html += F("  fetch('/command', { method: 'POST', headers: { 'Content-Type': 'application/x-www-form-urlencoded' }, body: 'command=' + cmd })");
    html += F("  .then(response => { if(response.ok) { alert('Command sent'); if(cmd=='clear_messages') location.reload(); } else alert('Failed'); })");
    html += F("  .catch(err => { alert('Network error'); });");
    html += F("}");
    html += F("setInterval(function(){ location.reload(); }, 60000);");
    html += F("</script>");
    html += F("</div></body></html>");
    
    return html;
}

/**
 * Generate individual tank page HTML - v2.7 optimized
 * Inputs: tankId (int 1-25)
 * Outputs: String (complete HTML page)
 */
String generateTankPage(int tankId) {
    if (tankId < 1 || tankId > MAX_TANKS) {
        String html;
        html.reserve(200);
        html += F("<!DOCTYPE html><html><head><title>Error</title></head><body>");
        html += F("<h1>Error: Invalid Tank ID</h1>");
        html += F("<a href='/'>Home</a></body></html>");
        return html;
    }
    
    int arrayIndex = tankId - 1;
    
    if (!tankConnected[arrayIndex]) {
        String html;
        html.reserve(500);
        html += F("<!DOCTYPE html><html><head>");
        html += F("<title>Tank ");
        html += String(tankId);
        html += F("</title>");
        html += F("</head><body style='font-family: Arial; text-align: center; padding: 50px;'>");
        html += F("<h1>Tank ");
        html += String(tankId);
        html += F(" is Disconnected</h1>");
        html += F("<p>This tank must be connected before you can access its controls.</p>");
        html += F("<a href='/tank-settings'>Back to Tank Settings</a>");
        html += F("</body></html>");
        return html;
    }
    
    String html;
    html.reserve(8000);  // Reserve memory for better performance
    
    html += F("<!DOCTYPE html><html><head>");
    html += F("<title>Tank ");
    html += String(tankId);
    html += F(" Control</title>");
    html += F("<meta name='viewport' content='width=device-width, initial-scale=1'>");
    html += F("<style>");
    html += F("body { font-family: 'Segoe UI', sans-serif; margin: 0; padding: 20px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); min-height: 100vh; }");
    html += F(".container { max-width: 800px; margin: 0 auto; background: rgba(255,255,255,0.95); border-radius: 20px; padding: 30px; }");
    html += F("h1 { color: #2c3e50; text-align: center; }");
    html += F(".status-section { margin: 20px 0; padding: 20px; background: rgba(52, 152, 219, 0.1); border-radius: 15px; }");
    html += F(".btn { padding: 12px 25px; margin: 10px; border: none; border-radius: 8px; cursor: pointer; font-weight: bold; }");
    html += F(".btn-success { background: linear-gradient(135deg, #27ae60, #229954); color: white; }");
    html += F(".btn-danger { background: linear-gradient(135deg, #e74c3c, #c0392b); color: white; }");
    html += F(".btn-info { background: linear-gradient(135deg, #3498db, #2980b9); color: white; }");
    html += F(".btn-warning { background: linear-gradient(135deg, #f39c12, #d68910); color: white; }");
    html += F(".status-item { margin: 15px 0; display: flex; justify-content: space-between; }");
    html += F(".status-label { font-weight: bold; }");
    html += F("select { padding: 8px; border-radius: 5px; margin: 5px; }");
    html += F(".back-link { display: inline-block; margin-bottom: 20px; color: #3498db; text-decoration: none; }");
    html += F(".delay-indicator { padding: 8px 16px; border-radius: 20px; font-weight: bold; text-align: center; margin: 10px; }");
    html += F(".delay-grey { background-color: #95a5a6; color: white; }");
    html += F(".delay-blue { background-color: #3498db; color: white; }");
    html += F("</style>");
    html += F("</head><body>");
    html += F("<div class='container'>");
    html += F("<a href='/tank-settings' class='back-link'>Back to Tank Settings</a>");
    html += F("<h1>Tank ");
    html += String(tankId);
    html += F(" Control Panel</h1>");
    html += F("<div class='status-section'>");
    html += F("<h2>Tank Status</h2>");
    html += F("<div class='status-item'><span class='status-label'>Battery Level:</span><span>");
    html += String(batteryLevels[arrayIndex]);
    html += F("%</span></div>");
    html += F("<div class='status-item'><span class='status-label'>MAC Address:</span><span>");
    html += (tankMACs[arrayIndex] != "" ? tankMACs[arrayIndex] : "Not reported");
    html += F("</span></div>");
    html += F("<div class='status-item'><span class='status-label'>Open or Closed:</span><span>");
    html += String(tankStatus[arrayIndex] ? "open" : "closed");
    html += F("</span></div>");
    html += F("<div class='status-item'><span class='status-label'>Connection:</span><span>Connected</span></div>");
    html += F("</div>");
    html += F("<div class='status-section'>");
    html += F("<h2>Tank Controls</h2>");
    html += F("<button class='btn btn-danger' onclick=\"sendCommand('test_close', ");
    html += String(tankId);
    html += F(")\">Test Close</button>");
    html += F("<button class='btn btn-success' onclick=\"sendCommand('test_open', ");
    html += String(tankId);
    html += F(")\">Test Open</button>");
    html += F("</div>");
    html += F("<div class='status-section'>");
    html += F("<h2>Open Delay Timer</h2>");
    html += F("<label>Open delay: </label>");
    html += F("<select id='delay");
    html += String(tankId);
    html += F("'>");
    
    for (int h = 1; h <= 24; h++) {
        html += F("<option value='");
        html += String(h);
        html += F("'>");
        html += String(h);
        html += F(" hour");
        if (h > 1) html += F("s");
        html += F("</option>");
    }
    
    html += F("</select>");
    html += F("<button class='btn btn-info' onclick=\"startDelay(");
    html += String(tankId);
    html += F(")\">Start delay now</button>");
    html += F("<button class='btn btn-warning' onclick=\"cancelDelay(");
    html += String(tankId);
    html += F(")\">Cancel delay</button>");
    html += F("<div class='delay-indicator ");
    html += String(delayRunning[arrayIndex] ? "delay-blue" : "delay-grey");
    html += F("'>Delay Running: ");
    html += String(delayRunning[arrayIndex] ? "YES" : "NO");
    html += F("</div>");
    html += F("</div>");
    html += F("<div class='status-section'>");
    html += F("<h2>Sleep Timer</h2>");
    html += F("<label>Sleep hours: </label>");
    html += F("<select id='sleep");
    html += String(tankId);
    html += F("'>");
    
    for (int h = 1; h <= 24; h++) {
        html += F("<option value='");
        html += String(h);
        html += F("'>");
        html += String(h);
        html += F(" hour");
        if (h > 1) html += F("s");
        html += F("</option>");
    }
    
    html += F("</select>");
    html += F("<button class='btn btn-info' onclick=\"sendSleep(");
    html += String(tankId);
    html += F(")\">Go sleep now</button>");
    html += F("</div>");
    html += F("<script>");
    html += F("function sendCommand(cmd, tankId) { fetch('/command', { method: 'POST', headers: { 'Content-Type': 'application/x-www-form-urlencoded' }, body: 'command=' + cmd + '&tank=' + tankId }).then(response => response.ok ? alert('Command sent') : alert('Failed')).catch(() => alert('Error')); }");
    html += F("function startDelay(tankId) { var hours = document.getElementById('delay' + tankId).value; fetch('/command', { method: 'POST', headers: { 'Content-Type': 'application/x-www-form-urlencoded' }, body: 'command=start_delay&tank=' + tankId + '&hours=' + hours }).then(response => { if(response.ok) { alert('Delay started'); location.reload(); } }).catch(() => alert('Error')); }");
    html += F("function cancelDelay(tankId) { fetch('/command', { method: 'POST', headers: { 'Content-Type': 'application/x-www-form-urlencoded' }, body: 'command=cancel_delay&tank=' + tankId }).then(response => { if(response.ok) { alert('Delay canceled'); location.reload(); } }).catch(() => alert('Error')); }");
    html += F("function sendSleep(tankId) { var hours = document.getElementById('sleep' + tankId).value; fetch('/command', { method: 'POST', headers: { 'Content-Type': 'application/x-www-form-urlencoded' }, body: 'command=sleep&tank=' + tankId + '&hours=' + hours }).then(response => response.ok ? alert('Sleep sent') : alert('Failed')).catch(() => alert('Error')); }");
    html += F("setInterval(function(){ location.reload(); }, 60000);");
    html += F("</script>");
    html += F("</div></body></html>");
    
    return html;
}

// ===== WEB SERVER SETUP =====

/**
 * Setup web server routes with exception handling
 * Inputs: None
 * Outputs: None (configures AsyncWebServer)
 */
void setupWebServer() {
    Serial.println("FUNCTION: setupWebServer() starting");

    // Default headers for faster navigation
    DefaultHeaders::Instance().addHeader("Connection", "keep-alive");
    // Change to more aggressive anti-cache headers:
    DefaultHeaders::Instance().addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    DefaultHeaders::Instance().addHeader("Pragma", "no-cache");
    DefaultHeaders::Instance().addHeader("Expires", "0");

    // Home page with exception handling
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        try {
            Serial.println("FUNCTION: Serving home page");
            request->send(200, "text/html", generateHomePage());
        } catch (...) {
            Serial.println("EXCEPTION: Error generating home page");
            request->redirect("/");
        }
    });
    
    // Tank settings page
    server.on("/tank-settings", HTTP_GET, [](AsyncWebServerRequest *request) {
        try {
            Serial.println("FUNCTION: Serving tank settings page");
            request->send(200, "text/html", generateTankSettingsPage());
        } catch (...) {
            Serial.println("EXCEPTION: Error generating tank settings");
            request->redirect("/");
        }
    });

    // AJAX Status Endpoint
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) {
        try {
            Serial.println("FUNCTION: Serving status API");
            
            DynamicJsonDocument doc(3072);
            doc["time"] = getCurrentTime();
            JsonArray tanks = doc.createNestedArray("tanks");
            
            for (int i = 0; i < MAX_TANKS; i++) {
                JsonObject tank = tanks.createNestedObject();
                tank["id"] = i + 1;
                tank["connected"] = tankConnected[i];
                tank["sleeping"] = tankInSleep[i];
                tank["timer"] = delayRunning[i];
                tank["battery"] = batteryLevels[i];
                tank["status"] = tankStatus[i];
                
                if (delayRunning[i]) {
                    unsigned long elapsed = millis() - delayStartTime[i];
                    unsigned long remaining = ((unsigned long)delayDuration[i] * 3600000) - elapsed;
                    tank["hoursLeft"] = remaining / 3600000;
                    tank["minutesLeft"] = (remaining % 3600000) / 60000;
                }
                
                if (tankInSleep[i]) {
                    tank["wakeTime"] = getWakeUpTime(i);
                }
            }
            
            String output;
            serializeJson(doc, output);
            request->send(200, "application/json", output);
            
        } catch (...) {
            Serial.println("EXCEPTION: Error generating status API");
            request->send(500, "application/json", "{\"error\":\"Server error\"}");
        }
    });
    
    // Individual tank pages
    for (int i = 1; i <= MAX_TANKS; i++) {
        String route = "/tank" + String(i);
        server.on(route.c_str(), HTTP_GET, [i](AsyncWebServerRequest *request) {
            try {
                Serial.println("FUNCTION: Serving Tank " + String(i) + " page");
                request->send(200, "text/html", generateTankPage(i));
            } catch (...) {
                Serial.println("EXCEPTION: Error generating Tank " + String(i) + " page");
                request->redirect("/");
            }
        });
    }
    
    // Command handler with exception handling
    server.on("/command", HTTP_POST, [](AsyncWebServerRequest *request) {
        try {
            if (request->hasParam("command", true)) {
                String command = request->getParam("command", true)->value();
                int tankId = request->hasParam("tank", true) ? request->getParam("tank", true)->value().toInt() : 0;
                int hours = request->hasParam("hours", true) ? request->getParam("hours", true)->value().toInt() : 0;
                
                Serial.println("FUNCTION: Command received - " + command);
                
                if (command == "open_all") {
                    openAllTanks();
                } else if (command == "close_all") {
                    closeAllTanks();
                } else if (command == "cancel_all_timers") {
                    cancelAllTimedOpenings();
                } else if (command == "test_slack") {
                    sendTestSlackMessage();
                } else if (command == "ping_all") {
                    pingAllTanks();
                } else if (command == "test_close") {
                    testCloseTank(tankId);
                } else if (command == "test_open") {
                    testOpenTank(tankId);
                } else if (command == "start_delay") {
                    startDelayTimer(tankId, hours);
                } else if (command == "cancel_delay") {
                    cancelDelayTimer(tankId);
                } else if (command == "sleep") {
                    sendSleepCommand(tankId, hours);
                } else if (command == "clear_messages") {
                    clearAllErrorMessages();
                }
                
                request->send(200, "text/plain", "OK");
            } else {
                request->send(400, "text/plain", "Missing command");
            }
        } catch (...) {
            Serial.println("EXCEPTION: Error processing command");
            request->send(500, "text/plain", "Server error");
        }
    });
    
    // 404 handler
    server.onNotFound([](AsyncWebServerRequest *request) {
        Serial.println("404: Page not found - " + request->url());
        request->redirect("/");
    });
    
    Serial.println("FUNCTION: setupWebServer() completed");
}

// ===== OLED DISPLAY =====

/**
 * Update OLED display with IP and channel
 * Inputs: None
 * Outputs: None (updates physical display)
 */
// Add this new function before updateDisplay()
/**
 * Check if OLED should be on based on current time
 * Inputs: None
 * Outputs: bool (true if OLED should be on)
 */
bool shouldOLEDBeOn() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        return true;  // Default to on if time unavailable
    }
    
    int currentHour = timeinfo.tm_hour;
    
    // If off hour is later than on hour (normal case: 19 to 6)
    if (OLED_OFF_HOUR > OLED_ON_HOUR) {
        return (currentHour >= OLED_ON_HOUR && currentHour < OLED_OFF_HOUR);
    }
    // If off hour is earlier than on hour (crosses midnight)
    else {
        return (currentHour >= OLED_ON_HOUR || currentHour < OLED_OFF_HOUR);
    }
}

// 
/**
 * Update OLED display with IP and channel (with time-based control)
 * Inputs: None
 * Outputs: None (updates physical display)
 */
/**
 * Update OLED display with IP, channel, and mDNS address (with time-based control)
 * Inputs: None
 * Outputs: None (updates physical display)
 */
void updateDisplay() {
  // Clear the OLED and set up text style
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setTextWrap(true);

  // Start slightly below the top edge
  int y = 8;                 // start one line down (~8 px)
  display.setCursor(0, y);

  // --- IP or WiFi status ---
  if (WiFi.status() == WL_CONNECTED) {
    display.print(F("IP: "));
    display.println(WiFi.localIP());
  } else {
    display.println(F("No WiFi"));
  }

  // Move cursor below last printed line
  y = display.getCursorY() + 1;
  display.setCursor(0, y);

  // --- WiFi channel ---
  display.print(F("CH: "));
  display.println(WiFi.channel());

  // Move cursor again
  y = display.getCursorY() + 1;
  display.setCursor(0, y);

  // --- Hostname / URL (without http://) ---
  //display.print(F("URL: "));
  display.println(F("fishbreedtanks.local"));

  // Move cursor again
  y = display.getCursorY() + 1;

  // --- RSSI (optional, only if space left) ---
  if (y + 8 <= display.height()) {
    display.setCursor(0, y);
    display.print(F("RSSI: "));
    display.print(WiFi.RSSI());
    display.println(F(" dBm"));
  }

  // Update the OLED
  display.display();
}




// ===== MAIN SETUP AND LOOP =====

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("=== Fish Tank Control System v3.6 Starting ===");

    // Initialize OLED with exception handling
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("ERROR: SSD1306 allocation failed"));
    
  display.setTextWrap(true);
} else {
        Serial.println("OLED display initialized");
        display.clearDisplay();
// removed:         display.drawBitmap(32, 0, fish_bmp, 64, 32, SSD1306_WHITE);
        display.display();
    }

    Serial.println("MAC address is: " + WiFi.macAddress());
    
    initWiFi();
    initTime();
    initESPNow();  // This now properly sets and verifies channel
    
    ArduinoOTA.setHostname("fishbreedtanksPrimary");
    ArduinoOTA.begin();
    Serial.println("OTA initialized");
    
    setupWebServer();
    server.begin();
    Serial.println("Web server started");
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("Access interface at: http://");
        Serial.println(WiFi.localIP());
        
        // MOVED HERE: Update display AFTER ESP-NOW is fully initialized
        delay(500);  // Extra delay to ensure channel is stable
        Serial.println("Final channel check: " + String(WiFi.channel()));
        updateDisplay();  // NOW this will show the correct channel
    }

    // In setup(), after initWiFi() and before setupWebServer()
    if (MDNS.begin("fishbreedtanks")) {
        Serial.println("mDNS responder started");
        Serial.println("Access at: http://fishbreedtanks.local");
        MDNS.addService("http", "tcp", 80);
    } else {
        Serial.println("Error setting up mDNS responder");
    }
    
    queueSlackMessage("Fish Tank Control System v3.6 started");
    Serial.println("=== System initialization complete ===");
}

void loop() {
    // v2.7: Non-blocking operations for responsive web interface
    ArduinoOTA.handle();
    
    // WiFi check every 30 seconds (with reconnect protection)
    static unsigned long lastWiFiCheck = 0;
    static bool _reconnecting = false;

    if (millis() - lastWiFiCheck > 30000) {
        lastWiFiCheck = millis();

        if (WiFi.status() != WL_CONNECTED && !_reconnecting) {
            _reconnecting = true;
            Serial.println("WiFi disconnected, attempting reconnect...");
            initWiFi();
            _reconnecting = false;
        }
    }

    // Timer checks every 60 seconds
    static unsigned long lastTimerCheck = 0;
    if (millis() - lastTimerCheck > 60000) {
        checkDelayTimers();
        checkSleepTimers();
        lastTimerCheck = millis();
    }
    
    // Connection status check every 30 seconds
    static unsigned long lastConnectionCheck = 0;
    if (millis() - lastConnectionCheck > 30000) {
        checkConnectionStatus();
        lastConnectionCheck = millis();
    }

    // OLED display refresh every 1 minute (60 seconds)
    static unsigned long lastDisplayUpdate = 0;
    if (millis() - lastDisplayUpdate > 60000) {
        updateDisplay();
        lastDisplayUpdate = millis();
    }

    // Process Slack queue (max 1 per second to avoid rate limits)
    if (!slackQueue.empty() && millis() - lastSlackSend > 1000) {
        String msg = slackQueue.front();
        slackQueue.pop();
        
        HTTPClient http;
        http.setReuse(false);
        http.setTimeout(2000);  // Reduced timeout
        http.begin(slackWebhook);
        http.addHeader("Content-Type", "application/json");
        
        DynamicJsonDocument doc(512);  // Smaller doc
        doc["text"] = msg;
        doc["username"] = "Fish Tank System";
        
        String jsonString;
        serializeJson(doc, jsonString);
        
        http.POST(jsonString);
        http.end();
        
        lastSlackSend = millis();
        Serial.println("SENT Slack: " + msg);
    }

    // v2.7: Minimal delay for responsive web interface
    delay(10);
}
