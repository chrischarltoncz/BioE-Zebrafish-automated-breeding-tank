/*
 * Arduino NANO ESP32-S3 Fish Tank Primary Control System v5.0
 * Manages up to 25 fish breeding tanks via ESP-NOW and web interface
 * 
 * NEW IN v5.3:
 * - more robust coding on the open all/close all tanks on the setting page. Old v5.0 to v5.2 code was fragile and not sure it worked correctly
 *
 * NEW IN v5.2:
 * - slack out battery percentage so the power can be easily tracked
 *
 * NEW IN v5.1:
 * - slack out actual opening clock time rather than 'opening in 1026 minutes' etc
 * 
 * NEW IN v5.0:
 * - Changed timed open and sleep from hours to minutes for precise timing
 * - Fixed home page time for open/sleep rounding minutes
 *
 * NEW IN v4.9:
 * - sync the time and date every 24 hours, check it sync'd correctly
 * - drop downs for sleep and timed open now a time and date instead
 *
 * NEW IN v4.7:
 * - fixed grey boxes during tank sleep state
 * - added delay on open/close buttons to allow the tank to respond, changed the button colors
 * - changed button ghost out on open/close from 5 seconds to 15
 *
 *
 * NEW IN v4.6:
 * - Removed "Sleep after opening" checkbox (now always enabled)
 * - Hard-coded sleep cycle: 4 hours sleep, 5 minutes wake, repeat
 * - Removed toggle_sleep_after_open command handler
 *
 * NEW IN v4.5:
 * - Added bi[o]hub logo to home page top right corner
 * - Updated page title to "Fish breed tank control" with smaller font
 * - Updated browser tab title to match
 * - open/close all tanks now only sends to active connected tanks
 * - fish type text added to each tank for labeling
 *
 * Required Libraries:
 * - ArduinoJson v6.21.3+ : https://github.com/bblanchon/ArduinoJson
 * - AsyncTCP v1.1.4+ : https://github.com/me-no-dev/AsyncTCP
 * - ESPAsyncWebServer v1.2.4+ : https://github.com/me-no-dev/ESPAsyncWebServer
 * - Adafruit GFX Library v1.11.9+ : https://github.com/adafruit/Adafruit-GFX-Library
 * - Adafruit SSD1306 v2.5.9+ : https://github.com/adafruit/Adafruit_SSD1306
 * 
 * Compatible with ESP32 Arduino Core 2.x (tested on 2.0.18)
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

// ===== CONFIGURATION =====
const char* ssid = "BHN-Guest";
const char* password = "N/A";
const char* slackWebhook = "N/A";

uint8_t secondaryMACAddresses[25][6] = {
  {0x30, 0xED, 0xA0, 0xAC, 0xAB, 0xE8}, // secondary1 - present
  {0x94, 0xA9, 0x90, 0x17, 0x84, 0x70}, // secondary2 - present
  {0x30, 0xED, 0xA0, 0xAC, 0xAC, 0xC0}, // secondary3 - present
  {0x30, 0xED, 0xA0, 0xAC, 0xAE, 0x78}, // secondary4 - present
  {0x10, 0xB4, 0x1D, 0xCD, 0x8D, 0x7C}, // secondary5 - present
  {0x10, 0xB4, 0x1D, 0xCD, 0x87, 0x1C}, // secondary6 - present
  {0x10, 0xB4, 0x1D, 0xCD, 0x88, 0x40}, // secondary7 - present
  {0x10, 0xB4, 0x1D, 0xCD, 0x8D, 0x84}, // secondary8 - present
  {0x10, 0xB4, 0x1D, 0xCD, 0x8D, 0x54}, // secondary9 - present
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

bool tankConnected[MAX_TANKS] = {false};
int batteryLevels[MAX_TANKS] = {0};
bool lowBatteryNotified[MAX_TANKS] = {false};
String errorMessages[MAX_TANKS] = {""};
String tankMACs[MAX_TANKS] = {""};
bool tankStatus[MAX_TANKS] = {false};
bool delayRunning[MAX_TANKS] = {false};
uint32_t delayStartTime[MAX_TANKS] = {0};
int delayDuration[MAX_TANKS] = {0};  // Now in MINUTES instead of hours
uint32_t lastHeartbeat[MAX_TANKS] = {0};
bool tankInSleep[MAX_TANKS] = {false};
uint32_t sleepStartTime[MAX_TANKS] = {0};
int sleepDuration[MAX_TANKS] = {0};  // Now in MINUTES instead of hours
String fishType[MAX_TANKS] = {""};

// clock time
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -8 * 3600;
const int daylightOffset_sec = 3600;

// display off times
const int OLED_OFF_HOUR = 19;
const int OLED_ON_HOUR = 6;
bool oledIsOn = true;

// Slack non blocking
#include <queue>
std::queue<String> slackQueue;
uint32_t lastSlackSend = 0;

typedef struct {
    int tankId;
    char command[32];
    int value;
    char message[128];
    int messageType;
} ESPNowMessage;

struct BroadcastResult {
    int attempted;
    int succeeded;
    int failed;
    String failedTanks;
};

// ===== INITIALIZATION =====

void initWiFi() {
    Serial.println("FUNCTION: initWiFi() starting");
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);

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

void initTime() {
    Serial.println("FUNCTION: initTime() starting");

    bool ok = syncTimeFromNtp(10000);  // 10-second timeout

    if (ok) {
        Serial.println("FUNCTION: initTime() completed successfully");
    } else {
        Serial.println("FUNCTION: initTime() completed with ERROR");
    }
}

bool syncTimeFromNtp(uint32_t timeoutMs) {
    Serial.println("Synchronizing time with NTP...");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    struct tm timeinfo;
    uint32_t start = millis();

    while ((millis() - start) < timeoutMs) {
        if (getLocalTime(&timeinfo)) {
            // Basic sanity check: make sure the year is reasonable
            int year = timeinfo.tm_year + 1900;
            if (year > 2020) {
                Serial.print("Time sync OK: ");
                Serial.println(&timeinfo, "%F %T");
                return true;
            }
        }
        delay(200);
    }

    Serial.println("Time sync FAILED (timeout or invalid time)");
    return false;
}

void addESPNowPeers() {
    Serial.println("FUNCTION: addESPNowPeers() starting");
    int currentChannel = WiFi.channel();
    
    for (int i = 0; i < MAX_TANKS; i++) {
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, secondaryMACAddresses[i], 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        
        if (esp_now_add_peer(&peerInfo) == ESP_OK) {
            Serial.println("Added Tank " + String(i + 1) + " as peer");
        } else {
            Serial.println("ERROR: Failed to add Tank " + String(i + 1));
        }
    }
    Serial.println("FUNCTION: addESPNowPeers() completed");
}

void initESPNow() {
    Serial.println("FUNCTION: initESPNow() starting");
    
    int currentChannel = WiFi.channel();
    Serial.println("WiFi channel before ESP-NOW: " + String(currentChannel));
    
    WiFi.mode(WIFI_STA);
    delay(500);
    
    int verifiedChannel = WiFi.channel();
    Serial.println("WiFi channel after mode switch: " + String(verifiedChannel));
    
    if (verifiedChannel != currentChannel) {
        Serial.println("WARNING: Channel mismatch detected! Forcing again...");
        delay(500);
        verifiedChannel = WiFi.channel();
        Serial.println("WiFi channel after second force: " + String(verifiedChannel));
    }

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
    
    esp_now_register_send_cb(onESPNowDataSent);
    esp_now_register_recv_cb(onESPNowDataReceived);
    
    addESPNowPeers();
    Serial.println("FUNCTION: initESPNow() completed successfully");
}

// ===== TIME FUNCTIONS =====
String getCurrentTime() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        return "Time unavailable";
    }
    
    char timeStr[100];
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S PST", &timeinfo);
    return String(timeStr);
}

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
    uint32_t elapsed = millis() - sleepStartTime[arrayIndex];
    uint32_t remaining = ((uint32_t)sleepDuration[arrayIndex] * 60000) - elapsed;  // Changed to minutes
    time_t wakeTime = now + (remaining / 1000);
    
    struct tm* wakeTimeInfo = localtime(&wakeTime);
    
    char timeStr[20];
    strftime(timeStr, sizeof(timeStr), "%I:%M %p PST", wakeTimeInfo);
    return String(timeStr);
}

String getTimedOpenTime(int arrayIndex) {
    if (!delayRunning[arrayIndex]) {
        return "";
    }
    
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        return "Time unavailable";
    }
    
    time_t now;
    time(&now);
    uint32_t elapsed = millis() - delayStartTime[arrayIndex];
    uint32_t remaining = ((uint32_t)delayDuration[arrayIndex] * 60000) - elapsed;  // Changed to minutes
    time_t openTime = now + (remaining / 1000);
    
    struct tm* openTimeInfo = localtime(&openTime);
    
    char timeStr[20];
    strftime(timeStr, sizeof(timeStr), "%I:%M %p PST", openTimeInfo);
    return String(timeStr);
}

static volatile bool _sendDone = false;
static volatile bool _sendOk  = false;

void onESPNowDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    _sendOk   = (status == ESP_NOW_SEND_SUCCESS);
    _sendDone = true;
}

void onESPNowDataReceived(const uint8_t *mac, const uint8_t *incomingData, int len) {
    ESPNowMessage message;
    memcpy(&message, incomingData, sizeof(message));
    
    int tankId = message.tankId;
    if (tankId < 1 || tankId > MAX_TANKS) {
        Serial.println("ERROR: Invalid tank ID received: " + String(tankId));
        return;
    }
    
    int arrayIndex = tankId - 1;
    
    // CHANGED: Don't automatically mark as connected or clear sleep state
    // Only update heartbeat timestamp
    lastHeartbeat[arrayIndex] = millis();
    
    // CHANGED: Only mark as connected if NOT in manual sleep mode
    if (!tankInSleep[arrayIndex]) {
        tankConnected[arrayIndex] = true;
    }
    
    Serial.println("FUNCTION: onESPNowDataReceived() - Tank " + String(tankId) + " type " + String(message.messageType));
    
    switch (message.messageType) {
        case 1:
            Serial.println("Status update from Tank " + String(tankId));
            if (String(message.message) == "timed_open_executed") {
                Serial.println("Tank " + String(tankId) + " completed timed opening");
                queueSlackMessage("Tank" + String(tankId) + " completed its timed opening");
                delayRunning[arrayIndex] = false;
                delayStartTime[arrayIndex] = 0;
                delayDuration[arrayIndex] = 0;
            }
            break;
            
        case 2:
            errorMessages[arrayIndex] = String(message.message);
            Serial.println("ERROR: Tank " + String(tankId) + ": " + String(message.message));
            break;
            
        case 3:
            batteryLevels[arrayIndex] = message.value;
            Serial.println("Tank " + String(tankId) + " battery: " + String(message.value) + "%");
            if (message.value < 20 && !lowBatteryNotified[arrayIndex]) {
                queueSlackMessage("Tank" + String(tankId) + " battery low (" + String(message.value) + "%)");
                lowBatteryNotified[arrayIndex] = true;
            } else if (message.value >= 25 && lowBatteryNotified[arrayIndex]) {
                lowBatteryNotified[arrayIndex] = false;
            }
            break;
            
        case 4:
            tankMACs[arrayIndex] = String(message.message);
            Serial.println("Tank " + String(tankId) + " MAC: " + String(message.message));
            break;
            
        case 5:
            {
                bool prev = tankStatus[arrayIndex];
                tankStatus[arrayIndex] = (message.value == 1);
                Serial.println("Tank " + String(tankId) + " status: " + (tankStatus[arrayIndex] ? "Open" : "Closed"));
                if (tankStatus[arrayIndex] != prev) {
                    if (tankStatus[arrayIndex]) {
                        queueSlackMessage("Tank" + String(tankId) + " opened the slot/door and entered sleep cycle (battery: " + String(batteryLevels[arrayIndex]) + "%)");
                    }
                }
            }
            break;
            
        case 6:
            break;
            
        default:
            Serial.println("WARNING: Unknown message type: " + String(message.messageType));
            break;
    }
}

// ===== ESP-NOW COMMANDS =====
bool sendCommandToTankReliable(int tankId, const String &command, int value = 0, int maxRetries = 3) {
    if (tankId < 1 || tankId > MAX_TANKS) return false;

    Serial.println("FUNCTION: sendCommandToTankReliable() - Tank " + String(tankId) + " cmd: " + command);

    ESPNowMessage message;
    message.tankId = tankId;
    strncpy(message.command, command.c_str(), sizeof(message.command) - 1);
    message.command[sizeof(message.command) - 1] = '\0';
    message.value = value;
    message.messageType = 0;

    int arrayIndex = tankId - 1;

    for (int attempt = 0; attempt <= maxRetries; attempt++) {
        _sendDone = false;
        _sendOk   = false;

        esp_err_t result = esp_now_send(secondaryMACAddresses[arrayIndex],
                                        (uint8_t *)&message, sizeof(message));
        if (result != ESP_OK) {
            Serial.println("ERROR: esp_now_send() failed for Tank " + String(tankId) +
                           " attempt " + String(attempt + 1));
            delay(100);
            continue;
        }

        // Wait for the send callback (MAC-layer ACK or timeout)
        uint32_t t0 = millis();
        while (!_sendDone && (millis() - t0 < 300)) {
            delay(1);
        }

        if (_sendDone && _sendOk) {
            if (attempt > 0) {
                Serial.println("Tank " + String(tankId) + " send OK on retry " + String(attempt));
            }
            return true;
        }

        Serial.println("WARNING: Tank " + String(tankId) + " send attempt " +
                        String(attempt + 1) + " failed/timed-out");
        // Progressively longer backoff between retries
        delay(100 * (attempt + 1));
    }

    Serial.println("ERROR: All retries exhausted for Tank " + String(tankId));
    return false;
}

// Wrapper that keeps the old function signature so nothing else breaks
void sendCommandToTank(int tankId, String command, int value = 0) {
    sendCommandToTankReliable(tankId, command, value, 0);
}

BroadcastResult sendCommandToAllTanksReliable(const String &command) {
    Serial.println("FUNCTION: sendCommandToAllTanksReliable() - " + command);

    BroadcastResult result = {0, 0, 0, ""};

    for (int i = 0; i < MAX_TANKS; i++) {
        if (!tankConnected[i]) continue;

        int tankId = i + 1;
        result.attempted++;

        if (sendCommandToTankReliable(tankId, command, 0, 3)) {
            result.succeeded++;
        } else {
            result.failed++;
            if (result.failedTanks.length() > 0) result.failedTanks += ", ";
            result.failedTanks += String(tankId);
        }

        // Give the receiving tank time to process before sending to the next one
        delay(200);
    }

    Serial.println("Broadcast '" + command + "': " +
                    String(result.succeeded) + "/" + String(result.attempted) +
                    " succeeded");

    if (result.failed > 0) {
        Serial.println("Failed tanks: " + result.failedTanks);
        queueSlackMessage("Broadcast '" + command + "' failed for tank(s): " + result.failedTanks);
    }

    return result;
}

void openAllTanks() {
    Serial.println("FUNCTION: openAllTanks() executed");
    BroadcastResult r = sendCommandToAllTanksReliable("open_slot");
    if (r.attempted == 0) {
        queueSlackMessage("Open all: no connected tanks to send to");
    }
}

void closeAllTanks() {
    Serial.println("FUNCTION: closeAllTanks() executed");
    BroadcastResult r = sendCommandToAllTanksReliable("close_slot");
    if (r.attempted == 0) {
        queueSlackMessage("Close all: no connected tanks to send to");
    }
}

void testCloseTank(int tankId) {
    Serial.println("FUNCTION: testCloseTank() - Tank " + String(tankId));
    sendCommandToTank(tankId, "close_slot");
}

void testOpenTank(int tankId) {
    Serial.println("FUNCTION: testOpenTank() - Tank " + String(tankId));
    sendCommandToTank(tankId, "open_slot");
}

void pingAllTanks() {
    Serial.println("FUNCTION: pingAllTanks() executed");
    sendCommandToAllTanksReliable("ping_test");
}

// start delay timer, new version v5.1
void startDelayTimer(int tankId, int minutes) {
    if (tankId < 1 || tankId > MAX_TANKS || minutes < 1 || minutes > 7200) {  // Max 5 days in minutes
        Serial.println("ERROR: Invalid delay parameters");
        return;
    }
    
    Serial.println("FUNCTION: startDelayTimer() - Tank " + String(tankId) + " for " + String(minutes) + " minutes");
    
    int arrayIndex = tankId - 1;
    
    delayRunning[arrayIndex] = true;
    delayStartTime[arrayIndex] = millis();
    delayDuration[arrayIndex] = minutes;
    
    sendCommandToTank(tankId, "start_delay", minutes);
    queueSlackMessage("Tank" + String(tankId) + " has timed open set for " + getFutureTimeString(minutes));
    
    Serial.println("DEBUG: Tank " + String(tankId) + " timer tracking started, tank will sleep");
}

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

void sendSleepCommand(int tankId, int minutes) {
    if (tankId < 1 || tankId > MAX_TANKS || minutes < 1 || minutes > 7200) {  // Max 5 days in minutes
        Serial.println("ERROR: Invalid sleep parameters");
        return;
    }
    
    Serial.println("FUNCTION: sendSleepCommand() - Tank " + String(tankId) + " for " + String(minutes) + " minutes");
    
    int arrayIndex = tankId - 1;
    
    tankInSleep[arrayIndex] = true;
    sleepStartTime[arrayIndex] = millis();
    sleepDuration[arrayIndex] = minutes;
    tankConnected[arrayIndex] = false;
    
    sendCommandToTank(tankId, "go_sleep", minutes);
    queueSlackMessage("Tank" + String(tankId) + " has gone into low power sleep until " + getFutureTimeString(minutes));
}

void clearAllErrorMessages() {
    Serial.println("FUNCTION: clearAllErrorMessages() executed");
    for (int i = 0; i < MAX_TANKS; i++) {
        errorMessages[i] = "";
    }
}

void checkDelayTimers() {
    for (int i = 0; i < MAX_TANKS; i++) {
        if (delayRunning[i]) {
            uint32_t elapsed = millis() - delayStartTime[i];
            uint32_t duration = (uint32_t)delayDuration[i] * 60000;  // Changed to minutes
            
            if (elapsed >= duration) {
                uint32_t gracePeriod = 300000;
                if (elapsed >= duration + gracePeriod) {
                    delayRunning[i] = false;
                    Serial.println("Delay timer expired for Tank " + String(i + 1) + " (grace period exceeded, no confirmation)");
                    queueSlackMessage("Tank" + String(i + 1) + " timer expired but no confirmation received (battery: " + String(batteryLevels[i]) + "%)");
                }
            }
        }
    }
}

void checkSleepTimers() {
    for (int i = 0; i < MAX_TANKS; i++) {
        if (tankInSleep[i]) {
            uint32_t elapsed = millis() - sleepStartTime[i];
            uint32_t duration = (uint32_t)sleepDuration[i] * 60000;  // Changed to minutes
            
            if (elapsed >= duration) {
                tankInSleep[i] = false;
                sleepStartTime[i] = 0;
                sleepDuration[i] = 0;
                tankConnected[i] = true;
                Serial.println("Sleep period ended for Tank " + String(i + 1));
                queueSlackMessage("Tank" + String(i + 1) + " awoke from sleep (battery: " + String(batteryLevels[i]) + "%)");
            }
        }
    }
}

void checkConnectionStatus() {
    uint32_t currentTime = millis();
    for (int i = 0; i < MAX_TANKS; i++) {
        if (delayRunning[i] || tankInSleep[i]) {
            continue;
        }
        
        if (tankConnected[i] && (currentTime - lastHeartbeat[i] > 180000)) {
            tankConnected[i] = false;
            Serial.println("Tank " + String(i + 1) + " connection timeout");
        }
    }
}

// Helper function to get a clock time string for "now + N minutes"
String getFutureTimeString(int minutesFromNow) {
    time_t now;
    time(&now);
    time_t future = now + (minutesFromNow * 60);
    struct tm* futureTime = localtime(&future);
    
    char timeStr[30];
    strftime(timeStr, sizeof(timeStr), "%I:%M %p PST on %m/%d", futureTime);
    return String(timeStr);
}

// ===== SLACK =====
void queueSlackMessage(String message) {
    slackQueue.push(message);
    Serial.println("QUEUED Slack: " + message);
}

void sendTestSlackMessage() {
    Serial.println("FUNCTION: sendTestSlackMessage() executed");
    queueSlackMessage("Testing Slack notification");
}

// ===== WEB PAGE GENERATION =====
String getTankDisplayInfo(int arrayIndex) {
    if (delayRunning[arrayIndex]) {
        return "tank-timer|Timer Active";
    }
    
    if (tankInSleep[arrayIndex]) {
        return "tank-sleeping|Sleeping";
    }
    
    if (!tankConnected[arrayIndex]) {
        return "tank-disconnected|Disconnected";
    }
    
    return "tank-connected|Connected";
}

String generateHomePage() {
    String currentTime = getCurrentTime();

    String html;
    html.reserve(14000);

    html += F("<!DOCTYPE html><html><head>");
    html += F("<title>Fish breed tank control v5.0</title>");
    html += F("<meta name='viewport' content='width=device-width, initial-scale=1'>");
    html += F("<style>");
    html += F("body { font-family: 'Segoe UI', sans-serif; margin: 0; padding: 20px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); min-height: 100vh; }");
    html += F(".container { max-width: 1200px; margin: 0 auto; background: rgba(255,255,255,0.95); border-radius: 20px; padding: 30px; box-shadow: 0 20px 40px rgba(0,0,0,0.1); position: relative; }");
    html += F(".logo { position: absolute; top: 20px; right: 5px; width: 250px; height: auto; font-family: 'Arial Black', sans-serif; font-size: 48px; font-weight: 900; color: #000; letter-spacing: -2px; line-height: 1; }");
    html += F(".logo .brackets { font-weight: 900; }");
    html += F("h1 { color: #2c3e50; text-align: center; font-size: 1.8rem; margin-bottom: 10px; }");
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
    html += F("<div class='logo'>bi<span class='brackets'>[</span>o<span class='brackets'>]</span>hub</div>");
    html += F("<h1>Fish breed tank control v5.0</h1>");
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
        
        if (fishType[i] != "") {
            html += "<br><small style='font-style: italic;'>(" + fishType[i] + ")</small>";
        }
        
        if (delayRunning[i]) {
            String openTime = getTimedOpenTime(i);
            html += "<br><small>Open: " + openTime + "</small>";
        } else if (tankInSleep[i]) {
            String wakeTime = getWakeUpTime(i);
            html += "<br><small>Wake: " + wakeTime + "</small>";
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

String generateTankSettingsPage() {
    String html;
    html.reserve(18000);
    
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
    
    html += F("<div class='control-section'>");
    html += F("<h2>Global Tank Controls</h2>");
    html += F("<button class='btn btn-success' onclick=\"sendCommand('open_all')\">Open all tanks</button>");
    html += F("<button class='btn btn-danger' onclick=\"sendCommand('close_all')\">Close all tanks</button>");
    html += F("<button class='btn btn-info' onclick=\"sendCommand('test_slack')\">Test Slack Notify</button>");
    html += F("<button class='btn btn-secondary' onclick=\"sendCommand('ping_all')\">Ping All Tanks</button>");
    html += F("</div>");
    
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
    
    html += F("<div class='control-section'>");
    html += F("<h2>Individual Tank Controls</h2>");
    html += F("<div class='tank-grid'>");
    
    for (int i = 1; i <= MAX_TANKS; i++) {
        int arrayIndex = i - 1;
        String displayInfo = getTankDisplayInfo(arrayIndex);
        int separatorPos = displayInfo.indexOf("|");
        String statusText = displayInfo.substring(separatorPos + 1);
        
        String boxClass = "disconnected";
        if (tankInSleep[arrayIndex]) {
            boxClass = "sleeping";
        } else if (delayRunning[arrayIndex]) {
            boxClass = "timer";
        } else if (tankConnected[arrayIndex]) {
            boxClass = "connected";
        }
        
        if (tankConnected[arrayIndex] || tankInSleep[arrayIndex] || delayRunning[arrayIndex]) {
            html += "<a href='/tank" + String(i) + "' class='tank-box " + boxClass + "'>";
        } else {
            html += "<div class='tank-box " + boxClass + "'>";
        }
        
        html += "Tank" + String(i) + "<br><small>" + statusText + "</small>";
        
        if (tankConnected[arrayIndex] || tankInSleep[arrayIndex] || delayRunning[arrayIndex]) {
            html += F("</a>");
        } else {
            html += F("</div>");
        }
    }
    
    html += F("</div></div>");
    
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
    
    if (!tankConnected[arrayIndex] && !delayRunning[arrayIndex] && !tankInSleep[arrayIndex]) {
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
    html.reserve(9000);
    
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
    html += F(".btn:disabled { opacity: 0.5; cursor: not-allowed; }");
    html += F(".status-item { margin: 15px 0; display: flex; justify-content: space-between; }");
    html += F(".status-label { font-weight: bold; }");
    html += F("select, .datetime-input { padding: 8px; border-radius: 5px; margin: 5px; }");
    html += F(".back-link { display: inline-block; margin-bottom: 20px; color: #3498db; text-decoration: none; }");
    html += F(".delay-indicator { padding: 8px 16px; border-radius: 20px; font-weight: bold; text-align: center; margin: 10px; }");
    html += F(".delay-grey { background-color: #95a5a6; color: white; }");
    html += F(".delay-blue { background-color: #3498db; color: white; }");
    html += F(".info-box { padding: 15px; background: rgba(241, 196, 15, 0.2); border-left: 4px solid #f39c12; border-radius: 8px; margin: 15px 0; }");
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
    
    html += F("<div class='status-item'><span class='status-label'>Connection:</span><span>");
    if (delayRunning[arrayIndex]) {
        html += F("Timer Running (Sleeping)");
    } else if (tankInSleep[arrayIndex]) {
        html += F("In Sleep Mode");
    } else if (tankConnected[arrayIndex]) {
        html += F("Connected");
    } else {
        html += F("Disconnected");
    }
    html += F("</span></div>");
    
    html += F("</div>");
    html += F("<div class='status-section'>");
    html += F("<h2>Fish Type</h2>");
    html += F("<label>Fish type (max 20 characters): </label>");
    html += F("<input type='text' id='fishType");
    html += String(tankId);
    html += F("' maxlength='20' value='");
    html += fishType[arrayIndex];
    html += F("' style='padding: 8px; border-radius: 5px; border: 1px solid #ccc; width: 200px;'>");
    html += F("<button class='btn btn-info' onclick=\"saveFishType(");
    html += String(tankId);
    html += F(")\">Save Fish Type</button>");
    html += F("</div>");
    html += F("<div class='status-section'>");
    html += F("<h2>Tank Controls</h2>");
    html += F("<div class='info-box'>");
    html += F("<strong>Auto Sleep Cycle:</strong> After opening, tank automatically enters a power-saving cycle: ");
    html += F("4 hours sleep -> 5 minutes awake -> repeat");
    html += F("</div>");
    html += F("<button id='btnClose' class='btn btn-info' onclick=\"sendCommand('test_close', ");
    html += String(tankId);
    html += F(")\">Test Close</button>");
    html += F("<button id='btnOpen' class='btn btn-info' onclick=\"sendCommand('test_open', ");
    html += String(tankId);
    html += F(")\">Test Open</button>");
    html += F("</div>");
    html += F("<div class='status-section'>");
    html += F("<h2>Open Delay Timer</h2>");
    html += F("<p style='color: #7f8c8d; font-size: 0.9em;'>Note: Tank enters sleep mode immediately after starting timer and cannot be cancelled.</p>");
    html += F("<label>Open at (date & time): </label>");
    html += F("<input type='datetime-local' id='delay");
    html += String(tankId);
    html += F("' class='datetime-input'>");
    html += F("<button class='btn btn-info' onclick=\"startDelay(");
    html += String(tankId);
    html += F(")\">Start delay now</button>");
    html += F("<div class='delay-indicator ");
    html += String(delayRunning[arrayIndex] ? "delay-blue" : "delay-grey");
    html += F("'>Delay Running: ");
    html += String(delayRunning[arrayIndex] ? "YES" : "NO");
    html += F("</div>");
    html += F("</div>");
    html += F("<div class='status-section'>");
    html += F("<h2>Sleep Timer</h2>");
    html += F("<label>Sleep until (date & time): </label>");
    html += F("<input type='datetime-local' id='sleep");
    html += String(tankId);
    html += F("' class='datetime-input'>");
    html += F("<button class='btn btn-info' onclick=\"sendSleep(");
    html += String(tankId);
    html += F(")\">Go sleep now</button>");
    html += F("</div>");
    html += F("<script>");
    html += F("function sendCommand(cmd, tankId) { ");
    html += F("var btnClose = document.getElementById('btnClose'); ");
    html += F("var btnOpen = document.getElementById('btnOpen'); ");
    html += F("btnClose.disabled = true; ");
    html += F("btnOpen.disabled = true; ");
    html += F("fetch('/command', { method: 'POST', headers: { 'Content-Type': 'application/x-www-form-urlencoded' }, body: 'command=' + cmd + '&tank=' + tankId }).then(response => response.ok ? alert('Command sent') : alert('Failed')).catch(() => alert('Error')); ");
    html += F("setTimeout(function() { btnClose.disabled = false; btnOpen.disabled = false; }, 16500); ");
    html += F("}");
    html += F("function startDelay(tankId) { var input = document.getElementById('delay' + tankId); if (!input || !input.value) { alert('Please select a date and time for the delay.'); return; } var target = new Date(input.value); var now = new Date(); var diffMs = target.getTime() - now.getTime(); if (diffMs <= 0) { alert('Selected time must be in the future.'); return; } var minutes = Math.max(1, Math.ceil(diffMs / (1000 * 60))); fetch('/command', { method: 'POST', headers: { 'Content-Type': 'application/x-www-form-urlencoded' }, body: 'command=start_delay&tank=' + tankId + '&minutes=' + minutes }).then(response => { if (response.ok) { alert('Delay started'); location.reload(); } else { alert('Failed'); } }).catch(() => alert('Error')); }");
    html += F("function sendSleep(tankId) { var input = document.getElementById('sleep' + tankId); if (!input || !input.value) { alert('Please select a date and time for sleep.'); return; } var target = new Date(input.value); var now = new Date(); var diffMs = target.getTime() - now.getTime(); if (diffMs <= 0) { alert('Selected time must be in the future.'); return; } var minutes = Math.max(1, Math.ceil(diffMs / (1000 * 60))); fetch('/command', { method: 'POST', headers: { 'Content-Type': 'application/x-www-form-urlencoded' }, body: 'command=sleep&tank=' + tankId + '&minutes=' + minutes }).then(response => { if (response.ok) { alert('Sleep sent'); location.reload(); } else { alert('Failed'); } }).catch(() => alert('Error')); }");
    html += F("function saveFishType(tankId) { var fishType = document.getElementById('fishType' + tankId).value; fetch('/command', { method: 'POST', headers: { 'Content-Type': 'application/x-www-form-urlencoded' }, body: 'command=save_fish_type&tank=' + tankId + '&fishtype=' + encodeURIComponent(fishType) }).then(response => { if(response.ok) { alert('Fish type saved'); } else { alert('Failed to save'); } }).catch(() => alert('Error')); }");
    html += F("setInterval(function(){ location.reload(); }, 60000);");
    html += F("</script>");
    html += F("</div></body></html>");
    
    return html;
}

// ===== WEB SERVER SETUP =====
void setupWebServer() {
    Serial.println("FUNCTION: setupWebServer() starting");

    DefaultHeaders::Instance().addHeader("Connection", "keep-alive");
    DefaultHeaders::Instance().addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    DefaultHeaders::Instance().addHeader("Pragma", "no-cache");
    DefaultHeaders::Instance().addHeader("Expires", "0");

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        try {
            Serial.println("FUNCTION: Serving home page");
            request->send(200, "text/html", generateHomePage());
        } catch (...) {
            Serial.println("EXCEPTION: Error generating home page");
            request->redirect("/");
        }
    });
    
    server.on("/tank-settings", HTTP_GET, [](AsyncWebServerRequest *request) {
        try {
            Serial.println("FUNCTION: Serving tank settings page");
            request->send(200, "text/html", generateTankSettingsPage());
        } catch (...) {
            Serial.println("EXCEPTION: Error generating tank settings");
            request->redirect("/");
        }
    });

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
                    uint32_t elapsed = millis() - delayStartTime[i];
                    uint32_t remaining = ((uint32_t)delayDuration[i] * 60000) - elapsed;  // Changed to minutes
                    tank["minutesLeft"] = remaining / 60000;
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
    
    server.on("/command", HTTP_POST, [](AsyncWebServerRequest *request) {
        try {
            if (request->hasParam("command", true)) {
                String command = request->getParam("command", true)->value();
                int tankId = request->hasParam("tank", true) ? request->getParam("tank", true)->value().toInt() : 0;
                int minutes = request->hasParam("minutes", true) ? request->getParam("minutes", true)->value().toInt() : 0;
                
                Serial.println("FUNCTION: Command received - " + command);
                
                if (command == "open_all") {
                    openAllTanks();
                } else if (command == "close_all") {
                    closeAllTanks();
                } else if (command == "test_slack") {
                    sendTestSlackMessage();
                } else if (command == "ping_all") {
                    pingAllTanks();
                } else if (command == "test_close") {
                    testCloseTank(tankId);
                } else if (command == "test_open") {
                    testOpenTank(tankId);
                } else if (command == "start_delay") {
                    startDelayTimer(tankId, minutes);
                } else if (command == "sleep") {
                    sendSleepCommand(tankId, minutes);
                } else if (command == "clear_messages") {
                    clearAllErrorMessages();
                } else if (command == "save_fish_type") {
                    if (tankId >= 1 && tankId <= MAX_TANKS && request->hasParam("fishtype", true)) {
                        int arrayIndex = tankId - 1;
                        String newFishType = request->getParam("fishtype", true)->value();
                        newFishType.trim();
                        if (newFishType.length() > 20) {
                            newFishType = newFishType.substring(0, 20);
                        }
                        fishType[arrayIndex] = newFishType;
                        Serial.println("Tank " + String(tankId) + " fish type set to: " + fishType[arrayIndex]);
                    }
                }
                // Note: Removed toggle_sleep_after_open handler - feature is now always enabled
                
                request->send(200, "text/plain", "OK");
            } else {
                request->send(400, "text/plain", "Missing command");
            }
        } catch (...) {
            Serial.println("EXCEPTION: Error processing command");
            request->send(500, "text/plain", "Server error");
        }
    });
    
    server.onNotFound([](AsyncWebServerRequest *request) {
        Serial.println("404: Page not found - " + request->url());
        request->redirect("/");
    });
    
    Serial.println("FUNCTION: setupWebServer() completed");
}

// ===== OLED DISPLAY =====
bool shouldOLEDBeOn() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        return true;
    }
    
    int currentHour = timeinfo.tm_hour;
    
    if (OLED_OFF_HOUR > OLED_ON_HOUR) {
        return (currentHour >= OLED_ON_HOUR && currentHour < OLED_OFF_HOUR);
    }
    else {
        return (currentHour >= OLED_ON_HOUR || currentHour < OLED_OFF_HOUR);
    }
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setTextWrap(true);

  int y = 8;
  display.setCursor(0, y);

  if (WiFi.status() == WL_CONNECTED) {
    display.print(F("IP: "));
    display.println(WiFi.localIP());
  } else {
    display.println(F("No WiFi"));
  }

  y = display.getCursorY() + 1;
  display.setCursor(0, y);

  display.print(F("CH: "));
  display.println(WiFi.channel());

  y = display.getCursorY() + 1;
  display.setCursor(0, y);

  display.println(F("fishbreedtanks.local"));

  y = display.getCursorY() + 1;

  if (y + 8 <= display.height()) {
    display.setCursor(0, y);
    display.print(F("RSSI: "));
    display.print(WiFi.RSSI());
    display.println(F(" dBm"));
  }

  display.display();
}

// ===== MAIN SETUP AND LOOP =====
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("=== Fish Tank Control System v5.0 Starting ===");

    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("ERROR: SSD1306 allocation failed"));
        display.setTextWrap(true);
    } else {
        Serial.println("OLED display initialized");
        display.clearDisplay();
        display.display();
    }

    Serial.println("MAC address is: " + WiFi.macAddress());
    
    initWiFi();
    initTime();
    initESPNow();
    
    ArduinoOTA.setHostname("fishbreedtanksPrimary");
    ArduinoOTA.begin();
    Serial.println("OTA initialized");
    
    setupWebServer();
    server.begin();
    Serial.println("Web server started");
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("Access interface at: http://");
        Serial.println(WiFi.localIP());
        
        delay(500);
        Serial.println("Final channel check: " + String(WiFi.channel()));
        updateDisplay();
    }

    if (MDNS.begin("fishbreedtanks")) {
        Serial.println("mDNS responder started");
        Serial.println("Access at: http://fishbreedtanks.local");
        MDNS.addService("http", "tcp", 80);
    } else {
        Serial.println("Error setting up mDNS responder");
    }
    
    queueSlackMessage("Fish Tank Control System v5.0 started");
    Serial.println("=== System initialization complete ===");
}

void loop() {
    static unsigned long lastTimeSyncMillis = 0;
    const unsigned long TIME_SYNC_INTERVAL_MS = 24UL * 60UL * 60UL * 1000UL; // 24 hours

    // Periodic 24-hour NTP resync
    if (millis() - lastTimeSyncMillis >= TIME_SYNC_INTERVAL_MS) {
        if (WiFi.status() == WL_CONNECTED) {
            if (syncTimeFromNtp(10000)) {
                lastTimeSyncMillis = millis();
            } else {
                Serial.println("Periodic time resync FAILED");
            }
        } else {
            Serial.println("Skipping time resync: WiFi not connected");
        }
    }

    ArduinoOTA.handle();
    
    static uint32_t lastWiFiCheck = 0;
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

    static uint32_t lastTimerCheck = 0;
    if (millis() - lastTimerCheck > 60000) {
        checkDelayTimers();
        checkSleepTimers();
        lastTimerCheck = millis();
    }
    
    static uint32_t lastConnectionCheck = 0;
    if (millis() - lastConnectionCheck > 30000) {
        checkConnectionStatus();
        lastConnectionCheck = millis();
    }

    static uint32_t lastDisplayUpdate = 0;
    if (millis() - lastDisplayUpdate > 60000) {
        updateDisplay();
        lastDisplayUpdate = millis();
    }

    if (!slackQueue.empty() && millis() - lastSlackSend > 1000) {
        String msg = slackQueue.front();
        slackQueue.pop();
        
        HTTPClient http;
        http.setReuse(false);
        http.setTimeout(2000);
        http.begin(slackWebhook);
        http.addHeader("Content-Type", "application/json");
        
        DynamicJsonDocument doc(512);
        doc["text"] = msg;
        doc["username"] = "Fish Tank System";
        
        String jsonString;
        serializeJson(doc, jsonString);
        
        http.POST(jsonString);
        http.end();
        
        lastSlackSend = millis();
        Serial.println("SENT Slack: " + msg);
    }

    delay(10);
}