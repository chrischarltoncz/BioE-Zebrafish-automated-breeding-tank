/*
 * ESP32-S3 Fish Tank Secondary Control System v2.8
 * Individual tank controller that communicates with Primary via ESP-NOW
 * 
 * CHANGES in v2.9:
 * - Fixed compilation errors (duplicate code blocks, extra braces)
 * - Channel discovery, pings Primary MAC on different channels to find the Primary
 * - V3.0
 * - fixed motor reverse polarity error
 * 
 * Built-in ESP32 Libraries Used:
 * - WiFi, esp_now, esp_sleep, esp_wifi
 * 
 * Folder Structure:
 * 
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_sleep.h>
#include <esp_wifi.h>

// ===== CONFIGURATION - MODIFY FOR EACH DEVICE =====
const String TANK_ID_STRING = "Tank1";           // Set this to Tank1, Tank2, etc.
const int TANK_ID_NUMBER = 1;                    // Numeric ID (1-25) matching string above

// Primary WiFi SSID to scan for channel detection
const char* primarySSID = "BHN-Guest";

// Hardcoded primary MAC address (UPDATE WITH ACTUAL PRIMARY MAC)
uint8_t primaryMACAddress[6] = {0x94, 0xA9, 0x90, 0x17, 0x76, 0xD8};

// === Probe state for channel discovery ===
volatile bool __probeWaiting = false;
volatile bool __probeSuccess = false;
bool __espNowReady = false;

// GPIO Pin Configuration
const int MOTOR_PWR_PIN = D2;                    // GPIO 05 (D2) - Motor power control
const int MOTOR_SLEEP_PIN = D7;                  // GPIO 10 (D7) - Motor control sleep pin
const int MOTOR_DIR_PIN = D10;                   // GPIO 21 (D10) - Motor direction control  
const int OPTO_SLEEP_PIN = D11;                  // GPIO 14 (D11) - Opto sensor LED sleep pin
const int DOOR_SENSOR_PIN = D9;                  // GPIO 18 (D9) - Door open sensor
const int BATTERY_ADC_PIN = A3;                  // GPIO 22 (A3) - Battery voltage ADC v2.5 used to be 22, now A3
const int MOTOR_ADC_CURRENT_PIN = A1;            // GPIO 02 (A1) motor current value across the shunt
const int LED_READY = 13;                        // this is ON when the unit is ready or normally powered up, goes off temporary if in SLEEP mode

// Battery calculation constants (12-bit ADC)
const float ADC_RESOLUTION = 4095.0;             // 12-bit ADC
const float ADC_VOLTAGE_REF = 3.3;
const float BATTERY_MAX_VOLTAGE = 4.2;
const float BATTERY_MIN_VOLTAGE = 3.1;

// ===== GLOBAL VARIABLES =====
int DoorStat = 0;                                // 0=unknown, 1=closed, 2=open
bool delayTimerActive = false;
unsigned long delayTimerStart = 0;
unsigned long delayTimerDuration = 0;

bool doorIsOpen = false;
int batteryPercentage = 0;
String ErrMess = "test error";                   // Error message variable
unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 30000;
int detectedChannel = 0;

// motor current variable
int motorCurrentRun = 0;            // the measured INT analog value for the free running door condition
int motorCurrentStall = 0;          // the measured INT analog value for the door in the stall position
int motorRunThresh = 1000;          // the limit for free running current, above this might be a stall
int motorStallThresh = 1050;        // the limit for the door stall end of roation current, below this might not be stalled
int delayRun = 200;                 // 200 milliseconds after switching on the motor to turn the door, before run measure
int delayStall = 3000;              // Seconds to allow the door to close before measuring the stall current
bool doorStallFlag = false;         // has the door stalled or not? true if stall threshold passed, false if not

// ESP-NOW Message Structure (must match primary)
typedef struct {
    int tankId;
    char command[32];
    int value;
    char message[128];
    int messageType;
} ESPNowMessage;

// Forward declarations
void onESPNowDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void onESPNowDataReceived(const uint8_t *mac, const uint8_t *incomingData, int len);

// ===== CHANNEL DETECTION =====

/**
 * Scan for primary's SSID to detect WiFi channel
 * Inputs: None
 * Outputs: int - detected channel number, or 1 if not found
 */
int scanForPrimaryChannel() {
    Serial.println("DEBUG: Scanning for primary SSID: " + String(primarySSID));
    
    // Try up to 3 times with delays
    for (int attempt = 0; attempt < 3; attempt++) {
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();
        delay(100);
        
        int numNetworks = WiFi.scanNetworks();
        Serial.println("DEBUG: Scan attempt " + String(attempt + 1) + " found " + String(numNetworks) + " networks");
        
        for (int i = 0; i < numNetworks; i++) {
            String ssid = WiFi.SSID(i);
            if (ssid == primarySSID) {
                int channel = WiFi.channel(i);
                Serial.println("DEBUG: Found primary SSID on channel " + String(channel));
                return channel;
            }
        }
        
        if (attempt < 2) {
            Serial.println("DEBUG: Primary not found, retrying in 2 seconds...");
            delay(2000);
        }
    }
    
    Serial.println("WARNING: Primary SSID not found after 3 attempts, defaulting to channel 1");
    return 1;
}

// ===== HARDWARE INITIALIZATION =====

/**
 * Initialize GPIO pins for motor control, sensors, and LED
 * Inputs: None
 * Outputs: None (configures pins)
 */
void initGPIO() {
    // set INPUT/OUTPUT
    pinMode(MOTOR_PWR_PIN, OUTPUT); // run the motor
    pinMode(MOTOR_DIR_PIN, OUTPUT); // direction of the motor
    pinMode(MOTOR_SLEEP_PIN, OUTPUT); // sleep the motor driver
    pinMode(OPTO_SLEEP_PIN, OUTPUT); // sleep the opto LED
    pinMode(MOTOR_ADC_CURRENT_PIN, INPUT); // read the motor current
    // set states
    digitalWrite(MOTOR_DIR_PIN, LOW); // 
    digitalWrite(MOTOR_PWR_PIN, LOW);
    digitalWrite(MOTOR_SLEEP_PIN, LOW);
    digitalWrite(OPTO_SLEEP_PIN, LOW);

    pinMode(DOOR_SENSOR_PIN, INPUT_PULLUP);
    pinMode(BATTERY_ADC_PIN, INPUT);

    pinMode(LED_READY, OUTPUT);
    digitalWrite(LED_READY, HIGH);
    
    Serial.println("DEBUG: GPIO initialized");
}

/**
 * Add primary device as ESP-NOW peer
 * Inputs: None
 * Outputs: None (adds peer to ESP-NOW)
 */
void addESPNowPeer() {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, primaryMACAddress, 6);
    peerInfo.channel = detectedChannel;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        Serial.println("DEBUG: Primary peer added successfully on channel " + String(detectedChannel));
    } else {
        Serial.println("ERROR: Failed to add primary peer");
    }
}

// ===== Channel probing helpers =====
struct __Candidate { int ch; int rssi; String bssid; };

static void __addProbePeer() {
    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, primaryMACAddress, 6);
    p.channel = 0; // follow current channel
    p.encrypt = false;
    if (esp_now_is_peer_exist(p.peer_addr)) esp_now_del_peer(p.peer_addr);
    esp_now_add_peer(&p);
}

static bool __tryChannel(int ch) {
    Serial.printf("Trying channel %d...\n", ch);
    // Ensure a clean slate
    esp_now_deinit();
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    delay(120);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed on this channel");
        return false;
    }
    // Register callbacks (recv will be re-registered in initESPNow too; harmless)
    esp_now_register_send_cb(onESPNowDataSent);
    esp_now_register_recv_cb(onESPNowDataReceived);

    __addProbePeer();

    const char* probe = "SLV probe";
    __probeWaiting = true;
    __probeSuccess = false;

    esp_err_t e = esp_now_send(primaryMACAddress, (const uint8_t*)probe, strlen(probe));
    if (e != ESP_OK) {
        Serial.printf("esp_now_send error=%d\n", e);
        esp_now_deinit();
        return false;
    }

    uint32_t t0 = millis();
    while (__probeWaiting && millis() - t0 < 400) {
        yield();
    }

    bool ok = __probeSuccess;
    Serial.printf("Probe result on ch %d: %s\n", ch, ok ? "SUCCESS" : "FAIL");

    if (!ok) {
        esp_now_deinit();
    } else {
        __espNowReady = true;
    }
    return ok;
}

static int __discoverPrimaryChannelViaProbe() {
    // scan for SSID candidates (strongest per channel)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    int n = WiFi.scanNetworks();
    struct __Candidate cand[20]; 
    int cnt=0;
    for (int i=0; i<n && cnt<20; i++) {
        if (WiFi.SSID(i) == primarySSID) {
            int ch = WiFi.channel(i);
            int rssi = WiFi.RSSI(i);
            String bssid = WiFi.BSSIDstr(i);
            bool have=false;
            for (int k=0;k<cnt;k++) { 
                if (cand[k].ch==ch) { 
                    have=true; 
                    if (rssi>cand[k].rssi) {
                        cand[k].rssi=rssi; 
                        cand[k].bssid=bssid;
                    } 
                    break;
                } 
            }
            if (!have) { 
                cand[cnt].ch=ch; 
                cand[cnt].rssi=rssi; 
                cand[cnt].bssid=bssid; 
                cnt++; 
            }
        }
    }
    // sort by RSSI desc (simple bubble since small)
    for (int a=0;a<cnt;a++) {
        for (int b=a+1;b<cnt;b++) {
            if (cand[b].rssi>cand[a].rssi) { 
                auto tmp=cand[a]; 
                cand[a]=cand[b]; 
                cand[b]=tmp; 
            }
        }
    }

    Serial.println("Probe candidates:");
    for (int i=0;i<cnt;i++) { 
        Serial.printf("  CH %d RSSI %d BSSID %s\n", cand[i].ch, cand[i].rssi, cand[i].bssid.c_str()); 
    }

    for (int i=0;i<cnt;i++) {
        if (__tryChannel(cand[i].ch)) return cand[i].ch;
    }

    // fallback sweep common channels
    int fallback[3] = {1,6,11};
    for (int i=0;i<3;i++) {
        if (__tryChannel(fallback[i])) return fallback[i];
    }

    return -1;
}

/**
 * Initialize ESP-NOW protocol
 * Inputs: None
 * Outputs: None (initializes ESP-NOW and sends initial status)
 */
void initESPNow() {
    // Discover channel by probing MST via ESP-NOW
    detectedChannel = __discoverPrimaryChannelViaProbe();
    if (detectedChannel < 0) {
        Serial.println("ERROR: Could not discover primary channel via probe");
        return;
    }

    // Ensure ESP-NOW is initialized (probe may have inited already)
    if (!__espNowReady) {
        if (esp_now_init() != ESP_OK) {
            Serial.println("ERROR: ESP-NOW init failed");
            return;
        }
    }

    // (Re)register callbacks
    esp_now_register_send_cb(onESPNowDataSent);
    esp_now_register_recv_cb(onESPNowDataReceived);

    // Add peer if not present
    if (!esp_now_is_peer_exist(primaryMACAddress)) addESPNowPeer();

    Serial.println("Connected to ESP-NOW");

    // Startup messages
    delay(3000);
    sendMACAddress();
    delay(1500);
    BattCalcEnergy();
    sendBatteryLevel();
    delay(1500);
    sendDoorStatus();
}

// ===== ESP-NOW CALLBACKS =====

/**
 * Callback when ESP-NOW data is sent
 * Inputs:
 *   - mac_addr: uint8_t* - MAC address of recipient
 *   - status: esp_now_send_status_t - send status
 * Outputs: None
 */
void onESPNowDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        Serial.println("DEBUG: ESP-NOW send SUCCESS");
    } else {
        Serial.println("WARNING: ESP-NOW send FAILED");
    }
    // probe flag tap-in
    if (__probeWaiting) { 
        __probeSuccess = (status == ESP_NOW_SEND_SUCCESS); 
        __probeWaiting = false; 
    }
}

/**
 * Callback when ESP-NOW data is received from primary
 * Inputs:
 *   - mac: uint8_t* - MAC address of sender
 *   - incomingData: uint8_t* - received data
 *   - len: int - data length
 * Outputs: None (processes commands)
 */
void onESPNowDataReceived(const uint8_t *mac, const uint8_t *incomingData, int len) {
  Serial.printf("DEBUG: RAW MESSAGE RECEIVED! len=%d\n", len);

  // Zero-init so any missing bytes don't contain garbage
  ESPNowMessage message = {};
  size_t n = min((size_t)len, (size_t)sizeof(message));  // cast both sides for Arduino's min()
  memcpy(&message, incomingData, n);

  // Ensure strings are null-terminated
  message.command[sizeof(message.command) - 1] = '\0';
  message.message[sizeof(message.message) - 1] = '\0';

  Serial.println("DEBUG: Tank ID in message: " + String(message.tankId));
  Serial.println("DEBUG: Command: " + String(message.command));

  // Only act on commands meant for this tank
  if (message.tankId != TANK_ID_NUMBER) {
    Serial.println("DEBUG: Tank ID mismatch - ignoring message");
    return;
  }

  String command(message.command);

  if (command == "open_slot") {
    Serial.println("DEBUG: Executing open_slot command");
    OpenDoor();

  } else if (command == "close_slot") {
    Serial.println("DEBUG: Executing close_slot command");
    CloseDoor();

  } else if (command == "start_delay") {
    Serial.println("DEBUG: Executing start_delay command");
    // Use your existing globals
    delayRun = true;
    delayStall = millis();

  } else {
    Serial.println("DEBUG: Unknown command received: " + command);
  }
}


// ===== MOTOR CONTROL FUNCTIONS =====

/**
 * OpenDoor - Open the tank door (REF20)
 * Inputs: None
 * Outputs: None (controls motor and updates door status)
 */
void OpenDoor() {
    Serial.println("----------------------------- Running the door open function -----------------------------");

    digitalWrite(MOTOR_SLEEP_PIN, HIGH); // wake up the DC motor control board
    digitalWrite(OPTO_SLEEP_PIN, HIGH); // wake up the opto sensor LED
    
    unsigned long startTime = millis(); // set start to millis()
    const unsigned long MOTOR_TIMEOUT = 6000; // set 6 second timeout

    digitalWrite(MOTOR_DIR_PIN, LOW); // motor to OPEN
    digitalWrite(MOTOR_PWR_PIN, HIGH); // motor ON

    delay(delayRun); // wait a small time for the current to stabilize

    motorCurrentRun = analogRead(MOTOR_ADC_CURRENT_PIN); // read the running rotating current value

    doorIsOpen = readDoorSensor(); // read the door status

    // this seems fixed now
    while (!doorIsOpen && (millis() - startTime) < MOTOR_TIMEOUT) {
        delay(100);
        doorIsOpen = readDoorSensor();
    }

    motorCurrentStall = analogRead(MOTOR_ADC_CURRENT_PIN); // read the stall current value

    digitalWrite(MOTOR_PWR_PIN, LOW); // motor OFF

    if (readDoorSensor()) {
        doorIsOpen = true;
        DoorStat = 2;
        Serial.println("Door is open");
        sendDoorStatus();
    } else {
        DoorStat = 0; // unknown state
        Serial.println("ERROR: Door open timeout");
        ReportErrors("Door open timeout");
    }

    digitalWrite(MOTOR_SLEEP_PIN, LOW); // sleep the DC motor control board
    digitalWrite(OPTO_SLEEP_PIN, LOW); // sleep the opto LED
} // end of open door

/**
 * CloseDoor - Close the tank door (REF21)
 * Inputs: None
 * Outputs: None (controls motor and updates door status)
 */
void CloseDoor() {
    Serial.println("----------------------------- Running the door close function -----------------------------");

    digitalWrite(MOTOR_SLEEP_PIN, HIGH); // wake up the DC motor control board
    digitalWrite(OPTO_SLEEP_PIN, HIGH); // wake up the opto sensor LED
    
    digitalWrite(MOTOR_DIR_PIN, HIGH); // motor to CLOSE
    digitalWrite(MOTOR_PWR_PIN, HIGH); // motor ON

    delay(delayRun); // wait a small time for the current to stabilize

    motorCurrentRun = analogRead(MOTOR_ADC_CURRENT_PIN); // read the running rotating current value

    delay(delayStall); // wait seconds before assuming the door is closed and measuring the stall current

    motorCurrentStall = analogRead(MOTOR_ADC_CURRENT_PIN); // read the stalled current value

    if(motorCurrentStall > motorStallThresh) doorStallFlag=true; // if the motor stall limit is passed, the door has stalled, set the flag to true
    if(motorCurrentStall < motorStallThresh) doorStallFlag=false; // if the motor stall limit is not passed, the door has not stalled, set the flag to false
    
    unsigned long startTime = millis();
    const unsigned long MOTOR_TIMEOUT = 3000; // 3 second timeout

    doorIsOpen = readDoorSensor(); // read sensor
    
    // this seems fixed now
    while ((millis() - startTime) < MOTOR_TIMEOUT) {
        delay(100);
    }
    
    digitalWrite(MOTOR_PWR_PIN, LOW); // motor OFF

    delay(200); // wait

    doorIsOpen = readDoorSensor(); // read the sensor
    
    if (!doorIsOpen && doorStallFlag==true) { // if the doorIsOpen flag is FALSE AND the doorStallFlag is TRUE. The door has closed
        DoorStat = 1; // set the door state 1
        Serial.println("Door is closed");
        sendDoorStatus();
    } else {
        DoorStat = 0; // unknown status
        Serial.println("ERROR: Door close timeout");
        Serial.print("Door stall flag is: ");
        Serial.println(doorStallFlag);
        ReportErrors("Door close timeout");
    }

    digitalWrite(MOTOR_SLEEP_PIN, LOW); // sleep the DC motor control board
    digitalWrite(OPTO_SLEEP_PIN, LOW); // sleep the opto LED
} // end of door close function

// ===== SENSOR FUNCTIONS =====

/**
 * Read door sensor state
 * Inputs: None
 * Outputs: bool - true if door is open, false if closed
 */
bool readDoorSensor() {
    return digitalRead(DOOR_SENSOR_PIN);
}

/**
 * Calculate battery percentage from voltage using polynomial curve fitting
 * Input: batteryVoltage (float) - Total battery voltage from 2.5V to 4.2V
 * Output: float - Battery percentage from 0% to 100%
 * 
 * Uses three polynomial equations based on discharge curve analysis:
 * Range 1 (2.5-3.4V): Linear equation y = 21.1x - 47.4
 * Range 2 (3.4-3.8V): 3rd order polynomial y = -13582 + 12408x - 3770x² + 382x³  
 * Range 3 (3.8-4.2V): 2nd order polynomial y = -2244 + 1105x - 130x²
 */
float calculateBatteryPercentage(float batteryVoltage) {
    float percentage = 0.0;
    
    // Constrain input voltage to valid range
    if (batteryVoltage < 2.5) batteryVoltage = 2.5;
    if (batteryVoltage > 4.2) batteryVoltage = 4.2;

    batteryVoltage = batteryVoltage + 0.25; // ???? 
    
    // Range 1: 2.5V to 3.0V (Low voltage range - Linear)
    // Equation from curve data: y = 22x - 49.7 (R² = 0.99)
    if (batteryVoltage >= 2.5 && batteryVoltage <= 3.0) {
        percentage = 22 * batteryVoltage - 49.7;
    }

    // Range 2: 3.0V to 3.6V (Mid voltage range - 3rd order polynomial)  
    // Equation from curve data: y = -2566 + 2345x + -714x² + 73.2x³ (R² = 0.998)
    else if (batteryVoltage > 3.0 && batteryVoltage <= 3.6) {
        float x = batteryVoltage;
        float x2 = x * x;
        float x3 = x2 * x;
        percentage = -2566 + 2345*x + -714*x2 + 73.2*x3;
    }
    
    // Range 3: 3.6V to 3.85V (Mid voltage range - 3rd order polynomial)  
    // Equation from curve data: y = 470219 + -377775x + 101090x² + -9009x³ (R² = 0.997)
    else if (batteryVoltage > 3.6 && batteryVoltage <= 3.85) {
        float x = batteryVoltage;
        float x2 = x * x;
        float x3 = x2 * x;
        percentage = 470219 + -377775*x + 101090*x2 + -9009*x3;
    }
    
    // Range 4: 3.85V to 4.25V (High voltage range - 2nd order polynomial)
    // Equation from curve data: y = -2426 + 1193x + -141x² (R² = 0.998)
    else if (batteryVoltage > 3.85 && batteryVoltage <= 4.25) {
        float x = batteryVoltage;
        float x2 = x * x;
        percentage = -2426 + 1193*x + -141*x2;
    }
    
    // Constrain output percentage to valid range
    if (percentage < 0.0) percentage = 0.0;
    if (percentage > 100.0) percentage = 100.0;
    
    return percentage;
}

/**
 * BattCalcEnergy - Calculate battery energy percentage
 * Inputs: None
 * Outputs: None (prints ADC value, calculates percentage)
 */
void BattCalcEnergy() {
    int adcValue = analogRead(BATTERY_ADC_PIN);
    Serial.println("DEBUG: ADC value: " + String(adcValue));
    
    // Convert ADC to voltage (12-bit ADC)
    float adcVoltage = (adcValue / ADC_RESOLUTION) * ADC_VOLTAGE_REF;
    
    // Multiply by 2 to get actual battery voltage (voltage divider)
    float batteryVoltage = adcVoltage * 2.0;
    
    // Calculate percentage using polynomial curves
    float percentage = calculateBatteryPercentage(batteryVoltage);
    
    batteryPercentage = (int)percentage;
    
    Serial.println("DEBUG: Battery voltage: " + String(batteryVoltage) + "V");
    Serial.println("DEBUG: Battery percentage: " + String(batteryPercentage) + "%");
}

// ===== TIMER FUNCTIONS =====

/**
 * TimedOpening - Start delay timer for automatic door opening
 * Inputs:
 *   - hours: int - delay duration in hours (1-24)
 * Outputs: None (starts timer)
 */
void TimedOpening(int hours) {
    if (hours < 1 || hours > 24) {
        Serial.println("ERROR: Invalid delay hours: " + String(hours));
        ReportErrors("Invalid delay hours");
        return;
    }
    
    delayTimerActive = true;
    delayTimerStart = millis();
    delayTimerDuration = (unsigned long)hours * 3600000;
    
    Serial.println("DEBUG: Delay timer started for " + String(hours) + " hours");
}

/**
 * Cancel active delay timer
 * Inputs: None
 * Outputs: None (cancels timer)
 */
void cancelDelayTimer() {
    delayTimerActive = false;
    delayTimerStart = 0;
    delayTimerDuration = 0;
    
    Serial.println("DEBUG: Delay timer canceled");
}

/**
 * Check if delay timer has expired
 * Inputs: None
 * Outputs: None (executes OpenDoor if expired)
 */
void checkDelayTimer() {
    if (!delayTimerActive) return;
    
    unsigned long elapsed = millis() - delayTimerStart;
    
    if (elapsed >= delayTimerDuration) {
        delayTimerActive = false;
        Serial.println("DEBUG: Delay timer expired - executing timed open");
        Serial.println("Timer is activated 0 hours");
        OpenDoor();  // v2.1: Timer calls OpenDoor directly
        sendTimedOpenNotification();
    }
}

// ===== SLEEP FUNCTIONS =====

/**
 * LowPowerSleep - Put device into deep sleep mode
 * Inputs:
 *   - hours: int - sleep duration in hours (1-24)
 * Outputs: None (enters deep sleep)
 */
void LowPowerSleep(int hours) {
    digitalWrite(LED_READY, LOW); // LED OFF
    if (hours < 1 || hours > 24) {
        Serial.println("ERROR: Invalid sleep hours: " + String(hours));
        ReportErrors("Invalid sleep hours");
        return;
    }
    
    // Send final status before sleeping
    BattCalcEnergy();
    sendBatteryLevel();
    sendDoorStatus();
    delay(100);
    
    Serial.println("DEBUG: Entering deep sleep for " + String(hours) + " hours");
    
    uint64_t sleepTime = (uint64_t)hours * 3600 * 1000000;
    esp_sleep_enable_timer_wakeup(sleepTime);
    esp_deep_sleep_start();
}

// ===== ESP-NOW MESSAGE FUNCTIONS =====

/**
 * ReportErrors - Send error message to primary
 * Inputs:
 *   - errorMsg: String - error description
 * Outputs: None (sends ESP-NOW message)
 */
void ReportErrors(String errorMsg) {
    ErrMess = errorMsg;
    
    ESPNowMessage message;
    message.tankId = TANK_ID_NUMBER;
    message.messageType = 2;
    message.value = 0;
    strncpy(message.message, errorMsg.c_str(), sizeof(message.message) - 1);
    message.message[sizeof(message.message) - 1] = '\0';
    
    esp_err_t result = esp_now_send(primaryMACAddress, (uint8_t *)&message, sizeof(message));
    if (result != ESP_OK) {
        Serial.println("ERROR: Failed to send error message");
    }
}

/**
 * Send battery level to primary
 * Inputs: None
 * Outputs: None (sends ESP-NOW message)
 */
void sendBatteryLevel() {
    ESPNowMessage message;
    message.tankId = TANK_ID_NUMBER;
    message.messageType = 3;
    message.value = batteryPercentage;
    strcpy(message.message, "");
    
    esp_err_t result = esp_now_send(primaryMACAddress, (uint8_t *)&message, sizeof(message));
    if (result == ESP_OK) {
        Serial.println("DEBUG: Battery level sent: " + String(batteryPercentage) + "%");
    }
}

/**
 * Send MAC address to primary
 * Inputs: None
 * Outputs: None (sends ESP-NOW message)
 */
void sendMACAddress() {
    String macAddress = WiFi.macAddress();
    
    ESPNowMessage message;
    message.tankId = TANK_ID_NUMBER;
    message.messageType = 4;
    message.value = 0;
    strncpy(message.message, macAddress.c_str(), sizeof(message.message) - 1);
    message.message[sizeof(message.message) - 1] = '\0';
    
    esp_err_t result = esp_now_send(primaryMACAddress, (uint8_t *)&message, sizeof(message));
    if (result == ESP_OK) {
        Serial.println("DEBUG: MAC address sent: " + macAddress);
    }
}

/**
 * Send door status to primary
 * Inputs: None
 * Outputs: None (sends ESP-NOW message)
 */
void sendDoorStatus() {
    doorIsOpen = readDoorSensor();
    
    ESPNowMessage message;
    message.tankId = TANK_ID_NUMBER;
    message.messageType = 5;
    message.value = doorIsOpen ? 1 : 0;
    strcpy(message.message, "");
    
    esp_err_t result = esp_now_send(primaryMACAddress, (uint8_t *)&message, sizeof(message));
    if (result == ESP_OK) {
        Serial.println("DEBUG: Door status sent: " + String(doorIsOpen ? "OPEN" : "CLOSED"));
    }
}

/**
 * Send heartbeat to primary
 * Inputs: None
 * Outputs: None (sends ESP-NOW message)
 */
void sendHeartbeat() {
    ESPNowMessage message;
    message.tankId = TANK_ID_NUMBER;
    message.messageType = 6;
    message.value = 0;
    strcpy(message.message, "heartbeat");
    
    esp_now_send(primaryMACAddress, (uint8_t *)&message, sizeof(message));
}

/**
 * Send timed open notification to primary
 * Inputs: None
 * Outputs: None (sends ESP-NOW message)
 */
void sendTimedOpenNotification() {
    ESPNowMessage message;
    message.tankId = TANK_ID_NUMBER;
    message.messageType = 1;
    message.value = 1;
    strcpy(message.message, "timed_open_executed");
    
    esp_now_send(primaryMACAddress, (uint8_t *)&message, sizeof(message));
    Serial.println("DEBUG: Timed open notification sent");
}

// ===== MAIN SETUP AND LOOP =====

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("=== Fish Tank Secondary v2.9 Starting ===");
    Serial.println("Tank ID: " + TANK_ID_STRING + " (" + String(TANK_ID_NUMBER) + ")");
    
    String macAddress = WiFi.macAddress();
    Serial.println("MAC address is: " + macAddress);
    
    // Set DoorStat to 0 on power up
    DoorStat = 0;
    
    initGPIO();
    initESPNow();
    
    // Read initial states
    doorIsOpen = readDoorSensor();
    BattCalcEnergy();
    
    if (doorIsOpen) {
        Serial.println("Door is open");
        DoorStat = 2;
    } else {
        Serial.println("Door is closed");
        DoorStat = 1;
    }
    
    Serial.println("=== Initialization complete ===");
}

void loop() {
    checkDelayTimer();
    
    // ADD: Re-scan for primary if no heartbeat response for 5 minutes
    static unsigned long lastPrimaryContact = 0;
    static bool initialContactMade = false;
    
    // If we haven't sent a heartbeat in 5 minutes, try re-initializing
    // This helps recover from channel mismatches or power-on timing issues
    if (millis() - lastHeartbeat > 300000) {  // 5 minutes
        Serial.println("WARNING: No primary response for 5 minutes - re-scanning for channel");
        initESPNow();  // Re-initialize ESP-NOW with fresh channel scan
        lastHeartbeat = millis();  // Reset timer to avoid rapid re-scanning
    }
    
    // Regular heartbeat transmission
    if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
        sendHeartbeat();
        lastHeartbeat = millis();
        initialContactMade = true;  // Mark that we've attempted contact
    }
    
    // Periodic status updates every 2 minutes
    static unsigned long lastStatusUpdate = 0;
    if (millis() - lastStatusUpdate > 120000) {
        BattCalcEnergy();
        sendBatteryLevel();
        sendDoorStatus();
        lastStatusUpdate = millis();
    }
    
    // Monitor door state changes every 5 seconds
    static bool lastDoorState = false;
    static unsigned long lastDoorCheck = 0;
    if (millis() - lastDoorCheck > 5000) {
        bool currentDoorState = readDoorSensor();
        if (currentDoorState != lastDoorState) {
            doorIsOpen = currentDoorState;
            
            if (doorIsOpen) {
                Serial.println("Door is open");
                DoorStat = 2;
            } else {
                Serial.println("Door is closed");
                DoorStat = 1;
            }
            
            sendDoorStatus();
        }
        lastDoorState = currentDoorState;
        lastDoorCheck = millis();
    }
    
    delay(100);
    digitalWrite(LED_READY, HIGH); // LED is normally lit for READY
}