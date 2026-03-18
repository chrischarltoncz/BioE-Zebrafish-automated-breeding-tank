/*
 * ESP32-S3 Fish Tank Secondary Control System v5.1
 * Individual tank controller that communicates with Primary via ESP-NOW
 *
*
 * CHANGES in V5.1 vs V4.8:
 * - fix the battery reporting 0% when the battery level is ok, was caused by an issue of ESP-NOW clashing with the function which calculates the battery voltage
 *
 * CHANGES in V4.6:
 * - attempt to fix sleep after open error which didn't work again
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_sleep.h>
#include <esp_wifi.h>

// ===== RTC MEMORY FOR DEEP SLEEP STATE =====
#define RTC_MAGIC 0xFEEDBEEF

RTC_DATA_ATTR uint32_t rtc_magic = 0;
RTC_DATA_ATTR bool rtc_timedOpenActive = false;
RTC_DATA_ATTR uint32_t rtc_remainingSeconds = 0;
RTC_DATA_ATTR bool rtc_inSleepCycle = false;
RTC_DATA_ATTR uint32_t rtc_checksum = 0;

// ===== CONFIGURATION - MODIFY FOR EACH DEVICE =====
const String TANK_ID_STRING = "Tank6";
const int TANK_ID_NUMBER = 6;
const char* primarySSID = "BHN-Guest";
uint8_t primaryMACAddress[6] = {0x94, 0xA9, 0x90, 0x17, 0x76, 0xD8};

volatile bool __probeWaiting = false;
volatile bool __probeSuccess = false;
bool __espNowReady = false;

// ===== GPIO Pin Configuration =====
const int MOTOR_PWR_PIN = D2;
const int MOTOR_SLEEP_PIN = D7;
const int MOTOR_DIR_PIN = D10;
const int OPTO_SLEEP_PIN = D11;
const int DOOR_SENSOR_PIN = D9;
const int BATTERY_ADC_PIN = A3;
const int MOTOR_ADC_CURRENT_PIN = A1;
const int LED_READY = 13;

// ===== PWM Configuration for Motor (NEW in V4.3) =====
const int MOTOR_PWM_CHANNEL = 0;
const int MOTOR_PWM_FREQ    = 1000;
const int MOTOR_PWM_RES     = 8;
const int MOTOR_DUTY        = 80;   // Duty cycle: value/255. Tank 6 = 80. Tank 4 = 150. Tank 5 = 80. Tank 3 = 150. Tank 2 = 150. Tank 1 = 150.

// ===== Battery calculation constants =====
const float ADC_RESOLUTION = 4095.0;
const float ADC_VOLTAGE_REF = 3.3;
const float BATTERY_MAX_VOLTAGE = 4.2;
const float BATTERY_MIN_VOLTAGE = 3.1;

// ===== Sleep cycle constants =====
const uint32_t CYCLE_SLEEP_HOURS = 4;
const uint32_t CYCLE_WAKE_SECONDS = 300;

// ===== GLOBAL VARIABLES =====
int DoorStat = 0;
bool doorIsOpen = false;
int batteryPercentage = 0;
String ErrMess = "test error";
uint32_t lastHeartbeat = 0;
uint32_t lastReconnectAttempt = 0;
const uint32_t HEARTBEAT_INTERVAL = 30000;
int detectedChannel = 0;

volatile bool pendingTimedSleep = false;
volatile int pendingTimedSleepMinutes = 0;
volatile bool pendingLowPowerSleep = false;
volatile int pendingLowPowerSleepMinutes = 0;
volatile bool pendingSleepCycle = false;
volatile bool g_isTimedOpenExecution = false;

int motorCurrentRun = 0;
int motorCurrentStall = 0;
int motorRunThresh = 1000;
int motorStallThresh = 1050;
int delayRun = 200;
int delayStall = 3000;
bool doorStallFlag = false;

// ===== Message structure (must match Primary) =====
typedef struct {
    int tankId;
    char command[32];
    int value;
    char message[128];
    int messageType;
} ESPNowMessage;

// ===== FORWARD DECLARATIONS (Required for compilation) =====
void onESPNowDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void onESPNowDataReceived(const uint8_t *mac, const uint8_t *incomingData, int len);
void setMotorPower(uint8_t power);
void OpenDoor();
void CloseDoor();
bool readDoorSensor();
void BattCalcEnergy();
void sendBatteryLevel();
void sendDoorStatus();
void sendMACAddress();
void sendHeartbeat();
void sendTimedOpenNotification();
void ReportErrors(String errorMsg);
void cancelDelayTimer();
void EnterSleepCycle();
void ContinueSleepCycle();
void LowPowerSleep(int minutes);
void TimedOpeningWithSleep(int minutes);
void initESPNow();
void initGPIO();

// ===== RTC MEMORY VALIDATION =====

uint32_t calculateRTCChecksum() {
    uint32_t sum = 0;
    sum += rtc_magic;
    sum += (rtc_timedOpenActive ? 1 : 0);
    sum += rtc_remainingSeconds;
    sum += (rtc_inSleepCycle ? 1 : 0);
    return sum;
}

bool validateRTCMemory() {
    if (rtc_magic != RTC_MAGIC) {
        Serial.println("RTC: First boot or magic mismatch - initializing");
        rtc_magic = RTC_MAGIC;
        rtc_timedOpenActive = false;
        rtc_remainingSeconds = 0;
        rtc_inSleepCycle = false;
        rtc_checksum = calculateRTCChecksum();
        return false;
    }
    
    uint32_t expectedChecksum = calculateRTCChecksum();
    if (rtc_checksum != expectedChecksum) {
        Serial.println("RTC: Checksum mismatch - memory corrupted, resetting");
        rtc_timedOpenActive = false;
        rtc_remainingSeconds = 0;
        rtc_inSleepCycle = false;
        rtc_checksum = calculateRTCChecksum();
        return false;
    }
    
    Serial.println("RTC: Memory validated successfully");
    return true;
}

void updateRTCChecksum() {
    rtc_checksum = calculateRTCChecksum();
}

// ===== PWM MOTOR CONTROL =====

void setMotorPower(uint8_t power) {
    ledcWrite(MOTOR_PWM_CHANNEL, power);
}

// ===== HARDWARE INITIALIZATION =====

void initGPIO() {
    pinMode(MOTOR_PWR_PIN, OUTPUT);
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    pinMode(MOTOR_SLEEP_PIN, OUTPUT);
    pinMode(OPTO_SLEEP_PIN, OUTPUT);
    pinMode(MOTOR_ADC_CURRENT_PIN, INPUT);
    ledcSetup(MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(MOTOR_PWR_PIN, MOTOR_PWM_CHANNEL);
    setMotorPower(0);
    
    digitalWrite(MOTOR_DIR_PIN, LOW);
    digitalWrite(MOTOR_SLEEP_PIN, LOW);
    digitalWrite(OPTO_SLEEP_PIN, LOW);

    pinMode(DOOR_SENSOR_PIN, INPUT_PULLUP);
    pinMode(BATTERY_ADC_PIN, INPUT);
    pinMode(LED_READY, OUTPUT);
    digitalWrite(LED_READY, HIGH);
    
    Serial.println("DEBUG: GPIO initialized (with PWM motor control)");
}

// ===== CHANNEL DETECTION =====

int scanForPrimaryChannel() {
    Serial.println("DEBUG: Scanning for primary SSID: " + String(primarySSID));
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
    p.channel = 0;
    p.encrypt = false;
    if (esp_now_is_peer_exist(p.peer_addr)) esp_now_del_peer(p.peer_addr);
    esp_now_add_peer(&p);
}

static bool __tryChannel(int ch) {
    Serial.printf("Trying channel %d...\n", ch);
    esp_now_deinit();
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    delay(120);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed on this channel");
        return false;
    }
    
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

    int fallback[3] = {1,6,11};
    for (int i=0;i<3;i++) {
        if (__tryChannel(fallback[i])) return fallback[i];
    }

    return -1;
}

void initESPNow() {
    Serial.println("DEBUG: Starting ESP-NOW initialization...");
    
    detectedChannel = __discoverPrimaryChannelViaProbe();
    if (detectedChannel < 0) {
        Serial.println("ERROR: Could not discover primary channel via probe");
        return;
    }
    
    Serial.println("DEBUG: Primary found on channel " + String(detectedChannel));
    if (!__espNowReady) {
        if (esp_now_init() != ESP_OK) {
            Serial.println("ERROR: ESP-NOW init failed");
            return;
        }
    }

    esp_now_register_send_cb(onESPNowDataSent);
    esp_now_register_recv_cb(onESPNowDataReceived);

    if (!esp_now_is_peer_exist(primaryMACAddress)) addESPNowPeer();

    Serial.println("Connected to ESP-NOW");

    delay(3000);
    Serial.println("DEBUG: Sending initial connection messages...");
    sendMACAddress();
    delay(1500);
    BattCalcEnergy();
    sendBatteryLevel();
    delay(1500);
    sendDoorStatus();
    Serial.println("DEBUG: ESP-NOW initialization complete");
}

// ===== ESP-NOW CALLBACKS =====

void onESPNowDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        Serial.println("DEBUG: ESP-NOW send SUCCESS");
    } else {
        Serial.println("WARNING: ESP-NOW send FAILED");
    }
    if (__probeWaiting) { 
        __probeSuccess = (status == ESP_NOW_SEND_SUCCESS);
        __probeWaiting = false; 
    }
}

void onESPNowDataReceived(const uint8_t *mac, const uint8_t *incomingData, int len) {
    Serial.printf("DEBUG: RAW MESSAGE RECEIVED! len=%d\n", len);
    ESPNowMessage message = {};
    size_t n = min((size_t)len, (size_t)sizeof(message));
    memcpy(&message, incomingData, n);

    message.command[sizeof(message.command) - 1] = '\0';
    message.message[sizeof(message.message) - 1] = '\0';

    Serial.println("DEBUG: Tank ID in message: " + String(message.tankId));
    Serial.println("DEBUG: Command: " + String(message.command));
    if (message.tankId != TANK_ID_NUMBER) {
        Serial.println("DEBUG: Tank ID mismatch - ignoring message");
        return;
    }

    String command(message.command);

    if (command == "open_slot") {
        Serial.println("DEBUG: Executing open_slot command");
        g_isTimedOpenExecution = false;
        OpenDoor();

    } else if (command == "close_slot") {
        Serial.println("DEBUG: Executing close_slot command");
        CloseDoor();

    } else if (command == "start_delay") {
        Serial.println("DEBUG: Scheduling timed opening with sleep");
        pendingTimedSleepMinutes = message.value;  
        pendingTimedSleep = true;
    } else if (command == "cancel_delay") {
        Serial.println("DEBUG: Executing cancel_delay command");
        cancelDelayTimer();
    } else if (command == "go_sleep") {
        Serial.println("DEBUG: Scheduling low power sleep");
        pendingLowPowerSleepMinutes = message.value;  
        pendingLowPowerSleep = true;
    } else {
        Serial.println("DEBUG: Unknown command received: " + command);
    }
}

// ===== MOTOR CONTROL FUNCTIONS =====
void OpenDoor() {
    Serial.println("----------------------------- Running the door open function -----------------------------");
    Serial.println("DEBUG: Door status BEFORE opening: " + String(readDoorSensor() ? "OPEN" : "CLOSED"));

    digitalWrite(MOTOR_SLEEP_PIN, HIGH);
    digitalWrite(OPTO_SLEEP_PIN, HIGH);
    Serial.println("DEBUG: Motor driver and opto sensor powered ON");
    delay(100);
    
    uint32_t startTime = millis();
    const uint32_t MOTOR_TIMEOUT = 16000;

    digitalWrite(MOTOR_DIR_PIN, LOW);
    delay(100);
    setMotorPower(MOTOR_DUTY);
    Serial.println("DEBUG: Motor running in OPEN direction (PWM duty: " + String(MOTOR_DUTY) + "/255)");
    delay(delayRun);

    motorCurrentRun = analogRead(MOTOR_ADC_CURRENT_PIN);
    Serial.println("DEBUG: Motor running current: " + String(motorCurrentRun));

    doorIsOpen = readDoorSensor();
    Serial.println("DEBUG: Initial door sensor reading: " + String(doorIsOpen ? "OPEN" : "CLOSED"));
    while (!doorIsOpen && (millis() - startTime) < MOTOR_TIMEOUT) {
        delay(100);
        doorIsOpen = readDoorSensor();
    }

    motorCurrentStall = analogRead(MOTOR_ADC_CURRENT_PIN);
    Serial.println("DEBUG: Motor stall current: " + String(motorCurrentStall));

    setMotorPower(0);
    Serial.println("DEBUG: Motor powered OFF");
    delay(200);

    int openCount = 0;
    const int samples = 9;
    for (int i = 0; i < samples; i++) {
        if (readDoorSensor()) openCount++;
        delay(10);
    }
    bool doorConfirmedOpen = (openCount >= (samples / 2 + 1));

    if (doorConfirmedOpen) {
        doorIsOpen = true;
        DoorStat = 2;
        Serial.println("SUCCESS: Door is OPEN (debounced confirm)");
        sendDoorStatus();
        if (g_isTimedOpenExecution) {
            Serial.println("DEBUG: Timed open -> scheduling automatic sleep cycle");
            delay(500);
            pendingSleepCycle = true;
        } else {
            Serial.println("DEBUG: Manual open -> skip automatic sleep cycle");
        }

        } else {
            DoorStat = 0;
            uint32_t elapsedTime = millis() - startTime;
            Serial.println("ERROR: Door did not open / timeout after " + String(elapsedTime) + "ms");
            ReportErrors("Door open timeout");

        if (g_isTimedOpenExecution) {
            Serial.println("DEBUG: Timed open failed -> still scheduling automatic sleep cycle");
            delay(500);
            pendingSleepCycle = true;
        } else {
            Serial.println("DEBUG: Non-timed open failed -> staying awake");
        }
    }

    digitalWrite(MOTOR_SLEEP_PIN, LOW);
    digitalWrite(OPTO_SLEEP_PIN, LOW);
    Serial.println("DEBUG: Motor driver and opto sensor powered OFF");
    Serial.println("----------------------------- Door open function complete -----------------------------");

    g_isTimedOpenExecution = false;
}

void CloseDoor() {
    Serial.println("----------------------------- Running the door close function -----------------------------");
    digitalWrite(MOTOR_SLEEP_PIN, HIGH);
    digitalWrite(OPTO_SLEEP_PIN, HIGH);
    
    digitalWrite(MOTOR_DIR_PIN, HIGH);
    delay(100); 
    setMotorPower(MOTOR_DUTY);

    delay(delayRun);

    motorCurrentRun = analogRead(MOTOR_ADC_CURRENT_PIN);

    delay(delayStall);

    motorCurrentStall = analogRead(MOTOR_ADC_CURRENT_PIN);
    if(motorCurrentStall > motorStallThresh) doorStallFlag=true;
    if(motorCurrentStall < motorStallThresh) doorStallFlag=false;
    
    uint32_t startTime = millis();
    const uint32_t MOTOR_TIMEOUT = 16000;

    doorIsOpen = readDoorSensor();
    while ((millis() - startTime) < MOTOR_TIMEOUT) {
        delay(100);
    }
    
    setMotorPower(0);  

    delay(200);
    doorIsOpen = readDoorSensor();
    
    if (!doorIsOpen && doorStallFlag==true) {
        DoorStat = 1;
        Serial.println("Door is closed");
        sendDoorStatus();
    } else {
        DoorStat = 0;
        Serial.println("ERROR: Door close timeout");
        Serial.print("Door stall flag is: ");
        Serial.println(doorStallFlag);
        ReportErrors("Door close timeout");
    }

    digitalWrite(MOTOR_SLEEP_PIN, LOW);
    digitalWrite(OPTO_SLEEP_PIN, LOW);
}

// ===== SENSOR FUNCTIONS =====

bool readDoorSensor() {
    return digitalRead(DOOR_SENSOR_PIN);
}

float calculateBatteryPercentage(float batteryVoltage) {
    float percentage = 0.0;
    
    if (batteryVoltage < 2.5) batteryVoltage = 2.5;
    if (batteryVoltage > 4.2) batteryVoltage = 4.2;

    batteryVoltage = batteryVoltage + 0.25;
    if (batteryVoltage >= 2.5 && batteryVoltage <= 3.0) {
        percentage = 22 * batteryVoltage - 49.7;
    }
    else if (batteryVoltage > 3.0 && batteryVoltage <= 3.6) {
        float x = batteryVoltage;
        float x2 = x * x;
        float x3 = x2 * x;
        percentage = -2566 + 2345*x + -714*x2 + 73.2*x3;
    }
    else if (batteryVoltage > 3.6 && batteryVoltage <= 3.85) {
        float x = batteryVoltage;
        float x2 = x * x;
        float x3 = x2 * x;
        percentage = 470219 + -377775*x + 101090*x2 + -9009*x3;
    }
    else if (batteryVoltage > 3.85 && batteryVoltage <= 4.25) {
        float x = batteryVoltage;
        float x2 = x * x;
        percentage = -2426 + 1193*x + -141*x2;
    }
    
    if (percentage < 0.0) percentage = 0.0;
    if (percentage > 100.0) percentage = 100.0;
    
    return percentage;
}

void BattCalcEnergy() {
    int adcValue = analogRead(BATTERY_ADC_PIN);
    Serial.println("DEBUG: ADC value: " + String(adcValue));
    
    float adcVoltage = (adcValue / ADC_RESOLUTION) * ADC_VOLTAGE_REF;
    float batteryVoltage = adcVoltage * 2.0;
    float percentage = calculateBatteryPercentage(batteryVoltage);
    
    batteryPercentage = (int)percentage;
    
    Serial.println("DEBUG: Battery voltage: " + String(batteryVoltage) + "V");
    Serial.println("DEBUG: Battery percentage: " + String(batteryPercentage) + "%");
}

// ===== TIMER FUNCTIONS WITH SLEEP =====

void TimedOpeningWithSleep(int minutes) {
    if (minutes < 1 || minutes > 7200) {  
        Serial.println("ERROR: Invalid delay minutes: " + String(minutes));
        ReportErrors("Invalid delay minutes");
        return;
    }
    
    uint32_t totalSeconds = (uint32_t)minutes * 60;
    const uint32_t TEN_MINUTES = 600;
    
    uint32_t sleepSeconds;
    if (totalSeconds > TEN_MINUTES) {
        sleepSeconds = totalSeconds - TEN_MINUTES;
    } else {
        sleepSeconds = 0;
    }
    
    Serial.println("DEBUG: Timed open with sleep initiated");
    if (sleepSeconds == 0) {
        rtc_timedOpenActive = true;
        rtc_remainingSeconds = totalSeconds;
        updateRTCChecksum();
        
        uint32_t waitStart = millis();
        uint32_t waitDuration = totalSeconds * 1000;
        while (millis() - waitStart < waitDuration) {
            if (millis() - lastHeartbeat > 30000) {
                sendHeartbeat();
                BattCalcEnergy();
                sendBatteryLevel();
                lastHeartbeat = millis();
            }
            delay(1000);
        }
        
        rtc_timedOpenActive = false;
        rtc_remainingSeconds = 0;
        updateRTCChecksum();
        
        sendTimedOpenNotification();
        delay(200);
        
        g_isTimedOpenExecution = true;
        OpenDoor();
        g_isTimedOpenExecution = false;
        
        return;
    }
    
    rtc_timedOpenActive = true;
    rtc_remainingSeconds = TEN_MINUTES;
    updateRTCChecksum();
    
    BattCalcEnergy();
    sendBatteryLevel();
    sendDoorStatus();
    delay(500);
    
    esp_now_deinit();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);
    
    digitalWrite(LED_READY, LOW);
    gpio_hold_en((gpio_num_t)LED_READY);
    gpio_deep_sleep_hold_en();
    
    uint64_t sleepTime = (uint64_t)sleepSeconds * 1000000;
    esp_sleep_enable_timer_wakeup(sleepTime);
    
    esp_deep_sleep_start();
}

void cancelDelayTimer() {
    rtc_timedOpenActive = false;
    rtc_remainingSeconds = 0;
    updateRTCChecksum();
    
    Serial.println("DEBUG: Timed open canceled");
}

// ===== SLEEP CYCLE FUNCTIONS =====

void EnterSleepCycle() {
    Serial.println("========== ENTERING AUTOMATIC SLEEP CYCLE ==========");
    Serial.println("Cycle pattern: 4 hours sleep -> 5 minutes awake -> repeat");
    
    rtc_inSleepCycle = true;
    updateRTCChecksum();
    
    // FIX V4.8: Ensuring global batteryPercentage is updated and visible 
    // before creating the notification message.
    BattCalcEnergy();
    sendBatteryLevel();
    sendDoorStatus();

    // BUG FIX V4.8: Removed local block {} to ensure direct variable access
    // and increased delay to 1s to allow full ESP-NOW delivery.
    ESPNowMessage sleepNotify = {};
    sleepNotify.tankId  = TANK_ID_NUMBER;
    sleepNotify.messageType = 1;
    sleepNotify.value   = batteryPercentage;   
    sleepNotify.command[0] = '\0';
    strcpy(sleepNotify.message, "entering_sleep_cycle");
    esp_now_send(primaryMACAddress, (uint8_t *)&sleepNotify, sizeof(sleepNotify));
    
    Serial.println("DEBUG: Sleep cycle notification sent (battery: " + String(batteryPercentage) + "%)");

    delay(1000); 
    
    digitalWrite(OPTO_SLEEP_PIN, LOW);
    delay(10);
    
    esp_now_deinit();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);
    
    digitalWrite(LED_READY, LOW);
    gpio_hold_en((gpio_num_t)LED_READY);
    gpio_deep_sleep_hold_en();
    Serial.println("DEBUG: Entering 4-hour sleep phase");
    Serial.flush();
    delay(100);
    uint64_t sleepTime = (uint64_t)CYCLE_SLEEP_HOURS * 3600 * 1000000;
    esp_sleep_enable_timer_wakeup(sleepTime);
    
    esp_deep_sleep_start();
}

void ContinueSleepCycle() {
    Serial.println("========== CONTINUING SLEEP CYCLE ==========");
    
    BattCalcEnergy();
    sendBatteryLevel();
    sendDoorStatus();
    delay(1000);
    
    digitalWrite(OPTO_SLEEP_PIN, LOW);
    delay(10);
    
    esp_now_deinit();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);
    
    digitalWrite(LED_READY, LOW);
    gpio_hold_en((gpio_num_t)LED_READY);
    gpio_deep_sleep_hold_en();
    uint64_t sleepTime = (uint64_t)CYCLE_SLEEP_HOURS * 3600 * 1000000;
    esp_sleep_enable_timer_wakeup(sleepTime);
    
    esp_deep_sleep_start();
}

void LowPowerSleep(int minutes) {
    if (minutes < 1 || minutes > 7200) {  
        Serial.println("ERROR: Invalid sleep minutes: " + String(minutes));
        ReportErrors("Invalid sleep minutes");
        return;
    }

    rtc_inSleepCycle = false;
    updateRTCChecksum();
    
    BattCalcEnergy();
    sendBatteryLevel();
    sendDoorStatus();
    delay(1000);
    
    esp_now_deinit();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);
    digitalWrite(LED_READY, LOW);
    gpio_hold_en((gpio_num_t)LED_READY);
    gpio_deep_sleep_hold_en();
    
    uint64_t sleepTime = (uint64_t)minutes * 60 * 1000000;
    esp_sleep_enable_timer_wakeup(sleepTime);
    
    esp_deep_sleep_start();
}

// ===== ESP-NOW MESSAGE FUNCTIONS =====

void ReportErrors(String errorMsg) {
    ErrMess = errorMsg;
    
    ESPNowMessage message;
    message.tankId = TANK_ID_NUMBER;
    message.messageType = 2;
    message.value = 0;
    strncpy(message.message, errorMsg.c_str(), sizeof(message.message) - 1);
    message.message[sizeof(message.message) - 1] = '\0';
    esp_now_send(primaryMACAddress, (uint8_t *)&message, sizeof(message));
}

void sendBatteryLevel() {
    ESPNowMessage message;
    message.tankId = TANK_ID_NUMBER;
    message.messageType = 3;
    message.value = batteryPercentage;
    strcpy(message.message, "");
    esp_now_send(primaryMACAddress, (uint8_t *)&message, sizeof(message));
}

void sendMACAddress() {
    String macAddress = WiFi.macAddress();
    
    ESPNowMessage message;
    message.tankId = TANK_ID_NUMBER;
    message.messageType = 4;
    message.value = 0;
    strncpy(message.message, macAddress.c_str(), sizeof(message.message) - 1);
    message.message[sizeof(message.message) - 1] = '\0';
    
    esp_now_send(primaryMACAddress, (uint8_t *)&message, sizeof(message));
}

void sendDoorStatus() {
    doorIsOpen = readDoorSensor();
    
    ESPNowMessage message;
    message.tankId = TANK_ID_NUMBER;
    message.messageType = 5;
    message.value = doorIsOpen ? 1 : 0;
    strcpy(message.message, "");
    
    esp_now_send(primaryMACAddress, (uint8_t *)&message, sizeof(message));
}

void sendHeartbeat() {
    ESPNowMessage message;
    message.tankId = TANK_ID_NUMBER;
    message.messageType = 6;
    message.value = 0;
    strcpy(message.message, "heartbeat");
    esp_now_send(primaryMACAddress, (uint8_t *)&message, sizeof(message));
}

void sendTimedOpenNotification() {
    ESPNowMessage message;
    message.tankId = TANK_ID_NUMBER;
    message.messageType = 1;
    message.value = 1;
    strcpy(message.message, "timed_open_executed");
    
    esp_now_send(primaryMACAddress, (uint8_t *)&message, sizeof(message));
}

// ===== MAIN SETUP AND LOOP =====

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("=== Fish Tank Secondary v5.1 Starting ===");
    
    bool rtcValid = validateRTCMemory();
    
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    switch(wakeup_reason) {
        case ESP_SLEEP_WAKEUP_TIMER:
            gpio_deep_sleep_hold_dis();
            gpio_hold_dis((gpio_num_t)LED_READY);
            break;
        default:
            break;
    }
    
    initGPIO();
    
    WiFi.mode(WIFI_STA);
    delay(500);
    
    if (rtc_inSleepCycle && rtcValid && wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
        digitalWrite(OPTO_SLEEP_PIN, HIGH);
        delay(100);
        
        initESPNow();
        uint32_t wakeStartTime = millis();
        uint32_t wakeDurationMs = CYCLE_WAKE_SECONDS * 1000;
        
        while (millis() - wakeStartTime < wakeDurationMs) {
            if (millis() - lastHeartbeat > 30000) {
                sendHeartbeat();
                BattCalcEnergy();
                sendBatteryLevel();
                sendDoorStatus();
                lastHeartbeat = millis();
            }
            delay(1000);
        }
        ContinueSleepCycle();
    }
    
    if (rtc_timedOpenActive && rtcValid) {
        initESPNow();
        uint32_t waitStart = millis();
        uint32_t waitDuration = rtc_remainingSeconds * 1000;
        while (millis() - waitStart < waitDuration) {
            if (millis() - lastHeartbeat > 30000) {
                sendHeartbeat();
                BattCalcEnergy();
                sendBatteryLevel();
                lastHeartbeat = millis();
            }
            delay(1000);
        }
        
        rtc_timedOpenActive = false;
        rtc_remainingSeconds = 0;
        updateRTCChecksum();
        
        sendTimedOpenNotification();
        delay(200);

        g_isTimedOpenExecution = true;
        OpenDoor();
        g_isTimedOpenExecution = false;
        
        if (pendingSleepCycle) {
            pendingSleepCycle = false;
            delay(500);
            EnterSleepCycle();
        }
        
    } else {
        initESPNow();
    }
}

void loop() {
    if (pendingSleepCycle) {
        pendingSleepCycle = false;
        EnterSleepCycle();
    }
    
    if (pendingTimedSleep) {
        pendingTimedSleep = false;
        TimedOpeningWithSleep(pendingTimedSleepMinutes);
    }
    
    if (pendingLowPowerSleep) {
        pendingLowPowerSleep = false;
        LowPowerSleep(pendingLowPowerSleepMinutes);
    }
    
    if (millis() - lastReconnectAttempt > 300000) {
        __espNowReady = false;  
        initESPNow();
        lastReconnectAttempt = millis();
    }
    
    if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
        sendHeartbeat();
        lastHeartbeat = millis();
    }
    
    static uint32_t lastStatusUpdate = 0;
    if (millis() - lastStatusUpdate > 120000) {
        BattCalcEnergy();
        sendBatteryLevel();
        sendDoorStatus();
        lastStatusUpdate = millis();
    }
    
    static bool lastDoorState = false;
    static uint32_t lastDoorCheck = 0;
    if (millis() - lastDoorCheck > 5000) {
        bool currentDoorState = readDoorSensor();
        if (currentDoorState != lastDoorState) {
            doorIsOpen = currentDoorState;
            sendDoorStatus();
        }
        lastDoorState = currentDoorState;
        lastDoorCheck = millis();
    }
    
    delay(100);
    digitalWrite(LED_READY, HIGH);
}