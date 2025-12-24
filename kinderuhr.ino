/* * PROJECT: Smart Kids Clock & Nightlight
 * HARDWARE: Waveshare ESP32-S3 Touch LCD 1.28 (Round)
 * * FEATURES:
 * - Day/Night Cycle (Sun/Moon) based on time
 * - Smart Home Integration via MQTT (Home Assistant)
 * - Battery Status & Charging Logic (keeps display on when plugged in)
 * - Touch Controls (Brightness Slider, Smart Button)
 * * REQUIRED LIBRARIES:
 * - Arduino_GFX_Library
 * - Adafruit GFX Library
 * - PubSubClient
 * - WiFi
 * - Preferences
 */

#include <Arduino.h> 
#include <WiFi.h>
#include <time.h>
#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <Adafruit_GFX.h> 
#include <Fonts/FreeSansBold24pt7b.h> 
#include <Fonts/FreeSansBold12pt7b.h> 
#include <Preferences.h> 
#include <PubSubClient.h> 
#include "USB.h" // Necessary for ESP32-S3 Native USB

/* ====================================================================
   ====================================================================
   ========      USER CONFIGURATION - EDIT THIS SECTION       =========
   ====================================================================
   ==================================================================== */

// --- 1. WIFI SETTINGS -----------------------------------------------
const char* ssid     = "DEIN_WLAN_NAME";      // <--- TODO: Dein WLAN Name
const char* password = "DEIN_WLAN_PASSWORT";  // <--- TODO: Dein WLAN Passwort

// --- 2. MQTT / HOME ASSISTANT SETTINGS ------------------------------
// Set to false if you don't use Home Assistant
#define USE_MQTT            true   

const char* mqtt_server = "192.168.1.XXX";    // <--- TODO: IP deines MQTT Brokers / Home Assistant
const int   mqtt_port   = 1883;               // Standard Port ist 1883
const char* mqtt_user   = "mqtt_user";        // <--- TODO: MQTT Benutzername
const char* mqtt_pass   = "mqtt_password";    // <--- TODO: MQTT Passwort

// Device Name in Home Assistant
const char* device_name = "Kinderuhr Smart";
const char* device_id   = "kinderuhr_esp32"; 

// --- 3. TIME SETTINGS -----------------------------------------------
// When does the day start (Sun) and end (Moon)?
const int TAG_START   = 8;   // 8:00 AM -> Sun appears
const int NACHT_START = 19;  // 7:00 PM -> Moon appears

// Timezone Settings (Default: Central Europe / Germany / Austria)
// POSIX format handles Daylight Saving Time automatically.
// For other zones, look up "POSIX TZ strings"
const char* ntpServer1 = "at.pool.ntp.org"; 
const char* ntpServer2 = "de.pool.ntp.org"; 
const char* time_zone  = "CET-1CEST,M3.5.0,M10.5.0/3"; 

// --- 4. DISPLAY TIMEOUTS --------------------------------------------
// Time in milliseconds before screen turns off (battery saving)
// 60000 = 60 Seconds
const unsigned long DISPLAY_TIMEOUT = 60000; 

// Time in milliseconds before the Smart Home Button closes automatically
const unsigned long HOME_TIMEOUT    = 10000; 

// --- 5. ADVANCED FEATURES -------------------------------------------
#define USE_WIFI            true   // Keep true for Time Sync
#define ENABLE_BUTTON_SCREEN true  // Swipe right for Smart Button?
#define NUTZE_USB_ERKENNUNG true   // Keep display on when USB is connected?

// --- 6. BATTERY CALIBRATION -----------------------------------------
// Adjust if percentage is off. 
// If displayed % is too high -> decrease value (e.g. 1.55)
// If displayed % is too low  -> increase value (e.g. 1.70)
#define VOLT_KALIBRIERUNG 1.627  

/* ====================================================================
   ========   END OF CONFIGURATION - DO NOT EDIT BELOW       ==========
   ==================================================================== */

// --- MQTT TOPICS ---
const char* topic_battery     = "uhr/status/battery";
const char* topic_charging    = "uhr/status/charging";
const char* topic_brightness  = "uhr/status/brightness"; 
const char* topic_set_bright  = "uhr/set/brightness";    
const char* topic_button      = "uhr/status/button"; 
const char* topic_display     = "uhr/status/display"; 

// --- HARDWARE PINS (Waveshare ESP32-S3 Touch LCD 1.28) ---
#define LCD_BL_PIN     5    
#define BAT_ADC_PIN    8    
#define TP_INT         4    
#define I2C_SDA_PIN    11   
#define I2C_SCL_PIN    10   
#define LCD_CS 21 
#define LCD_SCK 40 
#define LCD_D0 46  
#define LCD_D1 45  
#define LCD_D2 42  
#define LCD_D3 41  
#define I2C_ADDR_EXPANDER 0x20
#define I2C_ADDR_TOUCH    0x15

// --- BATTERY LIMITS ---
#define BAT_MAX_VOLT 4.15  
#define BAT_MIN_VOLT 3.30  

// --- COLORS (RGB565) ---
#define C_NIGHT_BG      0x1011  
#define C_DAY_BG        0x6D1F  
#define C_SUN_CORE      0xFFE0  
#define C_SUN_MID       0xFD20  
#define C_SUN_OUTER     0xFCA0  
#define C_MOON_BASE     0xF79E  
#define C_MOON_SHADOW   0xCE50  
#define C_STAR          0xFFFF  
#define C_TEXT          0xFFFF  
#define C_FACE          0x0000  
#define C_CHEEK         0xFBE4  
#define C_GREEN         0x07E0
#define C_RED           0xF800
#define C_CLOUD         0xFFFF
#define C_SLIDER_BG     0x3186 
#define C_MAGIC_BG      0x2015 
#define C_MAGIC_BTN     0xFA18 
#define C_MAGIC_BTN_ON  0xFFE0 
#define C_MAGIC_STAR    0xFFE0 

enum ScreenState { SCREEN_CLOCK, SCREEN_SMARTHOME };
ScreenState currentScreen = SCREEN_CLOCK;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCK, LCD_D0, LCD_D1, LCD_D2, LCD_D3);
Arduino_GFX *gfx = new Arduino_ST77916(bus, -1, 0, true, 360, 360);
Preferences preferences; 
WiFiClient espClient;
PubSubClient client(espClient); 

// GLOBAL VARIABLES
bool isNightMode = false;
bool firstRun = true;
bool showSlider = false; 
int currentBrightness = 255; 
unsigned long lastTouchTime = 0; 
unsigned long lastActivityTime = 0; 
bool isDisplayOn = true; 
int touchX = 0, touchY = 0;
int startTouchX = -1; 
int startTouchY = -1;

float globalSmoothedVolt = 4.0; 
int globalPercent = 0;      
bool globalCharging = false; 

// Logic for smart charging detection
unsigned long chargeKeepAliveTimer = 0; 
float voltage10sAgo = 0.0;              
unsigned long voltageTimer = 0;         

unsigned long lastMqttReconnect = 0;
bool buttonPressed = false; 

// PROTOTYPES
void drawDayScene(); void drawNightScene(); void drawSmartHomeScene(); void drawBatteryStatus(); 
void updateBatteryData(bool forceMqtt); void drawTime(String timeStr); float readBatteryVoltageRaw(); 
void setBacklight(int level); void updateBrightness(int level); bool reconnectMQTT(bool force); 
void callback(char* topic, byte* payload, unsigned int length); void sendAutoDiscovery(); 
void goToLightSleep(); void drawSlider(); void drawFace(int x, int y, bool isSleeping);

void initHardware() {
    pinMode(LCD_BL_PIN, OUTPUT); ledcAttach(LCD_BL_PIN, 5000, 8);
    pinMode(TP_INT, INPUT_PULLUP); 
    Wire.beginTransmission(I2C_ADDR_EXPANDER); Wire.write(0x03); Wire.write(0x00); Wire.endTransmission();
    Wire.beginTransmission(I2C_ADDR_EXPANDER); Wire.write(0x01); Wire.write(0x00); Wire.endTransmission();
    delay(50); Wire.beginTransmission(I2C_ADDR_EXPANDER); Wire.write(0x01); Wire.write(0xFF); Wire.endTransmission();
    delay(200); Wire.beginTransmission(I2C_ADDR_TOUCH); Wire.endTransmission();
}

bool readTouchCoordinates() {
    Wire.beginTransmission(I2C_ADDR_TOUCH); Wire.write(0x00); Wire.endTransmission();
    Wire.requestFrom(I2C_ADDR_TOUCH, 7);
    if (Wire.available()) {
        byte mode = Wire.read(); byte gesture = Wire.read(); byte fingers = Wire.read();
        byte xHigh = Wire.read(); byte xLow = Wire.read(); byte yHigh = Wire.read(); byte yLow = Wire.read();
        int tempX = ((xHigh & 0x0F) << 8) | xLow; int tempY = ((yHigh & 0x0F) << 8) | yLow;
        if (fingers > 0 && tempX < 400 && tempY < 400) { touchX = tempX; touchY = tempY; return true; }
    }
    return false;
}

float readBatteryVoltageRaw() {
    long sum = 0; for(int i=0; i<20; i++) { sum += analogRead(BAT_ADC_PIN); delay(1); }
    return (sum / 20.0 / 4095.0) * 3.3 * 2.0 * VOLT_KALIBRIERUNG;
}

// --- SMART CHARGING LOGIC (Reality Check V3) ---
void updateBatteryData(bool forceMqtt) {
    bool hardwareSerialActive = (NUTZE_USB_ERKENNUNG && Serial); 
    float currentRaw = readBatteryVoltageRaw();
    
    // 1. Heavy Smoothing (0.80) to ignore noise
    globalSmoothedVolt = (globalSmoothedVolt * 0.80) + (currentRaw * 0.20);

    // 2. Trend Recording (Every 10s)
    if (millis() - voltageTimer > 10000) {
        voltage10sAgo = globalSmoothedVolt; 
        voltageTimer = millis();
    }

    // --- DECISION: ARE WE CHARGING? ---
    bool detectCharge = false;

    // A) USB Data connected
    if (hardwareSerialActive) detectCharge = true;

    // B) Voltage physically high (Failsafe)
    else if (globalSmoothedVolt > 4.05) detectCharge = true;

    // C) Positive Voltage Trend
    else if (globalSmoothedVolt > (voltage10sAgo + 0.02)) detectCharge = true;

    // --- CHARGE KEEPER (Buffer) ---
    if (detectCharge) {
        chargeKeepAliveTimer = millis() + 60000; // 60s buffer
        globalCharging = true;
    } else {
        if (millis() > chargeKeepAliveTimer) {
            globalCharging = false;
        }
    }
    
    // --- REALITY CHECK (Emergency Stop) ---
    // If voltage drops significantly, cable was pulled.
    if (currentRaw < (globalSmoothedVolt - 0.05)) {
        globalCharging = false;
        chargeKeepAliveTimer = 0;
    }

    // Calculate Percentage
    int oldPercent = globalPercent;
    
    if (globalSmoothedVolt >= BAT_MAX_VOLT) globalPercent = 100;
    else if (globalSmoothedVolt <= BAT_MIN_VOLT) globalPercent = 0;
    else { globalPercent = (int)((globalSmoothedVolt - BAT_MIN_VOLT) / (BAT_MAX_VOLT - BAT_MIN_VOLT) * 100.0); }

    // --- REALITY CHECK 2 ---
    // If percentage DROPS, we are definitely not charging.
    if (globalPercent < oldPercent) {
        globalCharging = false;
        chargeKeepAliveTimer = 0;
    }

    // While charging, keep display awake
    if (globalCharging) {
        lastActivityTime = millis();
    }

    if (forceMqtt || globalPercent != oldPercent) {
        if (USE_MQTT && client.connected()) {
            client.publish(topic_battery, String(globalPercent).c_str(), true);
            client.publish(topic_charging, globalCharging ? "ON" : "OFF", true);
            client.publish(topic_brightness, String(currentBrightness).c_str(), true);
            client.publish(topic_display, isDisplayOn ? "ON" : "OFF", true);
        }
    }
}

void callback(char* topic, byte* payload, unsigned int length) {
  if (!USE_MQTT) return;
  String message; for (int i = 0; i < length; i++) message += (char)payload[i];
  if (String(topic) == topic_set_bright) {
      int newVal = message.toInt(); currentBrightness = newVal;
      if (isDisplayOn) updateBrightness(newVal);
      preferences.putInt("bright", currentBrightness); 
  }
}

void sendAutoDiscovery() {
    if (!USE_MQTT) return;
    client.publish("homeassistant/sensor/kinderuhr/battery/config", ("{\"name\": \"Batterie\", \"state_topic\": \"" + String(topic_battery) + "\", \"unit_of_measurement\": \"%\", \"device_class\": \"battery\", \"unique_id\": \"uhr_bat\", \"device\": {\"identifiers\": [\"" + String(device_id) + "\"], \"name\": \"" + String(device_name) + "\", \"manufacturer\": \"Eigenbau\", \"model\": \"ESP32 Round\"}}").c_str(), true);
    client.publish("homeassistant/binary_sensor/kinderuhr/charging/config", ("{\"name\": \"Lädt\", \"state_topic\": \"" + String(topic_charging) + "\", \"payload_on\": \"ON\", \"payload_off\": \"OFF\", \"device_class\": \"battery_charging\", \"unique_id\": \"uhr_chg\", \"device\": {\"identifiers\": [\"" + String(device_id) + "\"]}}").c_str(), true);
    client.publish("homeassistant/number/kinderuhr/brightness/config", ("{\"name\": \"Helligkeit\", \"command_topic\": \"" + String(topic_set_bright) + "\", \"state_topic\": \"" + String(topic_brightness) + "\", \"min\": 10, \"max\": 255, \"unique_id\": \"uhr_bri\", \"device\": {\"identifiers\": [\"" + String(device_id) + "\"]}}").c_str(), true);
    if (ENABLE_BUTTON_SCREEN) client.publish("homeassistant/binary_sensor/kinderuhr/button/config", ("{\"name\": \"Nachtlicht Schalter\", \"state_topic\": \"" + String(topic_button) + "\", \"payload_on\": \"PRESS\", \"off_delay\": 1, \"unique_id\": \"uhr_btn\", \"device\": {\"identifiers\": [\"" + String(device_id) + "\"]}}").c_str(), true);
    client.publish("homeassistant/binary_sensor/kinderuhr/display/config", ("{\"name\": \"Display\", \"state_topic\": \"" + String(topic_display) + "\", \"payload_on\": \"ON\", \"payload_off\": \"OFF\", \"icon\": \"mdi:monitor\", \"unique_id\": \"uhr_disp\", \"device\": {\"identifiers\": [\"" + String(device_id) + "\"]}}").c_str(), true);
}

bool reconnectMQTT(bool force) {
  if (!USE_MQTT) return false;
  if (client.connected()) return true;
  if (force || (millis() - lastMqttReconnect > 5000)) {
     lastMqttReconnect = millis();
     if(WiFi.status() != WL_CONNECTED) {
         WiFi.reconnect(); unsigned long startWifi = millis();
         while(WiFi.status() != WL_CONNECTED && millis() - startWifi < 1000) { delay(10); }
     }
     if (client.connect("ESP32_SmartWatch", mqtt_user, mqtt_pass)) {
         client.subscribe(topic_set_bright); sendAutoDiscovery(); updateBatteryData(true); return true;
     }
  }
  return false;
}

// --- GRAPHICS ---
void drawFace(int x, int y, bool isSleeping) {
  uint16_t faceColor = isNightMode ? C_MOON_BASE : C_SUN_CORE;
  if (isSleeping) {
    gfx->drawCircle(x-20, y-5, 8, C_FACE); gfx->drawCircle(x-20, y-6, 8, C_FACE); gfx->fillRect(x-30, y-15, 20, 10, faceColor); 
    gfx->drawCircle(x+20, y-5, 8, C_FACE); gfx->drawCircle(x+20, y-6, 8, C_FACE); gfx->fillRect(x+10, y-15, 20, 10, faceColor);
  } else {
    gfx->fillCircle(x-20, y-10, 8, C_FACE); gfx->fillCircle(x+20, y-10, 8, C_FACE); 
    gfx->fillCircle(x-22, y-12, 2, C_STAR); gfx->fillCircle(x+18, y-12, 2, C_STAR);
  }
  gfx->fillCircle(x-35, y+5, 9, C_CHEEK); gfx->fillCircle(x+35, y+5, 9, C_CHEEK);
  if (isSleeping) gfx->fillCircle(x, y+15, 4, C_FACE); else { gfx->fillCircle(x, y+10, 13, C_FACE); gfx->fillCircle(x, y+6, 13, faceColor); }
}

void drawDayScene() {
  if (!isDisplayOn) return;
  gfx->fillScreen(C_DAY_BG); int cx = 180; int cy = 190; int r = 85; 
  gfx->fillCircle(70, 90, 25, C_CLOUD); gfx->fillCircle(95, 80, 20, C_CLOUD); gfx->fillCircle(115, 90, 25, C_CLOUD);
  gfx->fillCircle(250, 100, 20, C_CLOUD); gfx->fillCircle(275, 90, 25, C_CLOUD); gfx->fillCircle(300, 100, 20, C_CLOUD);
  gfx->fillCircle(cx, cy, r, C_SUN_OUTER); gfx->fillCircle(cx, cy, r-10, C_SUN_MID); gfx->fillCircle(cx, cy, r-20, C_SUN_CORE);
  for (int i = 0; i < 360; i += 45) {
    float angle = i * 3.14159 / 180.0;
    gfx->fillTriangle(cx + cos(angle)*(r+20), cy + sin(angle)*(r+20), cx + cos(angle-0.2)*r, cy + sin(angle-0.2)*r, cx + cos(angle+0.2)*r, cy + sin(angle+0.2)*r, C_SUN_OUTER);
  }
  drawFace(cx, cy, false);
}

void drawNightScene() {
  if (!isDisplayOn) return;
  gfx->fillScreen(C_NIGHT_BG);
  int stars[][3] = {{50,60,3}, {300,80,4}, {80,220,2}, {280,250,3}, {180,40,2}, {20, 150,3}, {340, 180,2}};
  for(int i=0; i<7; i++) gfx->fillCircle(stars[i][0], stars[i][1], stars[i][2], C_STAR); 
  int cx = 180; int cy = 155; int r = 70;
  gfx->fillCircle(cx, cy, r, C_MOON_BASE);
  gfx->fillCircle(cx-30, cy-30, 8, C_MOON_SHADOW); gfx->fillCircle(cx+10, cy+40, 10, C_MOON_SHADOW);
  gfx->fillCircle(cx-50, cy+20, 6, C_MOON_SHADOW); gfx->fillCircle(cx+40, cy-10, 12, C_MOON_SHADOW);
  drawFace(cx, cy, true);
  gfx->setFont(&FreeSansBold12pt7b); gfx->setTextSize(1); gfx->setTextColor(0xEF5D); gfx->setCursor(cx+65, cy-35); gfx->print("Zzz");
}

void drawSmartHomeScene() {
  gfx->fillScreen(C_MAGIC_BG); 
  gfx->fillCircle(50, 80, 2, C_MAGIC_STAR); gfx->fillCircle(300, 120, 3, C_MAGIC_STAR);
  gfx->fillCircle(120, 280, 2, C_MAGIC_STAR); gfx->fillCircle(250, 60, 2, C_MAGIC_STAR); gfx->fillCircle(40, 200, 3, C_MAGIC_STAR);
  uint16_t btnColor = buttonPressed ? C_MAGIC_BTN_ON : C_MAGIC_BTN;
  int btnX = 180; int btnY = 180; int btnR = 85;
  gfx->fillCircle(btnX, btnY, btnR, btnColor);
  if (buttonPressed) { gfx->fillCircle(btnX, btnY, btnR-10, 0xFFF0); gfx->drawCircle(btnX, btnY, btnR, C_MAGIC_STAR); } 
  else { gfx->fillCircle(btnX-10, btnY-10, 35, 0xFFFF); gfx->fillCircle(btnX+5, btnY-15, 30, btnColor); gfx->setFont(NULL); gfx->setTextSize(2); gfx->setTextColor(0xFFFF); gfx->setCursor(btnX+20, btnY+10); gfx->print("Zzz"); }
}

void drawTime(String timeStr) {
  if (!isDisplayOn || currentScreen != SCREEN_CLOCK) return; 
  uint16_t bg = isNightMode ? C_NIGHT_BG : C_DAY_BG;
  if (!showSlider) {
      gfx->fillRect(0, 290, 360, 50, bg); gfx->setFont(&FreeSansBold24pt7b); gfx->setTextColor(C_TEXT); gfx->setTextSize(1); 
      int16_t x1, y1; uint16_t w, h; gfx->getTextBounds(timeStr, 0, 0, &x1, &y1, &w, &h);
      int xPosStart = (360-w)/2 - 3; int yPos = 320 + 5; 
      gfx->setCursor(xPosStart, yPos); gfx->print(timeStr[0]); gfx->setCursor(gfx->getCursorX(), yPos); gfx->print(timeStr[1]); 
      gfx->setCursor(gfx->getCursorX()+4, yPos); gfx->print(':'); gfx->setCursor(gfx->getCursorX()+4, yPos); gfx->print(timeStr[3]); 
      gfx->setCursor(gfx->getCursorX(), yPos); gfx->print(timeStr[4]);
  }
}

void drawBatteryStatus() {
    if (!isDisplayOn || currentScreen != SCREEN_CLOCK || showSlider) return; 
    gfx->setTextSize(1); uint16_t bgColor = isNightMode ? C_NIGHT_BG : C_DAY_BG;
    gfx->fillRect(100, 20, 160, 45, bgColor); gfx->setFont(&FreeSansBold12pt7b); 
    char statusStr[15]; 
    if (globalCharging) { sprintf(statusStr, "Laden %d%%", globalPercent); gfx->setTextColor(C_GREEN, bgColor); } 
    else { sprintf(statusStr, "%d%%", globalPercent); gfx->setTextColor(C_TEXT, bgColor); }
    int textX = globalCharging ? 100 : 130; gfx->setCursor(textX, 45); gfx->print(statusStr);
    int x_sym = 190; if (globalCharging) x_sym = 220; 
    gfx->drawRect(x_sym, 28, 25, 16, C_STAR); 
    uint16_t fillerColor = (globalCharging) ? C_GREEN : ((globalPercent<20) ? C_RED : C_TEXT);
    gfx->fillRect(x_sym+2, 30, map(globalPercent, 0, 100, 0, 21), 12, fillerColor); gfx->fillRect(x_sym+25, 32, 3, 8, C_STAR);
}

void setBacklight(int level) { ledcWrite(LCD_BL_PIN, (level<0)?0:(level>255?255:level)); }

void updateBrightness(int level) {
    if (level<10) level=10; if (level>255) level=255; currentBrightness = level;
    if (isDisplayOn) setBacklight(currentBrightness);
    if (USE_MQTT && client.connected()) client.publish(topic_brightness, String(currentBrightness).c_str(), true);
}

void drawSlider() {
    if (!isDisplayOn) return;
    const int SLIDER_X = 260; const int SLIDER_Y_TOP = 60; const int SLIDER_H = 240; const int SLIDER_W = 40; 
    uint16_t bgColor = isNightMode ? C_NIGHT_BG : C_DAY_BG;
    gfx->fillRect(SLIDER_X-10, SLIDER_Y_TOP-20, SLIDER_W+20, SLIDER_H+40, bgColor);
    gfx->fillRoundRect(SLIDER_X, SLIDER_Y_TOP, SLIDER_W, SLIDER_H, 15, C_SLIDER_BG);
    int fillHeight = map(currentBrightness, 0, 255, 0, SLIDER_H);
    gfx->fillRoundRect(SLIDER_X, (SLIDER_Y_TOP+SLIDER_H)-fillHeight, SLIDER_W, fillHeight, 15, C_SUN_CORE);
    gfx->drawRoundRect(SLIDER_X, SLIDER_Y_TOP, SLIDER_W, SLIDER_H, 15, C_TEXT);
    gfx->setFont(NULL); gfx->setCursor(SLIDER_X+5, SLIDER_Y_TOP-15); gfx->setTextColor(C_TEXT, bgColor); gfx->print("LICHT");
}

void goToLightSleep() {
    Serial.println("Go Sleep...");
    isDisplayOn = false; setBacklight(0); showSlider = false;
    if(USE_MQTT && client.connected()) { client.publish(topic_display, "OFF", true); client.loop(); delay(200); }
    esp_sleep_enable_ext0_wakeup((gpio_num_t)TP_INT, 0);
    esp_light_sleep_start();
    Serial.println("Wake Up!");
    isDisplayOn = true; lastActivityTime = millis(); lastMqttReconnect = 0; 
    updateBatteryData(false); setBacklight(currentBrightness);
    if(USE_MQTT && client.connected()) client.publish(topic_display, "ON", true);
    // WLAN Check beim Aufwachen
    if(WiFi.status()!=WL_CONNECTED) Serial.println("Wifi reconnecting...");
    
    if (currentScreen == SCREEN_SMARTHOME) drawSmartHomeScene(); 
    else { if (isNightMode) drawNightScene(); else drawDayScene(); drawBatteryStatus(); 
           struct tm timeinfo; if (getLocalTime(&timeinfo)) { char timeStr[6]; sprintf(timeStr, "%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min); drawTime(timeStr); } }
}

void setup() {
    Serial.begin(115200); delay(1000); 
    // WLAN Power Max
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    
    preferences.begin("my-app", false); currentBrightness = preferences.getInt("bright", 255);
    analogReadResolution(12); analogSetAttenuation(ADC_11db); pinMode(BAT_ADC_PIN, INPUT); 
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); delay(100); initHardware(); updateBrightness(currentBrightness); 
    if (!gfx->begin()) { while(1); }
    updateBatteryData(false); gfx->fillScreen(C_DAY_BG); 
    if (USE_WIFI) {
       WiFi.setSleep(false); // Kein WLAN Sleep!
       WiFi.begin(ssid, password); int retry=0; while (WiFi.status()!=WL_CONNECTED && retry<10) { delay(500); retry++; }
       if (USE_MQTT) { client.setServer(mqtt_server, mqtt_port); client.setCallback(callback); client.setBufferSize(512); }
       configTzTime(time_zone, ntpServer1, ntpServer2);
    }
    firstRun = true; lastActivityTime = millis(); voltage10sAgo = readBatteryVoltageRaw();
}

void loop() {
    static int lastMinute = -1; static unsigned long lastBatteryUpdate = 0; unsigned long currentMillis = millis();

    // Batterie alle 2 Sek
    if (currentMillis - lastBatteryUpdate >= 2000 || firstRun) {
        updateBatteryData(false);
        if (isDisplayOn && !showSlider) drawBatteryStatus(); 
        lastBatteryUpdate = currentMillis;
    }

    bool preventSleep = false;
    // Wenn Lade-Modus aktiv ist -> Nicht schlafen
    if (globalCharging) preventSleep = true;

    // Timer resetten wenn am Strom
    if (preventSleep) lastActivityTime = currentMillis;

    // Schlafen gehen
    if (!preventSleep && isDisplayOn && (currentMillis - lastActivityTime > DISPLAY_TIMEOUT)) {
        goToLightSleep(); return; 
    }

    // WLAN / MQTT Loop
    if (USE_WIFI && WiFi.status() == WL_CONNECTED) {
       if (USE_MQTT && !client.connected()) reconnectMQTT(false); 
       if (USE_MQTT) client.loop(); 
    }

    // Touch Logik
    bool currentlyTouching = readTouchCoordinates();
    if (currentlyTouching) {
        lastActivityTime = currentMillis; 
        if (startTouchX == -1) { startTouchX = touchX; startTouchY = touchY; }
        if (currentScreen == SCREEN_CLOCK) {
            int deltaX = startTouchX - touchX;
            if (ENABLE_BUTTON_SCREEN && startTouchX > 200 && deltaX > 80 && !showSlider) {
                currentScreen = SCREEN_SMARTHOME; drawSmartHomeScene(); startTouchX = -1; return; 
            }
            if (!showSlider && touchY < 80 && touchX > 100 && touchX < 260 && deltaX < 20) {
                 showSlider = true; lastTouchTime = currentMillis; drawSlider();
            }
            if (showSlider) {
                lastTouchTime = currentMillis;
                if (touchX > 240) { 
                    int val = constrain(touchY, 60, 300);
                    int newB = map(val, 60, 300, 255, 10);
                    updateBrightness(newB); drawSlider(); 
                }
            }
        } else if (currentScreen == SCREEN_SMARTHOME) {
            if (sqrt(pow(touchX-180, 2) + pow(touchY-180, 2)) < 85) { 
                if (!buttonPressed) {
                    buttonPressed = true; drawSmartHomeScene(); 
                    if (USE_MQTT) {
                        // Aggressive Reconnect bei Button-Press
                        if (!client.connected()) { for(int i=0; i<3; i++) { if(reconnectMQTT(true)) break; delay(100); } }
                        if (client.connected()) client.publish(topic_button, "PRESS");
                    }
                }
            }
        }
    } else {
        startTouchX = -1; startTouchY = -1;
        if (buttonPressed) { buttonPressed = false; drawSmartHomeScene(); }
    }
    
    // Auto-Return zur Uhr
    if (isDisplayOn && currentScreen == SCREEN_SMARTHOME && (currentMillis - lastActivityTime > HOME_TIMEOUT)) {
        currentScreen = SCREEN_CLOCK; if (isNightMode) drawNightScene(); else drawDayScene(); drawBatteryStatus(); lastMinute = -1; 
    }
    
    // Slider Timeout
    if (showSlider && (currentMillis - lastTouchTime > 3000)) { 
        showSlider = false; preferences.putInt("bright", currentBrightness);
        if (isNightMode) drawNightScene(); else drawDayScene(); lastMinute = -1; drawBatteryStatus();
    }

    // Uhrzeit Update
    if (currentScreen == SCREEN_CLOCK) {
        struct tm timeinfo;
        if (USE_WIFI && getLocalTime(&timeinfo)) {
          int h = timeinfo.tm_hour; int m = timeinfo.tm_min; bool currentIsNight = (h >= NACHT_START || h < TAG_START);
          if ((currentIsNight != isNightMode || firstRun) && !showSlider) {
            isNightMode = currentIsNight; firstRun = false;
            if (isNightMode) drawNightScene(); else drawDayScene(); lastMinute = -1; 
          }
          if (m != lastMinute && !showSlider) { 
              char timeStr[6]; sprintf(timeStr, "%02d:%02d", h, m); drawTime(timeStr); lastMinute = m; 
          }
        } else if (!USE_WIFI && firstRun) { drawDayScene(); drawBatteryStatus(); firstRun = false; }
    }
    delay(20); 
}
