/*
  ESP32 API-only robot controller for local React web app
  -------------------------------------------------------
  This version does NOT host the full UI.
  It only hosts:
  - AP mode page
  - API endpoints for the React app

  Local React app on laptop calls:
  GET http://192.168.4.1/api/status
  GET http://192.168.4.1/api/serve?table=2
  GET http://192.168.4.1/api/stop

  Wiring:
  ESP32 IO17 TX  -> STM32 PA10 RX
  ESP32 IO18 RX  <- STM32 PA9 TX
  ESP32 GND      <-> STM32 GND

  UART route format sent to STM32:
  ROUTE:2,0,3,0,1,4\n

  STM32 command values:
  0 = STRAIGHT
  1 = LEFT
  2 = RIGHT
  3 = SERVE_STOP
  4 = DOCK
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include "HX711.h"
#include <Adafruit_NeoPixel.h>
#include <math.h>

#define ESP_UART_RX_PIN 18
#define ESP_UART_TX_PIN 17
#define STM_BAUD        115200

#define AP_SSID         "RestaurantRobot_AP"
#define AP_PASSWORD     "12345678"

IPAddress apLocalIp(192, 168, 4, 1);
IPAddress apGateway(192, 168, 4, 1);
IPAddress apSubnet(255, 255, 255, 0);

HardwareSerial STM32Serial(1);
WebServer server(80);
Preferences prefs;

String wifiSsid = "";
String wifiPass = "";
String lastRoute = "None";
String lastReply = "--";
String robotState = "Ready";
String selectedTable = "None";
unsigned long lastCommandTime = 0;

/* Tray-1 HX711 weight sensor + API calibration */
#define HX711_DOUT  5
#define HX711_SCK   20
#define FOOD_THRESHOLD_G 300.0f
#define FOOD_CLEAR_CONFIRM_COUNT 4   // kept for old status/debug compatibility
#define FOOD_EMPTY_CONFIRM_MS 10000UL  // food must stay below threshold for 10 seconds before return
#define WEIGHT_READ_INTERVAL_MS 500

#define BUZZER_PIN 48
#define FOOD_ALERT_INTERVAL_MS 5000
#define FOOD_ALERT_TONE_HZ 2200
#define FOOD_ALERT_TONE_MS 140

/* Ultrasonic obstacle sensors */
#define FRONT_TRIG_PIN 15
#define FRONT_ECHO_PIN 16
#define LEFT_TRIG_PIN  13
#define LEFT_ECHO_PIN  14
#define RIGHT_TRIG_PIN 11
#define RIGHT_ECHO_PIN 12

#define FRONT_OBSTACLE_CM 15.0f
#define SIDE_CLOSE_CM 28.0f
#define SIDE_FAST_APPROACH_DELTA_CM 5.0f
#define OBSTACLE_MIN_STOP_MS 5000
#define OBSTACLE_CLEAR_CONFIRM_MS 1200
#define OBSTACLE_HELP_MS 20000
#define OBSTACLE_ALERT_INTERVAL_MS 700

/* Power / dock sensing
   IO41: charger/pogo detector. Your measured values: connected ~= 3.0 V, not connected ~= 1.4 V.
   IO8 : battery divider ADC. Your measured values: 11.4 V battery -> 2.82 V at IO8.
*/
#define CHARGER_DETECT_PIN 41
#define BATTERY_ADC_PIN    8

#define CHARGER_CONNECTED_THRESHOLD_V 2.20f
#define POWER_READ_INTERVAL_MS        500UL

#define BATTERY_DIVIDER_FACTOR        (11.4f / 2.82f)  // ~= 4.043
#define BATTERY_FULL_V                12.50f
#define BATTERY_LOW_V                 10.50f

/* Software-only battery stabilizer.
   This does not fix the electrical noise, but it prevents the web app from
   jumping between false values like 100% and 4%.

   Method:
   1) Take many ADC samples and use the median.
   2) Keep a rolling median history.
   3) Ignore impossible jumps.
   4) When not charging, voltage is allowed to fall faster than rise.
      When charging, voltage is allowed to rise faster than fall.
*/
#define BATTERY_ADC_SAMPLES           31
#define BATTERY_HISTORY_COUNT         15
#define BATTERY_MIN_VALID_PACK_V      9.00f
#define BATTERY_MAX_VALID_PACK_V      13.20f
#define BATTERY_SPIKE_REJECT_V        0.85f
#define BATTERY_NOT_CHG_RISE_STEP_V   0.004f
#define BATTERY_NOT_CHG_FALL_STEP_V   0.025f
#define BATTERY_CHG_RISE_STEP_V       0.025f
#define BATTERY_CHG_FALL_STEP_V       0.004f


/* WS2812B decoration LEDs
   GPIO36 has one chained strip: first 88 LEDs = right side, next 88 LEDs = left side.
   GPIO37 is an optional extra decoration channel. If not connected, it is harmless.
   Install Arduino library: Adafruit NeoPixel.
*/
#define LED_PIN_36                36
#define LED_PIN_37                37
#define LED_SIDE_COUNT            88
#define LED36_RIGHT_START         0
#define LED36_LEFT_START          LED_SIDE_COUNT
#define LED36_TOTAL_COUNT         (LED_SIDE_COUNT * 2)
#define LED37_COUNT               88
#define LED_UPDATE_INTERVAL_MS    25UL

#define LED_PATTERN_FIXED         0
#define LED_PATTERN_RUNNING       1
#define LED_PATTERN_RANDOM        2
#define LED_PATTERN_RAINBOW       3
#define LED_PATTERN_BREATHING     4


HX711 trayScale;

/*
  HX711 library formula:
  weight_g = (raw_average - offset) / calibration_factor

  With your previous readings:
  empty raw ~= -530200
  1050g raw ~= -555600
  factor ~= (-555600 - -530200) / 1050 = -24.19
*/
float calibration_factor = -25.61f;
long trayOffsetRaw = 0;
bool trayOffsetSaved = false;
bool trayCalibrationSaved = false;

float trayWeightG = 0.0f;
long trayRawAverage = 0;
long trayValueRaw = 0;
bool trayReady = false;
bool trayHasFood = false;
bool servingWaitingForFood = false;
bool foodTakenSentThisServe = false;
uint8_t foodClearCounter = 0;
unsigned long foodEmptySinceMs = 0;
unsigned long lastWeightReadMs = 0;
unsigned long lastFoodAlertMs = 0;

float frontDistanceCm = -1;
float leftDistanceCm = -1;
float rightDistanceCm = -1;
float prevLeftDistanceCm = -1;
float prevRightDistanceCm = -1;

bool frontSensorEnabled = true;
bool leftSensorEnabled = true;
bool rightSensorEnabled = true;
uint32_t obstacleBypassUntilMs = 0;
bool obstaclePaused = false;
bool obstacleHelpNeeded = false;
String obstacleSensor = "none";
String obstacleMessage = "";
uint32_t obstacleStartMs = 0;
uint32_t obstacleClearStartMs = 0;
uint32_t lastObstacleAlertMs = 0;
uint32_t lastUltrasonicReadMs = 0;

int chargerAdcRaw = 0;
float chargerSenseVoltage = 0.0f;
bool chargerConnected = false;
uint8_t chargerConnectedCounter = 0;
uint8_t chargerDisconnectedCounter = 0;
bool chargerOnSentToSTM = false;

int batteryAdcRaw = 0;
float batteryPinVoltage = 0.0f;
float batteryVoltage = 0.0f;       // stable value used by the web app
float batteryVoltageRaw = 0.0f;    // latest raw/median ADC result before smart filtering
float batteryVoltageCandidate = 0.0f;
int batteryPercent = 0;
bool batteryLow = false;
bool batteryFilterReady = false;
bool batteryReadingUnstable = false;
uint8_t batteryLargeJumpCounter = 0;
uint8_t batteryHistoryIndex = 0;
uint8_t batteryHistoryCount = 0;
float batteryHistory[BATTERY_HISTORY_COUNT];
uint32_t lastPowerReadMs = 0;


struct LedRuntimeConfig
{
  bool enabled;
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t brightness;
  uint8_t pattern;
  uint8_t speed;
};

Adafruit_NeoPixel ledStrip36(LED36_TOTAL_COUNT, LED_PIN_36, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ledStrip37(LED37_COUNT, LED_PIN_37, NEO_GRB + NEO_KHZ800);

LedRuntimeConfig ledRight = { true, 40, 246, 255, 70, LED_PATTERN_FIXED, 45 };
LedRuntimeConfig ledLeft  = { true, 40, 246, 255, 70, LED_PATTERN_FIXED, 45 };
LedRuntimeConfig ledExtra = { false, 255, 79, 227, 60, LED_PATTERN_RUNNING, 45 };
bool ledLinkSides = true;
uint32_t lastLedUpdateMs = 0;
uint32_t ledFrame = 0;


/* Live journey state sent to the React UI */
bool journeyActive = false;
int activeTable = 0;
String currentEvent = "IDLE";
uint32_t currentEventSeq = 0;
unsigned long journeyStartMs = 0;
unsigned long lastEventMs = 0;

/* WiFi reconnection watchdog */
unsigned long lastWiFiCheckMs = 0;

/* Keep recent events so React does not miss fast back-to-back updates. */
#define EVENT_HISTORY_MAX 40

struct RobotEventItem
{
  uint32_t seq;
  String event;
  unsigned long ms;
};

RobotEventItem eventHistory[EVENT_HISTORY_MAX];
uint8_t eventHistoryCount = 0;

bool sendToSTM32WaitAck(String line, const String &expectedAck, uint8_t retries, uint32_t timeoutMs);

String routeForTable(int table)
{
  switch (table)
  {
    case 1:
      return "ROUTE:0,3,0,4";
    case 2:
      return "ROUTE:2,2,3,1,1,4";   // Table 2: right, right, stop, left, left, dock
    case 3:
      return "ROUTE:2,0,3,0,1,4";   // Your tested Table 3 route
    default:
      return "";
  }
}


int getIntArgClamped(const String &name, int defaultValue, int minValue, int maxValue)
{
  if (!server.hasArg(name))
  {
    return defaultValue;
  }

  int value = server.arg(name).toInt();
  if (value < minValue) value = minValue;
  if (value > maxValue) value = maxValue;
  return value;
}

bool waitHx711Ready(uint32_t timeoutMs = 1500)
{
  uint32_t start = millis();

  while (millis() - start < timeoutMs)
  {
    if (trayScale.is_ready())
    {
      trayReady = true;
      return true;
    }

    delay(2);
  }

  trayReady = false;
  return false;
}

void loadHx711Calibration()
{
  prefs.begin("hx711", true);
  calibration_factor = prefs.getFloat("scale", calibration_factor);
  trayOffsetRaw = prefs.getLong("offset", 0);
  trayCalibrationSaved = prefs.getBool("scaleSaved", false);
  trayOffsetSaved = prefs.getBool("offsetSaved", false);
  prefs.end();
}

void saveHx711Scale()
{
  prefs.begin("hx711", false);
  prefs.putFloat("scale", calibration_factor);
  prefs.putBool("scaleSaved", true);
  prefs.end();
  trayCalibrationSaved = true;
}

void saveHx711Offset()
{
  prefs.begin("hx711", false);
  prefs.putLong("offset", trayOffsetRaw);
  prefs.putBool("offsetSaved", true);
  prefs.end();
  trayOffsetSaved = true;
}

void applyHx711Calibration()
{
  trayScale.set_scale(calibration_factor);
  trayScale.set_offset(trayOffsetRaw);
}

float weightFromRawDiff(long valueRaw)
{
  /*
    Four half-bridge orientation can make raw move either direction when load is added.
    For tray food detection, load magnitude matters. So:
      weight_g = abs(raw - tare_offset) / abs(scale)
  */
  float scaleAbs = fabs(calibration_factor);
  if (scaleAbs < 0.000001f)
  {
    return 0.0f;
  }

  return fabs((float)valueRaw) / scaleAbs;
}

String weightJson(long raw, int times)
{
  long valueRaw = raw - trayOffsetRaw;
  float signedWeightG = valueRaw / calibration_factor;
  float weightG = weightFromRawDiff(valueRaw);
  bool hasFood = weightG >= FOOD_THRESHOLD_G;

  String json = "{";
  json += "\"ok\":true,";
  json += "\"times\":" + String(times) + ",";
  json += "\"raw\":" + String(raw) + ",";
  json += "\"offset\":" + String(trayOffsetRaw) + ",";
  json += "\"valueRaw\":" + String(valueRaw) + ",";
  json += "\"scale\":" + String(calibration_factor, 6) + ",";
  json += "\"weightG\":" + String(weightG, 2) + ",";
  json += "\"signedWeightG\":" + String(signedWeightG, 2) + ",";
  json += "\"rawDirection\":\"" + String(valueRaw >= 0 ? "positive" : "negative") + "\",";
  json += "\"thresholdG\":" + String(FOOD_THRESHOLD_G, 0) + ",";
  json += "\"hasFood\":" + String(hasFood ? "true" : "false") + ",";
  json += "\"trayReady\":" + String(trayReady ? "true" : "false") + ",";
  json += "\"offsetSaved\":" + String(trayOffsetSaved ? "true" : "false") + ",";
  json += "\"scaleSaved\":" + String(trayCalibrationSaved ? "true" : "false");
  json += "}";
  return json;
}

void updateTrayWeight()
{
  if (millis() - lastWeightReadMs < WEIGHT_READ_INTERVAL_MS)
  {
    return;
  }

  lastWeightReadMs = millis();

  if (!waitHx711Ready(1800))
  {
    trayReady = false;
    return;
  }

  trayReady = true;
  trayRawAverage = trayScale.read_average(5);
  trayValueRaw = trayRawAverage - trayOffsetRaw;
  trayWeightG = weightFromRawDiff(trayValueRaw);

  if (trayWeightG < 0 && trayWeightG > -30)
  {
    trayWeightG = 0;
  }

  trayHasFood = trayWeightG >= FOOD_THRESHOLD_G;
}

void checkFoodTakenRelease()
{
  if (!servingWaitingForFood || foodTakenSentThisServe)
  {
    foodEmptySinceMs = 0;
    foodClearCounter = 0;
    return;
  }

  updateTrayWeight();

  if (!trayReady)
  {
    return;
  }

  unsigned long now = millis();

  if (trayWeightG < FOOD_THRESHOLD_G)
  {
    if (foodEmptySinceMs == 0)
    {
      foodEmptySinceMs = now;
    }

    /* Debug/status counter only: counts 500 ms weight-read cycles. */
    if (foodClearCounter < 255)
    {
      foodClearCounter++;
    }
  }
  else
  {
    foodEmptySinceMs = 0;
    foodClearCounter = 0;
  }

  /* Return only if the tray has stayed empty continuously for 10 seconds. */
  if (foodEmptySinceMs != 0 && (now - foodEmptySinceMs) >= FOOD_EMPTY_CONFIRM_MS)
  {
    if (sendToSTM32WaitAck("FOOD_TAKEN", "OK:FOOD_TAKEN", 3, 700))
    {
      foodTakenSentThisServe = true;
      servingWaitingForFood = false;
      foodEmptySinceMs = 0;
      currentEventSeq++;
      currentEvent = "FOOD_TAKEN_CONFIRMED";
      addEventHistory(currentEventSeq, currentEvent);
      robotState = "Food empty for 10s - returning";
    }
    else
    {
      currentEventSeq++;
      currentEvent = "FOOD_TAKEN_ACK_TIMEOUT";
      addEventHistory(currentEventSeq, currentEvent);
      robotState = "Food empty for 10s, STM32 ACK missing";
    }
  }
}

float clampFloat(float value, float minVal, float maxVal)
{
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

void sortIntArray(int *arr, int n)
{
  for (int i = 1; i < n; i++)
  {
    int key = arr[i];
    int j = i - 1;
    while (j >= 0 && arr[j] > key)
    {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
}

void sortFloatArray(float *arr, int n)
{
  for (int i = 1; i < n; i++)
  {
    float key = arr[i];
    int j = i - 1;
    while (j >= 0 && arr[j] > key)
    {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
}

float readBatteryPinVoltageMedian()
{
  int rawSamples[BATTERY_ADC_SAMPLES];
  int mvSamples[BATTERY_ADC_SAMPLES];

  for (int i = 0; i < BATTERY_ADC_SAMPLES; i++)
  {
    rawSamples[i] = analogRead(BATTERY_ADC_PIN);
    mvSamples[i] = analogReadMilliVolts(BATTERY_ADC_PIN);
    delayMicroseconds(700);
  }

  sortIntArray(rawSamples, BATTERY_ADC_SAMPLES);
  sortIntArray(mvSamples, BATTERY_ADC_SAMPLES);

  batteryAdcRaw = rawSamples[BATTERY_ADC_SAMPLES / 2];

  float pinV = mvSamples[BATTERY_ADC_SAMPLES / 2] / 1000.0f;

  /* Fallback if analogReadMilliVolts returns invalid/zero. */
  if (pinV <= 0.02f && batteryAdcRaw > 0)
  {
    pinV = (batteryAdcRaw * 3.3f) / 4095.0f;
  }

  return pinV;
}

float getBatteryHistoryMedian(float newPackVoltage)
{
  batteryHistory[batteryHistoryIndex] = newPackVoltage;
  batteryHistoryIndex = (batteryHistoryIndex + 1) % BATTERY_HISTORY_COUNT;

  if (batteryHistoryCount < BATTERY_HISTORY_COUNT)
  {
    batteryHistoryCount++;
  }

  float temp[BATTERY_HISTORY_COUNT];
  for (int i = 0; i < batteryHistoryCount; i++)
  {
    temp[i] = batteryHistory[i];
  }

  sortFloatArray(temp, batteryHistoryCount);
  return temp[batteryHistoryCount / 2];
}

void updateStableBatteryVoltage(float rawPackVoltage)
{
  batteryVoltageRaw = rawPackVoltage;
  batteryReadingUnstable = false;

  if (rawPackVoltage < BATTERY_MIN_VALID_PACK_V || rawPackVoltage > BATTERY_MAX_VALID_PACK_V)
  {
    batteryReadingUnstable = true;
    return;
  }

  batteryVoltageCandidate = getBatteryHistoryMedian(rawPackVoltage);

  if (!batteryFilterReady)
  {
    batteryVoltage = batteryVoltageCandidate;
    batteryFilterReady = true;
    return;
  }

  float diff = batteryVoltageCandidate - batteryVoltage;

  float maxRise;
  float maxFall;

  if (chargerConnected)
  {
    /* While charging, the trusted trend is upward. */
    maxRise = BATTERY_CHG_RISE_STEP_V;
    maxFall = BATTERY_CHG_FALL_STEP_V;
  }
  else
  {
    /* While not charging, the trusted trend is downward. */
    maxRise = BATTERY_NOT_CHG_RISE_STEP_V;
    maxFall = BATTERY_NOT_CHG_FALL_STEP_V;
  }

  /* Reject single big jumps, but do not get stuck forever if the new value
     stays stable for many readings after startup. */
  if (fabsf(diff) > BATTERY_SPIKE_REJECT_V)
  {
    batteryReadingUnstable = true;

    if (batteryLargeJumpCounter < 255)
    {
      batteryLargeJumpCounter++;
    }

    /* After about 5 seconds of persistent difference, start moving slowly
       toward the candidate instead of permanently holding the old value. */
    if (batteryLargeJumpCounter < 10)
    {
      return;
    }
  }
  else
  {
    batteryLargeJumpCounter = 0;
  }

  if (diff > maxRise) diff = maxRise;
  if (diff < -maxFall) diff = -maxFall;

  batteryVoltage += diff;
}

void updatePowerSensors()
{
  if (lastPowerReadMs != 0 && millis() - lastPowerReadMs < POWER_READ_INTERVAL_MS)
  {
    return;
  }

  lastPowerReadMs = millis();

  chargerAdcRaw = analogRead(CHARGER_DETECT_PIN);
  chargerSenseVoltage = analogReadMilliVolts(CHARGER_DETECT_PIN) / 1000.0f;

  /* Fallback if analogReadMilliVolts is not calibrated yet. */
  if (chargerSenseVoltage <= 0.02f && chargerAdcRaw > 0)
  {
    chargerSenseVoltage = (chargerAdcRaw * 3.3f) / 4095.0f;
  }

  bool chargerNow = chargerSenseVoltage >= CHARGER_CONNECTED_THRESHOLD_V;

  if (chargerNow)
  {
    if (chargerConnectedCounter < 255) chargerConnectedCounter++;
    chargerDisconnectedCounter = 0;

    if (chargerConnectedCounter >= 3)
    {
      chargerConnected = true;
    }
  }
  else
  {
    if (chargerDisconnectedCounter < 255) chargerDisconnectedCounter++;
    chargerConnectedCounter = 0;

    if (chargerDisconnectedCounter >= 3)
    {
      chargerConnected = false;
      chargerOnSentToSTM = false;
    }
  }

  batteryPinVoltage = readBatteryPinVoltageMedian();
  float rawPackVoltage = batteryPinVoltage * BATTERY_DIVIDER_FACTOR;
  updateStableBatteryVoltage(rawPackVoltage);

  float percent = ((batteryVoltage - BATTERY_LOW_V) / (BATTERY_FULL_V - BATTERY_LOW_V)) * 100.0f;
  batteryPercent = (int)(clampFloat(percent, 0.0f, 100.0f) + 0.5f);
  batteryLow = batteryVoltage <= BATTERY_LOW_V;

  /* During dock parking, STM32 can finish docking from either Hall T marker
     or charger contact. Send CHARGER_ON once when contact is detected. */
  if (chargerConnected && !chargerOnSentToSTM)
  {
    sendToSTM32("CHARGER_ON");
    chargerOnSentToSTM = true;
  }
}

void playFoodWaitingBeep()
{
  tone(BUZZER_PIN, FOOD_ALERT_TONE_HZ, FOOD_ALERT_TONE_MS);
}

void updateFoodWaitingBuzzer()
{
  if (servingWaitingForFood && trayHasFood && !foodTakenSentThisServe)
  {
    if (lastFoodAlertMs == 0 || millis() - lastFoodAlertMs >= FOOD_ALERT_INTERVAL_MS)
    {
      lastFoodAlertMs = millis();
      playFoodWaitingBeep();
    }
  }
  else
  {
    lastFoodAlertMs = 0;
  }
}


float readUltrasonicCm(uint8_t trigPin, uint8_t echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 25000UL);

  if (duration == 0)
  {
    return -1;
  }

  return duration / 58.0f;
}

bool isTurnOrDockEvent()
{
  return currentEvent == "TURN_LEFT" || currentEvent == "TURN_RIGHT" ||
         currentEvent == "UTURN" || currentEvent == "DOCK_TURNING" ||
         currentEvent == "DOCK_UTURN" || currentEvent == "REVERSING" ||
         currentEvent == "DOCK_REVERSING" || currentEvent == "SERVING";
}

void readObstacleSensors()
{
  if (millis() - lastUltrasonicReadMs < 180)
  {
    return;
  }

  lastUltrasonicReadMs = millis();

  prevLeftDistanceCm = leftDistanceCm;
  prevRightDistanceCm = rightDistanceCm;

  if (frontSensorEnabled) frontDistanceCm = readUltrasonicCm(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
  else frontDistanceCm = -1;

  delay(25);

  if (leftSensorEnabled) leftDistanceCm = readUltrasonicCm(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
  else leftDistanceCm = -1;

  delay(25);

  if (rightSensorEnabled) rightDistanceCm = readUltrasonicCm(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
  else rightDistanceCm = -1;
}

bool validCm(float cm)
{
  return cm > 1 && cm < 350;
}

String detectObstacleNow()
{
  if (!journeyActive || millis() < obstacleBypassUntilMs || servingWaitingForFood)
  {
    return "";
  }

  if (validCm(frontDistanceCm) && frontDistanceCm <= FRONT_OBSTACLE_CM)
  {
    obstacleMessage = "Front obstacle " + String(frontDistanceCm, 1) + " cm";
    return "front";
  }

  /*
    During turns, side ultrasonic values can naturally decrease because the robot
    body rotates near table/path boundaries. Ignore side approach detection during
    known turn/reverse/serving states. Front sensor still works.
  */
  if (!isTurnOrDockEvent())
  {
    if (validCm(leftDistanceCm) && validCm(prevLeftDistanceCm) &&
        leftDistanceCm < SIDE_CLOSE_CM &&
        (prevLeftDistanceCm - leftDistanceCm) >= SIDE_FAST_APPROACH_DELTA_CM)
    {
      obstacleMessage = "Left side getting closer " + String(leftDistanceCm, 1) + " cm";
      return "left";
    }

    if (validCm(rightDistanceCm) && validCm(prevRightDistanceCm) &&
        rightDistanceCm < SIDE_CLOSE_CM &&
        (prevRightDistanceCm - rightDistanceCm) >= SIDE_FAST_APPROACH_DELTA_CM)
    {
      obstacleMessage = "Right side getting closer " + String(rightDistanceCm, 1) + " cm";
      return "right";
    }
  }

  return "";
}

void playObstacleBeep()
{
  tone(BUZZER_PIN, 2600, 110);
}

void updateObstacleSafety()
{
  readObstacleSensors();

  String detected = detectObstacleNow();

  if (detected.length() > 0)
  {
    obstacleClearStartMs = 0;

    if (!obstaclePaused)
    {
      if (sendToSTM32WaitAck("PAUSE", "OK:PAUSE", 2, 500))
      {
        obstaclePaused = true;
        obstacleHelpNeeded = false;
        obstacleSensor = detected;
        obstacleStartMs = millis();
        currentEventSeq++;
        currentEvent = "OBSTACLE_STOP";
        addEventHistory(currentEventSeq, currentEvent);
        robotState = "Obstacle stop: " + obstacleMessage;
      }
    }
    else
    {
      obstacleSensor = detected;
    }

    if (millis() - lastObstacleAlertMs >= OBSTACLE_ALERT_INTERVAL_MS)
    {
      lastObstacleAlertMs = millis();
      playObstacleBeep();
    }

    if (obstaclePaused && !obstacleHelpNeeded && millis() - obstacleStartMs >= OBSTACLE_HELP_MS)
    {
      obstacleHelpNeeded = true;
      currentEventSeq++;
      currentEvent = "HELP_NEEDED";
      addEventHistory(currentEventSeq, currentEvent);
      robotState = "Help needed: obstacle not cleared";
    }

    return;
  }

  if (obstaclePaused)
  {
    if (obstacleClearStartMs == 0)
    {
      obstacleClearStartMs = millis();
    }

    if ((millis() - obstacleStartMs >= OBSTACLE_MIN_STOP_MS) &&
        (millis() - obstacleClearStartMs >= OBSTACLE_CLEAR_CONFIRM_MS))
    {
      if (sendToSTM32WaitAck("RESUME", "OK:RESUME", 2, 500))
      {
        obstaclePaused = false;
        obstacleHelpNeeded = false;
        obstacleSensor = "none";
        obstacleMessage = "Obstacle cleared";
        obstacleStartMs = 0;
        obstacleClearStartMs = 0;
        currentEventSeq++;
        currentEvent = "OBSTACLE_CLEARED";
        addEventHistory(currentEventSeq, currentEvent);
        robotState = "Obstacle cleared - resumed";
      }
    }
  }
}

uint32_t routeExpectedDurationMs(int table)
{
  if (table == 1) return 19000;
  if (table == 2) return 26500;
  if (table == 3) return 34500;
  return 0;
}

String tableName(int table)
{
  switch (table)
  {
    case 1: return "Table 1";
    case 2: return "Table 2";
    case 3: return "Table 3";
    default: return "Unknown";
  }
}

String jsonEscape(String s)
{
  s.replace("\\", "\\\\");
  s.replace("\"", "\\\"");
  s.replace("\n", "\\n");
  s.replace("\r", "");
  return s;
}

void addCors()
{
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

void sendJson(int code, String body)
{
  addCors();
  server.send(code, "application/json", body);
}

void handleOptions()
{
  addCors();
  server.send(204);
}


String ledPatternName(uint8_t pattern)
{
  switch (pattern)
  {
    case LED_PATTERN_FIXED: return "fixed";
    case LED_PATTERN_RUNNING: return "running";
    case LED_PATTERN_RANDOM: return "random";
    case LED_PATTERN_RAINBOW: return "rainbow";
    case LED_PATTERN_BREATHING: return "breathing";
    default: return "fixed";
  }
}

uint8_t ledPatternFromString(String value, uint8_t fallback)
{
  value.toLowerCase();
  if (value == "fixed") return LED_PATTERN_FIXED;
  if (value == "running") return LED_PATTERN_RUNNING;
  if (value == "random") return LED_PATTERN_RANDOM;
  if (value == "rainbow") return LED_PATTERN_RAINBOW;
  if (value == "breathing") return LED_PATTERN_BREATHING;
  return fallback;
}

uint32_t colorScale(Adafruit_NeoPixel &strip, uint8_t r, uint8_t g, uint8_t b, uint8_t level)
{
  uint16_t rr = ((uint16_t)r * level) / 255;
  uint16_t gg = ((uint16_t)g * level) / 255;
  uint16_t bb = ((uint16_t)b * level) / 255;
  return strip.Color((uint8_t)rr, (uint8_t)gg, (uint8_t)bb);
}

uint32_t wheelColor(Adafruit_NeoPixel &strip, uint8_t pos, uint8_t level)
{
  pos = 255 - pos;
  if (pos < 85)
  {
    return colorScale(strip, 255 - pos * 3, 0, pos * 3, level);
  }
  if (pos < 170)
  {
    pos -= 85;
    return colorScale(strip, 0, pos * 3, 255 - pos * 3, level);
  }
  pos -= 170;
  return colorScale(strip, pos * 3, 255 - pos * 3, 0, level);
}

uint8_t breathingLevel(uint8_t maxLevel, uint8_t speed)
{
  uint32_t period = map(speed, 1, 100, 4200, 800);
  float phase = ((float)(millis() % period) / (float)period) * 6.2831853f;
  float wave = (sinf(phase) + 1.0f) * 0.5f;
  uint8_t minLevel = maxLevel / 12;
  return minLevel + (uint8_t)((maxLevel - minLevel) * wave);
}

void renderLedSegment(Adafruit_NeoPixel &strip, uint16_t start, uint16_t count, const LedRuntimeConfig &cfg, bool reverseDirection)
{
  if (count == 0)
  {
    return;
  }

  if (!cfg.enabled || cfg.brightness == 0)
  {
    for (uint16_t i = 0; i < count; i++)
    {
      strip.setPixelColor(start + i, 0);
    }
    return;
  }

  uint8_t baseLevel = cfg.brightness;

  if (cfg.pattern == LED_PATTERN_FIXED)
  {
    uint32_t c = colorScale(strip, cfg.r, cfg.g, cfg.b, baseLevel);
    for (uint16_t i = 0; i < count; i++)
    {
      strip.setPixelColor(start + i, c);
    }
    return;
  }

  if (cfg.pattern == LED_PATTERN_BREATHING)
  {
    uint8_t level = breathingLevel(baseLevel, cfg.speed);
    uint32_t c = colorScale(strip, cfg.r, cfg.g, cfg.b, level);
    for (uint16_t i = 0; i < count; i++)
    {
      strip.setPixelColor(start + i, c);
    }
    return;
  }

  if (cfg.pattern == LED_PATTERN_RAINBOW)
  {
    uint8_t speedStep = map(cfg.speed, 1, 100, 1, 8);
    for (uint16_t i = 0; i < count; i++)
    {
      uint16_t pos = reverseDirection ? (count - 1 - i) : i;
      uint8_t hue = (uint8_t)((pos * 256UL / count + ledFrame * speedStep) & 0xFF);
      strip.setPixelColor(start + i, wheelColor(strip, hue, baseLevel));
    }
    return;
  }

  if (cfg.pattern == LED_PATTERN_RANDOM)
  {
    uint32_t background = colorScale(strip, cfg.r, cfg.g, cfg.b, baseLevel / 20);
    for (uint16_t i = 0; i < count; i++)
    {
      if (random(0, 100) < 12)
      {
        strip.setPixelColor(start + i, colorScale(strip, cfg.r, cfg.g, cfg.b, baseLevel));
      }
      else
      {
        strip.setPixelColor(start + i, background);
      }
    }
    return;
  }

  /* Running / comet pattern */
  uint8_t speedStep = map(cfg.speed, 1, 100, 1, 5);
  uint16_t head = (ledFrame * speedStep) % count;
  uint8_t tail = 10;

  for (uint16_t i = 0; i < count; i++)
  {
    uint16_t pos = reverseDirection ? (count - 1 - i) : i;
    uint16_t dist = (pos + count - head) % count;
    uint8_t level = baseLevel / 22;

    if (dist == 0)
    {
      level = baseLevel;
    }
    else if (dist < tail)
    {
      level = (uint8_t)(((uint16_t)baseLevel * (tail - dist)) / tail);
      if (level < baseLevel / 18) level = baseLevel / 18;
    }

    strip.setPixelColor(start + i, colorScale(strip, cfg.r, cfg.g, cfg.b, level));
  }
}

void showLedsNow()
{
  renderLedSegment(ledStrip36, LED36_RIGHT_START, LED_SIDE_COUNT, ledRight, false);
  renderLedSegment(ledStrip36, LED36_LEFT_START, LED_SIDE_COUNT, ledLeft, true);
  ledStrip36.show();

#if LED37_COUNT > 0
  renderLedSegment(ledStrip37, 0, LED37_COUNT, ledExtra, false);
  ledStrip37.show();
#endif
}

void updateLedEffects()
{
  uint32_t now = millis();
  if (now - lastLedUpdateMs < LED_UPDATE_INTERVAL_MS)
  {
    return;
  }

  lastLedUpdateMs = now;
  ledFrame++;
  showLedsNow();
}

void ledBegin()
{
  ledStrip36.begin();
  ledStrip36.setBrightness(255);
  ledStrip36.clear();
  ledStrip36.show();

#if LED37_COUNT > 0
  ledStrip37.begin();
  ledStrip37.setBrightness(255);
  ledStrip37.clear();
  ledStrip37.show();
#endif
}

void saveOneLedConfig(const char *prefix, const LedRuntimeConfig &cfg)
{
  prefs.putBool((String(prefix) + "en").c_str(), cfg.enabled);
  prefs.putUChar((String(prefix) + "r").c_str(), cfg.r);
  prefs.putUChar((String(prefix) + "g").c_str(), cfg.g);
  prefs.putUChar((String(prefix) + "b").c_str(), cfg.b);
  prefs.putUChar((String(prefix) + "br").c_str(), cfg.brightness);
  prefs.putUChar((String(prefix) + "pat").c_str(), cfg.pattern);
  prefs.putUChar((String(prefix) + "spd").c_str(), cfg.speed);
}

void loadOneLedConfig(const char *prefix, LedRuntimeConfig &cfg)
{
  cfg.enabled = prefs.getBool((String(prefix) + "en").c_str(), cfg.enabled);
  cfg.r = prefs.getUChar((String(prefix) + "r").c_str(), cfg.r);
  cfg.g = prefs.getUChar((String(prefix) + "g").c_str(), cfg.g);
  cfg.b = prefs.getUChar((String(prefix) + "b").c_str(), cfg.b);
  cfg.brightness = prefs.getUChar((String(prefix) + "br").c_str(), cfg.brightness);
  cfg.pattern = prefs.getUChar((String(prefix) + "pat").c_str(), cfg.pattern);
  cfg.speed = prefs.getUChar((String(prefix) + "spd").c_str(), cfg.speed);
}

void saveLedSettings()
{
  prefs.begin("leds", false);
  prefs.putBool("link", ledLinkSides);
  saveOneLedConfig("r", ledRight);
  saveOneLedConfig("l", ledLeft);
  saveOneLedConfig("x", ledExtra);
  prefs.end();
}

void loadLedSettings()
{
  prefs.begin("leds", true);
  ledLinkSides = prefs.getBool("link", ledLinkSides);
  loadOneLedConfig("r", ledRight);
  loadOneLedConfig("l", ledLeft);
  loadOneLedConfig("x", ledExtra);
  prefs.end();
}

String ledConfigJson(const LedRuntimeConfig &cfg)
{
  String json = "{";
  json += "\"enabled\":" + String(cfg.enabled ? "true" : "false") + ",";
  json += "\"r\":" + String(cfg.r) + ",";
  json += "\"g\":" + String(cfg.g) + ",";
  json += "\"b\":" + String(cfg.b) + ",";
  json += "\"brightness\":" + String(cfg.brightness) + ",";
  json += "\"pattern\":\"" + ledPatternName(cfg.pattern) + "\",";
  json += "\"speed\":" + String(cfg.speed);
  json += "}";
  return json;
}

String ledStatusJson()
{
  String json = "{";
  json += "\"pin36\":" + String(LED_PIN_36) + ",";
  json += "\"pin37\":" + String(LED_PIN_37) + ",";
  json += "\"sideCount\":" + String(LED_SIDE_COUNT) + ",";
  json += "\"pin36Total\":" + String(LED36_TOTAL_COUNT) + ",";
  json += "\"pin37Count\":" + String(LED37_COUNT) + ",";
  json += "\"linkSides\":" + String(ledLinkSides ? "true" : "false") + ",";
  json += "\"right\":" + ledConfigJson(ledRight) + ",";
  json += "\"left\":" + ledConfigJson(ledLeft) + ",";
  json += "\"extra\":" + ledConfigJson(ledExtra);
  json += "}";
  return json;
}

void updateLedConfigFromArgs(LedRuntimeConfig &cfg)
{
  if (server.hasArg("enabled")) cfg.enabled = server.arg("enabled").toInt() != 0;
  if (server.hasArg("r")) cfg.r = getIntArgClamped("r", cfg.r, 0, 255);
  if (server.hasArg("g")) cfg.g = getIntArgClamped("g", cfg.g, 0, 255);
  if (server.hasArg("b")) cfg.b = getIntArgClamped("b", cfg.b, 0, 255);
  if (server.hasArg("brightness")) cfg.brightness = getIntArgClamped("brightness", cfg.brightness, 0, 255);
  if (server.hasArg("speed")) cfg.speed = getIntArgClamped("speed", cfg.speed, 1, 100);
  if (server.hasArg("pattern")) cfg.pattern = ledPatternFromString(server.arg("pattern"), cfg.pattern);
}

void handleApiLedConfig()
{
  String zone = server.hasArg("zone") ? server.arg("zone") : "sides";
  zone.toLowerCase();

  if (server.hasArg("link"))
  {
    ledLinkSides = server.arg("link").toInt() != 0;
  }

  if (zone == "all")
  {
    updateLedConfigFromArgs(ledRight);
    ledLeft = ledRight;
    updateLedConfigFromArgs(ledExtra);
  }
  else if (zone == "sides" || zone == "both")
  {
    updateLedConfigFromArgs(ledRight);
    ledLeft = ledRight;
  }
  else if (zone == "right")
  {
    updateLedConfigFromArgs(ledRight);
    if (ledLinkSides) ledLeft = ledRight;
  }
  else if (zone == "left")
  {
    updateLedConfigFromArgs(ledLeft);
    if (ledLinkSides) ledRight = ledLeft;
  }
  else if (zone == "extra" || zone == "pin37")
  {
    updateLedConfigFromArgs(ledExtra);
  }
  else
  {
    sendJson(400, "{\"ok\":false,\"message\":\"Bad LED zone. Use all, sides, right, left, or extra.\"}");
    return;
  }

  saveLedSettings();
  showLedsNow();

  String json = jsonStatus();
  json.replace("\"ok\":true", "\"ok\":true,\"message\":\"LED settings updated\"");
  sendJson(200, json);
}


String readStm32Reply(uint32_t timeoutMs)
{
  String reply = "";
  uint32_t start = millis();

  while (millis() - start < timeoutMs)
  {
    while (STM32Serial.available())
    {
      char c = (char)STM32Serial.read();

      if (c == '\n')
      {
        reply.trim();
        return reply;
      }

      if (c != '\r')
      {
        reply += c;
      }
    }

    delay(2);
  }

  reply.trim();
  return reply;
}

void sendToSTM32(String line)
{
  line.trim();

  STM32Serial.print(line);
  STM32Serial.print("\n");

  lastRoute = line;
  lastCommandTime = millis();
  robotState = "Sent: " + line;

  String reply = readStm32Reply(450);
  if (reply.length() > 0)
  {
    handleStm32Line(reply);
  }
  else
  {
    lastReply = "No immediate STM32 reply";
  }
}


bool sendToSTM32WaitAck(String line, const String &expectedAck, uint8_t retries = 2, uint32_t timeoutMs = 900)
{
  line.trim();

  for (uint8_t attempt = 1; attempt <= retries; attempt++)
  {
    STM32Serial.print(line);
    STM32Serial.print("\n");

    lastRoute = line;
    lastCommandTime = millis();
    robotState = "Sent: " + line + " attempt " + String(attempt);

    uint32_t start = millis();

    while (millis() - start < timeoutMs)
    {
      while (STM32Serial.available())
      {
        String reply = STM32Serial.readStringUntil('\n');
        reply.trim();

        if (reply.length() == 0)
        {
          continue;
        }

        if (reply.startsWith("EVT:"))
        {
          handleStm32Line(reply);
          continue;
        }

        lastReply = reply;
        handleStm32Line(reply);

        if (reply.startsWith(expectedAck))
        {
          robotState = "ACK: " + expectedAck;
          return true;
        }

        if (reply.startsWith("ERR:"))
        {
          return false;
        }
      }

      delay(2);
    }
  }

  lastReply = "ACK timeout waiting for " + expectedAck;
  robotState = lastReply;
  return false;
}



void startAccessPoint()
{
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAPConfig(apLocalIp, apGateway, apSubnet);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  Serial.println("AP-only mode active.");
  Serial.print("SSID: ");
  Serial.println(AP_SSID);
  Serial.print("Password: ");
  Serial.println(AP_PASSWORD);
  Serial.print("Robot API: http://");
  Serial.println(WiFi.softAPIP());
}

bool connectToSavedWiFi(uint32_t timeoutMs = 12000)
{
  (void)timeoutMs;
  /*
    AP-only mode: no router connection.
    Connect client device to RestaurantRobot_AP and use http://192.168.4.1
  */
  wifiSsid = AP_SSID;
  wifiPass = AP_PASSWORD;
  return true;
}

void saveWiFiCredentials(String ssid, String pass)
{
  prefs.begin("wifi", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.end();

  wifiSsid = ssid;
  wifiPass = pass;
}



void clearEventHistory()
{
  eventHistoryCount = 0;
}

void addEventHistory(uint32_t seq, String eventName)
{
  if (eventHistoryCount >= EVENT_HISTORY_MAX)
  {
    for (uint8_t i = 1; i < EVENT_HISTORY_MAX; i++)
    {
      eventHistory[i - 1] = eventHistory[i];
    }
    eventHistoryCount = EVENT_HISTORY_MAX - 1;
  }

  eventHistory[eventHistoryCount].seq = seq;
  eventHistory[eventHistoryCount].event = eventName;
  eventHistory[eventHistoryCount].ms = journeyStartMs > 0 ? (millis() - journeyStartMs) : 0;
  eventHistoryCount++;
}

String eventHistoryJson()
{
  String out = "[";

  for (uint8_t i = 0; i < eventHistoryCount; i++)
  {
    if (i > 0) out += ",";
    out += "{";
    out += "\"seq\":" + String(eventHistory[i].seq) + ",";
    out += "\"event\":\"" + jsonEscape(eventHistory[i].event) + "\",";
    out += "\"ms\":" + String(eventHistory[i].ms);
    out += "}";
  }

  out += "]";
  return out;
}

void maintainWiFi()
{
  /*
    AP-only mode. Keep AP running; no station reconnect needed.
  */
  if (WiFi.getMode() != WIFI_AP)
  {
    startAccessPoint();
  }
}

void handleStm32Line(String line)
{
  line.trim();

  if (line.length() == 0)
  {
    return;
  }

  Serial.print("STM32: ");
  Serial.println(line);

  if (line.startsWith("EVT:"))
  {
    int firstColon = line.indexOf(':');
    int secondColon = line.indexOf(':', firstColon + 1);

    if (secondColon > 0)
    {
      currentEventSeq = line.substring(firstColon + 1, secondColon).toInt();
      currentEvent = line.substring(secondColon + 1);
    }
    else
    {
      currentEventSeq++;
      currentEvent = line.substring(4);
    }

    lastEventMs = millis();
    robotState = currentEvent;
    addEventHistory(currentEventSeq, currentEvent);

    if (currentEvent == "SERVING")
    {
      servingWaitingForFood = true;
      foodTakenSentThisServe = false;
      foodClearCounter = 0;
      foodEmptySinceMs = 0;
      lastFoodAlertMs = 0;
    }

    if (currentEvent == "REVERSING" || currentEvent == "RETURNING" ||
        currentEvent == "DOCKED" || currentEvent == "STOPPED" ||
        currentEvent == "ERROR")
    {
      servingWaitingForFood = false;
      foodEmptySinceMs = 0;
    }

    if (currentEvent == "DOCKED" || currentEvent == "STOPPED" || currentEvent == "ERROR")
    {
      journeyActive = false;
    }

    return;
  }

  lastReply = line;

  if (line.startsWith("OK:ROUTE"))
  {
    robotState = "ROUTE_ACCEPTED";
  }
  else if (line.startsWith("OK:STOP"))
  {
    robotState = "STOPPED";
    currentEvent = "STOPPED";
    currentEventSeq++;
    addEventHistory(currentEventSeq, currentEvent);
    journeyActive = false;
  }
  else if (line.startsWith("OK:CHARGER"))
  {
    /* Keep the live navigation state unchanged. Charger state is reported
       separately through /api/status as chargerConnected. */
  }
  else
  {
    robotState = line;
  }
}

String setupPage()
{
  String page = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Robot ESP Setup</title>
  <style>
    body{
      margin:0;
      min-height:100vh;
      display:grid;
      place-items:center;
      background:radial-gradient(circle at 20% 10%, rgba(40,246,255,.22), transparent 32%),
                 radial-gradient(circle at 80% 20%, rgba(255,79,227,.18), transparent 30%),
                 #050711;
      color:#eef9ff;
      font-family:Arial, sans-serif;
    }
    .card{
      width:min(92vw,460px);
      padding:28px;
      border-radius:26px;
      border:1px solid rgba(40,246,255,.25);
      background:rgba(9,18,34,.82);
      box-shadow:0 30px 90px rgba(0,0,0,.45);
    }
    h1{margin:0 0 8px;font-size:32px}
    p{color:#8eabc5;line-height:1.5}
    label{display:block;margin:14px 0 7px;color:#b9d7ef}
    input{
      width:100%;
      box-sizing:border-box;
      border:1px solid rgba(136,164,190,.25);
      border-radius:16px;
      outline:none;
      padding:14px 15px;
      color:#eef9ff;
      background:rgba(255,255,255,.06);
      font-size:15px;
    }
    button{
      width:100%;
      margin-top:18px;
      border:0;
      border-radius:18px;
      padding:15px 18px;
      font-weight:800;
      color:#041015;
      cursor:pointer;
      background:linear-gradient(90deg,#28f6ff,#43ffac);
    }
    .mini{
      margin-top:18px;
      padding:12px;
      border-radius:16px;
      background:rgba(255,255,255,.05);
      color:#8eabc5;
      font-size:12px;
      word-break:break-word;
    }
  </style>
</head>
<body>
  <main class="card">
    <h1>ESP32 Robot API</h1>
    <p>This ESP hosts only the setup page and robot API. The main UI is hosted from your laptop.</p>
    <form method="POST" action="/savewifi">
      <label>Restaurant WiFi SSID</label>
      <input name="ssid" placeholder="WiFi name" required>
      <label>Password</label>
      <input name="pass" type="password" placeholder="WiFi password">
      <button type="submit">Save & Connect</button>
    </form>
    <div class="mini">
      Setup AP: DeliveryRobot_Setup / 12345678<br>
      API after WiFi: http://192.168.4.1/api/status or ESP IP /api/status
    </div>
  </main>
</body>
</html>
)rawliteral";

  return page;
}

String jsonStatus()
{
  bool connected = true;
  String ip = WiFi.softAPIP().toString();

  String json = "{";
  json += "\"ok\":true,";
  json += "\"wifiConnected\":true,";
  json += "\"apMode\":true,";
  json += "\"ip\":\"" + jsonEscape(ip) + "\",";
  json += "\"apIp\":\"" + jsonEscape(WiFi.softAPIP().toString()) + "\",";
  json += "\"robotState\":\"" + jsonEscape(robotState) + "\",";
  json += "\"selectedTable\":\"" + jsonEscape(selectedTable) + "\",";
  json += "\"lastRoute\":\"" + jsonEscape(lastRoute) + "\",";
  json += "\"lastReply\":\"" + jsonEscape(lastReply) + "\",";
  json += "\"journeyActive\":" + String(journeyActive ? "true" : "false") + ",";
  json += "\"activeTable\":" + String(activeTable) + ",";
  json += "\"currentEvent\":\"" + jsonEscape(currentEvent) + "\",";
  json += "\"eventSeq\":" + String(currentEventSeq) + ",";
  json += "\"elapsedMs\":" + String(journeyActive ? (millis() - journeyStartMs) : 0) + ",";
  json += "\"lastEventAgeMs\":" + String(lastEventMs == 0 ? 0 : (millis() - lastEventMs)) + ",";
  json += "\"expectedDurationMs\":" + String(routeExpectedDurationMs(activeTable)) + ",";
  json += "\"events\":" + eventHistoryJson() + ",";
  json += "\"trayReady\":" + String(trayReady ? "true" : "false") + ",";
  json += "\"trayRawAverage\":" + String(trayRawAverage) + ",";
  json += "\"trayOffsetRaw\":" + String(trayOffsetRaw) + ",";
  json += "\"trayValueRaw\":" + String(trayValueRaw) + ",";
  json += "\"trayWeightG\":" + String(trayWeightG, 1) + ",";
  json += "\"traySignedWeightG\":" + String((calibration_factor == 0 ? 0 : (trayValueRaw / calibration_factor)), 1) + ",";
  json += "\"trayRawDirection\":\"" + String(trayValueRaw >= 0 ? "positive" : "negative") + "\",";
  json += "\"trayHasFood\":" + String(trayHasFood ? "true" : "false") + ",";
  json += "\"foodThresholdG\":" + String(FOOD_THRESHOLD_G, 0) + ",";
  json += "\"foodEmptyConfirmMs\":" + String(FOOD_EMPTY_CONFIRM_MS) + ",";
  json += "\"foodEmptyForMs\":" + String((foodEmptySinceMs == 0) ? 0 : (millis() - foodEmptySinceMs)) + ",";
  json += "\"hx711Scale\":" + String(calibration_factor, 6) + ",";
  json += "\"hx711OffsetSaved\":" + String(trayOffsetSaved ? "true" : "false") + ",";
  json += "\"hx711ScaleSaved\":" + String(trayCalibrationSaved ? "true" : "false") + ",";
  json += "\"servingWaitingForFood\":" + String(servingWaitingForFood ? "true" : "false") + ",";
  json += "\"foodTakenSent\":" + String(foodTakenSentThisServe ? "true" : "false") + ",";
  json += "\"foodAlertBuzzer\":" + String((servingWaitingForFood && trayHasFood && !foodTakenSentThisServe) ? "true" : "false") + ",";
  json += "\"frontCm\":" + String(frontDistanceCm, 1) + ",";
  json += "\"leftCm\":" + String(leftDistanceCm, 1) + ",";
  json += "\"rightCm\":" + String(rightDistanceCm, 1) + ",";
  json += "\"obstaclePaused\":" + String(obstaclePaused ? "true" : "false") + ",";
  json += "\"obstacleHelpNeeded\":" + String(obstacleHelpNeeded ? "true" : "false") + ",";
  json += "\"obstacleSensor\":\"" + jsonEscape(obstacleSensor) + "\",";
  json += "\"obstacleMessage\":\"" + jsonEscape(obstacleMessage) + "\",";
  json += "\"frontSensorEnabled\":" + String(frontSensorEnabled ? "true" : "false") + ",";
  json += "\"leftSensorEnabled\":" + String(leftSensorEnabled ? "true" : "false") + ",";
  json += "\"rightSensorEnabled\":" + String(rightSensorEnabled ? "true" : "false") + ",";
  json += "\"obstacleBypassActive\":" + String(millis() < obstacleBypassUntilMs ? "true" : "false") + ",";
  json += "\"chargerConnected\":" + String(chargerConnected ? "true" : "false") + ",";
  json += "\"chargerSenseVoltage\":" + String(chargerSenseVoltage, 2) + ",";
  json += "\"chargerAdcRaw\":" + String(chargerAdcRaw) + ",";
  json += "\"chargerThresholdV\":" + String(CHARGER_CONNECTED_THRESHOLD_V, 2) + ",";
  json += "\"batteryVoltage\":" + String(batteryVoltage, 2) + ",";
  json += "\"batteryVoltageRaw\":" + String(batteryVoltageRaw, 2) + ",";
  json += "\"batteryVoltageCandidate\":" + String(batteryVoltageCandidate, 2) + ",";
  json += "\"batteryReadingUnstable\":" + String(batteryReadingUnstable ? "true" : "false") + ",";
  json += "\"batteryPercent\":" + String(batteryPercent) + ",";
  json += "\"batteryLow\":" + String(batteryLow ? "true" : "false") + ",";
  json += "\"batteryPinVoltage\":" + String(batteryPinVoltage, 2) + ",";
  json += "\"batteryAdcRaw\":" + String(batteryAdcRaw) + ",";
  json += "\"batteryDividerFactor\":" + String(BATTERY_DIVIDER_FACTOR, 3) + ",";
  json += "\"batteryFullV\":" + String(BATTERY_FULL_V, 2) + ",";
  json += "\"batteryLowV\":" + String(BATTERY_LOW_V, 2) + ",";
  json += "\"led\":" + ledStatusJson() + ",";
  json += "\"uptimeSec\":" + String(millis() / 1000);
  json += "}";

  return json;
}

void handleRoot()
{
  server.sendHeader("Location", "/setup");
  server.send(302, "text/plain", "");
}

void handleSetup()
{
  server.send(200, "text/html", setupPage());
}

void handleSaveWiFi()
{
  String ssid = server.arg("ssid");
  String pass = server.arg("pass");

  ssid.trim();

  if (ssid.length() == 0)
  {
    server.send(400, "text/plain", "SSID required");
    return;
  }

  saveWiFiCredentials(ssid, pass);

  server.send(
    200,
    "text/html",
    "<html><body style='background:#050711;color:#eef9ff;font-family:Arial;padding:30px'>"
    "<h2>WiFi saved</h2>"
    "<p>ESP32 is connecting. Check Serial Monitor for IP address.</p>"
    "<p>Then run React app on laptop and set ESP address to http://192.168.4.1 or ESP IP.</p>"
    "<p><a style='color:#28f6ff' href='/api/status'>Check API status</a></p>"
    "</body></html>"
  );

  delay(500);
  WiFi.disconnect();
  delay(300);
  connectToSavedWiFi(12000);
}

void handleApiStatus()
{
  sendJson(200, jsonStatus());
}

void handleApiServe()
{
  if (!server.hasArg("table"))
  {
    sendJson(400, "{\"ok\":false,\"message\":\"Missing table number\"}");
    return;
  }

  int table = server.arg("table").toInt();
  String route = routeForTable(table);

  if (route.length() == 0)
  {
    sendJson(400, "{\"ok\":false,\"message\":\"Invalid table\"}");
    return;
  }

  selectedTable = tableName(table);
  activeTable = table;
  journeyStartMs = millis();
  lastEventMs = millis();
  clearEventHistory();
  servingWaitingForFood = false;
  foodTakenSentThisServe = false;
  foodClearCounter = 0;
  foodEmptySinceMs = 0;
  currentEvent = "SENDING";
  currentEventSeq = 0;
  robotState = "Sending " + selectedTable;

  bool ack = sendToSTM32WaitAck(route, "OK:ROUTE", 3, 900);

  if (!ack)
  {
    journeyActive = false;
    activeTable = 0;
    currentEvent = "ROUTE_NOT_ACK";
    currentEventSeq++;
    addEventHistory(currentEventSeq, currentEvent);

    String json = jsonStatus();
    json.replace("\"ok\":true", "\"ok\":false,\"message\":\"Route not acknowledged by STM32. Try again.\"");
    sendJson(504, json);
    return;
  }

  journeyActive = true;

  String json = jsonStatus();
  json.replace("\"ok\":true", "\"ok\":true,\"message\":\"Route acknowledged by STM32\",\"ack\":true");
  sendJson(200, json);
}

void handleApiWeightRaw()
{
  int times = getIntArgClamped("times", 10, 1, 50);

  if (!waitHx711Ready(1800))
  {
    trayReady = false;
    sendJson(503, "{\"ok\":false,\"message\":\"HX711 not ready\"}");
    return;
  }

  trayReady = true;
  long raw = trayScale.read_average(times);
  trayRawAverage = raw;
  trayValueRaw = raw - trayOffsetRaw;
  trayWeightG = weightFromRawDiff(trayValueRaw);
  trayHasFood = trayWeightG >= FOOD_THRESHOLD_G;

  sendJson(200, weightJson(raw, times));
}

void handleApiWeightTare()
{
  int times = getIntArgClamped("times", 20, 1, 80);

  if (!waitHx711Ready(1800))
  {
    trayReady = false;
    sendJson(503, "{\"ok\":false,\"message\":\"HX711 not ready\"}");
    return;
  }

  trayReady = true;
  trayOffsetRaw = trayScale.read_average(times);
  applyHx711Calibration();
  saveHx711Offset();

  trayRawAverage = trayOffsetRaw;
  trayValueRaw = 0;
  trayWeightG = 0;
  trayHasFood = false;
  foodClearCounter = 0;
  foodEmptySinceMs = 0;

  String json = weightJson(trayRawAverage, times);
  json.replace("\"ok\":true", "\"ok\":true,\"message\":\"Tare saved. Empty tray is now zero.\"");
  sendJson(200, json);
}

void handleApiWeightCalibrate()
{
  if (!server.hasArg("known"))
  {
    sendJson(400, "{\"ok\":false,\"message\":\"Missing known weight in grams. Example: /api/weight/calibrate?known=1050\"}");
    return;
  }

  float knownG = server.arg("known").toFloat();
  int times = getIntArgClamped("times", 20, 1, 80);

  if (knownG <= 0)
  {
    sendJson(400, "{\"ok\":false,\"message\":\"known must be greater than 0 grams\"}");
    return;
  }

  if (!trayOffsetSaved)
  {
    sendJson(400, "{\"ok\":false,\"message\":\"Tare first with empty tray using /api/weight/tare\"}");
    return;
  }

  if (!waitHx711Ready(1800))
  {
    trayReady = false;
    sendJson(503, "{\"ok\":false,\"message\":\"HX711 not ready\"}");
    return;
  }

  trayReady = true;
  long raw = trayScale.read_average(times);
  long valueRaw = raw - trayOffsetRaw;

  if (labs(valueRaw) < 100)
  {
    sendJson(400, "{\"ok\":false,\"message\":\"Raw difference too small. Add known weight before calibrating.\"}");
    return;
  }

  calibration_factor = fabs((float)valueRaw) / knownG;
  applyHx711Calibration();
  saveHx711Scale();

  trayRawAverage = raw;
  trayValueRaw = valueRaw;
  trayWeightG = weightFromRawDiff(trayValueRaw);
  trayHasFood = trayWeightG >= FOOD_THRESHOLD_G;

  String json = weightJson(raw, times);
  json.replace("\"ok\":true", "\"ok\":true,\"message\":\"Calibration saved\""); 
  sendJson(200, json);
}

void handleApiWeightSetScale()
{
  String scaleArg = "";

  if (server.hasArg("scale"))
  {
    scaleArg = server.arg("scale");
  }
  else if (server.hasArg("factor"))
  {
    scaleArg = server.arg("factor");
  }
  else
  {
    sendJson(400, "{\"ok\":false,\"message\":\"Missing scale/factor. Example: /api/weight/setscale?scale=25.61\"}");
    return;
  }

  float newScale = scaleArg.toFloat();

  if (fabs(newScale) < 0.000001f)
  {
    sendJson(400, "{\"ok\":false,\"message\":\"Scale/factor cannot be zero\"}");
    return;
  }

  calibration_factor = fabs(newScale);
  applyHx711Calibration();
  saveHx711Scale();

  if (trayScale.is_ready())
  {
    trayReady = true;
    trayRawAverage = trayScale.read_average(10);
    trayValueRaw = trayRawAverage - trayOffsetRaw;
    trayWeightG = weightFromRawDiff(trayValueRaw);
    trayHasFood = trayWeightG >= FOOD_THRESHOLD_G;
  }

  String json = weightJson(trayRawAverage, 10);
  json.replace("\"ok\":true", "\"ok\":true,\"message\":\"Scale/factor saved\"");
  sendJson(200, json);
}

void handleApiWeightConfig()
{
  sendJson(200, jsonStatus());
}

/* Backward-compatible short tare endpoint */
void handleApiTare()
{
  handleApiWeightTare();
}



void handleApiObstacleBypass()
{
  int seconds = 60;

  if (server.hasArg("seconds"))
  {
    seconds = server.arg("seconds").toInt();
  }

  if (seconds < 5) seconds = 5;
  if (seconds > 600) seconds = 600;

  obstacleBypassUntilMs = millis() + ((uint32_t)seconds * 1000UL);

  if (obstaclePaused)
  {
    sendToSTM32WaitAck("RESUME", "OK:RESUME", 2, 500);
    obstaclePaused = false;
  }

  obstacleHelpNeeded = false;
  obstacleMessage = "Obstacle bypass active for " + String(seconds) + " s";
  robotState = obstacleMessage;

  String json = jsonStatus();
  json.replace("\"ok\":true", "\"ok\":true,\"message\":\"Obstacle bypass enabled for " + String(seconds) + " seconds\"");
  sendJson(200, json);
}

void handleApiObstacleEnable()
{
  if (server.hasArg("front")) frontSensorEnabled = server.arg("front").toInt() != 0;
  if (server.hasArg("left")) leftSensorEnabled = server.arg("left").toInt() != 0;
  if (server.hasArg("right")) rightSensorEnabled = server.arg("right").toInt() != 0;

  obstacleMessage = "Obstacle sensor settings updated";
  sendJson(200, jsonStatus());
}

void handleApiHallRead()
{
  STM32Serial.print("HALL_READ\n");

  uint32_t start = millis();
  String reply = "";

  while (millis() - start < 900)
  {
    while (STM32Serial.available())
    {
      reply = STM32Serial.readStringUntil('\n');
      reply.trim();

      if (reply.startsWith("HALL:"))
      {
        lastReply = reply;
        String json = "{";
        json += "\"ok\":true,";
        json += "\"hall\":\"" + jsonEscape(reply) + "\"";
        json += "}";
        sendJson(200, json);
        return;
      }

      if (reply.startsWith("EVT:"))
      {
        handleStm32Line(reply);
      }
      else if (reply.length() > 0)
      {
        lastReply = reply;
      }
    }
    delay(2);
  }

  sendJson(504, "{\"ok\":false,\"message\":\"No HALL response from STM32\"}");
}

void handleApiHallUpdate()
{
  STM32Serial.print("HALL_UPDATE\n");

  uint32_t start = millis();
  String hallReply = "";
  bool ok = false;

  while (millis() - start < 1200)
  {
    while (STM32Serial.available())
    {
      String reply = STM32Serial.readStringUntil('\n');
      reply.trim();

      if (reply.startsWith("HALL_UPDATED:"))
      {
        hallReply = reply;
        lastReply = reply;
      }
      else if (reply.startsWith("OK:HALL_UPDATE"))
      {
        ok = true;
      }
      else if (reply.startsWith("EVT:"))
      {
        handleStm32Line(reply);
      }
      else if (reply.length() > 0)
      {
        lastReply = reply;
      }

      if (ok)
      {
        String json = "{";
        json += "\"ok\":true,";
        json += "\"message\":\"Hall baseline updated in STM32 RAM\",";
        json += "\"hall\":\"" + jsonEscape(hallReply) + "\"";
        json += "}";
        sendJson(200, json);
        return;
      }
    }
    delay(2);
  }

  sendJson(504, "{\"ok\":false,\"message\":\"Hall update not acknowledged\"}");
}

void handleApiReturn()
{
  robotState = "Return requested";
  bool ack = sendToSTM32WaitAck("RETURN_NOW", "OK:", 3, 900);

  if (!ack)
  {
    String json = jsonStatus();
    json.replace("\"ok\":true", "\"ok\":false,\"message\":\"Return command not acknowledged by STM32\"");
    sendJson(504, json);
    return;
  }

  journeyActive = true;
  if (journeyStartMs == 0)
  {
    journeyStartMs = millis();
  }

  String json = jsonStatus();
  json.replace("\"ok\":true", "\"ok\":true,\"message\":\"Return command acknowledged\",\"ack\":true");
  sendJson(200, json);
}

void handleApiStop()
{
  /*
    This only works if the STM32 firmware supports a STOP UART command.
    If your current STM32 code only handles ROUTE, this will be ignored by STM32.
  */
  selectedTable = "None";
  journeyActive = false;
  activeTable = 0;
  currentEvent = "STOP_REQUESTED";
  currentEventSeq++;
  addEventHistory(currentEventSeq, currentEvent);
  robotState = "STOP command sent";
  sendToSTM32("STOP");

  String json = jsonStatus();
  json.replace("\"ok\":true", "\"ok\":true,\"message\":\"STOP sent. STM32 must support STOP command.\"");
  sendJson(200, json);
}

void handleApiRawRoute()
{
  /*
    Optional testing endpoint:
    /api/raw?route=ROUTE:2,0,3,0,1,4
  */
  if (!server.hasArg("route"))
  {
    sendJson(400, "{\"ok\":false,\"message\":\"Missing route\"}");
    return;
  }

  String route = server.arg("route");
  route.trim();

  if (!route.startsWith("ROUTE:"))
  {
    sendJson(400, "{\"ok\":false,\"message\":\"Route must start with ROUTE:\"}");
    return;
  }

  selectedTable = "Custom";
  sendToSTM32(route);

  String json = jsonStatus();
  json.replace("\"ok\":true", "\"ok\":true,\"message\":\"Raw route sent\"");
  sendJson(200, json);
}

void handleClearWiFi()
{
  prefs.begin("wifi", false);
  prefs.clear();
  prefs.end();

  wifiSsid = "";
  wifiPass = "";

  sendJson(200, "{\"ok\":true,\"message\":\"WiFi credentials cleared\"}");
}

void handleNotFound()
{
  if (server.method() == HTTP_OPTIONS)
  {
    handleOptions();
    return;
  }

  addCors();
  server.send(404, "text/plain", "Not found");
}

void setup()
{
  Serial.begin(115200);
  delay(200);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(FRONT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_ECHO_PIN, INPUT);
  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(LEFT_ECHO_PIN, INPUT);
  pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);

  pinMode(CHARGER_DETECT_PIN, INPUT);
  pinMode(BATTERY_ADC_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(CHARGER_DETECT_PIN, ADC_11db);
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);

  loadLedSettings();
  ledBegin();
  showLedsNow();

  loadHx711Calibration();
  trayScale.begin(HX711_DOUT, HX711_SCK);
  applyHx711Calibration();

  if (trayScale.is_ready())
  {
    trayReady = true;

    if (!trayOffsetSaved)
    {
      Serial.println("HX711 ready. No saved tare found, auto-taring empty tray...");
      trayOffsetRaw = trayScale.read_average(20);
      applyHx711Calibration();
      saveHx711Offset();
    }

    updateTrayWeight();
  }
  else
  {
    trayReady = false;
    Serial.println("HX711 not ready at boot.");
  }

  STM32Serial.begin(STM_BAUD, SERIAL_8N1, ESP_UART_RX_PIN, ESP_UART_TX_PIN);
  updatePowerSensors();

  Serial.println();
  Serial.println("ESP32 API-only robot controller starting...");

  startAccessPoint();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/setup", HTTP_GET, handleSetup);
  server.on("/savewifi", HTTP_POST, handleSaveWiFi);

  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/api/serve", HTTP_GET, handleApiServe);
  server.on("/api/stop", HTTP_GET, handleApiStop);
  server.on("/api/return", HTTP_GET, handleApiReturn);
  server.on("/api/obstacle/bypass", HTTP_GET, handleApiObstacleBypass);
  server.on("/api/obstacle/enable", HTTP_GET, handleApiObstacleEnable);
  server.on("/api/led/config", HTTP_GET, handleApiLedConfig);
  server.on("/api/hall/read", HTTP_GET, handleApiHallRead);
  server.on("/api/hall/update", HTTP_GET, handleApiHallUpdate);
  server.on("/api/tare", HTTP_GET, handleApiTare);
  server.on("/api/weight/raw", HTTP_GET, handleApiWeightRaw);
  server.on("/api/weight/tare", HTTP_GET, handleApiWeightTare);
  server.on("/api/weight/calibrate", HTTP_GET, handleApiWeightCalibrate);
  server.on("/api/weight/setscale", HTTP_GET, handleApiWeightSetScale);
  server.on("/api/weight/config", HTTP_GET, handleApiWeightConfig);
  server.on("/api/raw", HTTP_GET, handleApiRawRoute);
  server.on("/api/clearwifi", HTTP_POST, handleClearWiFi);

  server.on("/api/status", HTTP_OPTIONS, handleOptions);
  server.on("/api/serve", HTTP_OPTIONS, handleOptions);
  server.on("/api/stop", HTTP_OPTIONS, handleOptions);
  server.on("/api/return", HTTP_OPTIONS, handleOptions);
  server.on("/api/obstacle/bypass", HTTP_OPTIONS, handleOptions);
  server.on("/api/obstacle/enable", HTTP_OPTIONS, handleOptions);
  server.on("/api/led/config", HTTP_OPTIONS, handleOptions);
  server.on("/api/hall/read", HTTP_OPTIONS, handleOptions);
  server.on("/api/hall/update", HTTP_OPTIONS, handleOptions);
  server.on("/api/tare", HTTP_OPTIONS, handleOptions);
  server.on("/api/weight/raw", HTTP_OPTIONS, handleOptions);
  server.on("/api/weight/tare", HTTP_OPTIONS, handleOptions);
  server.on("/api/weight/calibrate", HTTP_OPTIONS, handleOptions);
  server.on("/api/weight/setscale", HTTP_OPTIONS, handleOptions);
  server.on("/api/weight/config", HTTP_OPTIONS, handleOptions);
  server.on("/api/raw", HTTP_OPTIONS, handleOptions);
  server.on("/api/clearwifi", HTTP_OPTIONS, handleOptions);

  server.onNotFound(handleNotFound);

  server.begin();

  Serial.println("HTTP API server started.");
  Serial.print("Setup AP page: http://");
  Serial.println(WiFi.softAPIP());

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print("Robot API: http://");
    Serial.println(WiFi.localIP());
    Serial.println("Try: http://192.168.4.1/api/status");
  }
}

void loop()
{
  server.handleClient();
  maintainWiFi();
  updatePowerSensors();
  updateLedEffects();
  updateTrayWeight();
  checkFoodTakenRelease();
  updateFoodWaitingBuzzer();
  updateObstacleSafety();

  while (STM32Serial.available())
  {
    String line = STM32Serial.readStringUntil('\n');
    handleStm32Line(line);
  }
}
