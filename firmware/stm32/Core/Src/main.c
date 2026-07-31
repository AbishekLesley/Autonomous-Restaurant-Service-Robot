/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Magnetic line follower PID - JGB37 fast correction version
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart1;   /* USART1: STM32 <-> ESP32 */
UART_HandleTypeDef huart3;   /* USART3: debug serial */
TIM_HandleTypeDef htim4;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);

/* USER CODE BEGIN PFP */
uint16_t Read_ADC_Channel(uint32_t channel);
void Read_All_Hall_Sensors(void);
int Calculate_Line_Error(int *errorOut, int *strengthOut);
int Detect_T_Junction(void);
int Detect_Stop_Cross_Marker(void);
int Detect_Clear_90_Joint(void);

typedef enum
{
  ROUTE_CMD_STRAIGHT = 0,
  ROUTE_CMD_LEFT,
  ROUTE_CMD_RIGHT,
  ROUTE_CMD_STOP,       /* same behavior as serve stop in this version */
  ROUTE_CMD_SERVE_STOP = ROUTE_CMD_STOP,
  ROUTE_CMD_DOCK
} RouteCommand;

RouteCommand GetCurrentRouteCommand(void);
uint8_t IsJunctionRouteCommand(RouteCommand cmd);
uint8_t IsStrongRouteJunctionCandidate(int tDetected, int stopCrossDetected, int clear90Detected);
void ClearTurnHoldState(void);
void ResetPidFiltersFromError(int error);
void StartRouteJunctionStop(int8_t direction);
void StartRouteTurn(int8_t direction);
void FinishRouteTurn(int error);
void StartRouteStraightPass(void);
uint8_t BuildReturnRouteFromCurrentPosition(void);
uint8_t StartImmediateReturnToDock(void);
void StartServeStopSequence(void);
void StartDockStopTurnSequence(void);
void StartDockReverseToT(void);
void FinishDockParking(void);
uint8_t IsUTurnLineLocked(int lineDetected, int tDetected, int error);
uint8_t UpdateWideMarkerDetection(int tDetected);
uint8_t UpdateStopCrossMarkerDetection(int crossDetected);

void SetRobotSpeed(int leftSpeed, int rightSpeed);
void SetRobotSpeedSmooth(int leftSpeed, int rightSpeed, int step);
void DriveLeftMotor(int speed);
void DriveRightMotor(int speed);
void MotorStop(void);
void ResetFollowStateAfterStop(void);

void UpdateEncoders(void);
void Print_Debug(int error, int strength, int leftSpeed, int rightSpeed, int lineDetected);

/* ESP32 command UART helpers */
void CheckEspCommand(void);
void ProcessEspCommand(char *line);
uint8_t LoadRouteFromCsv(char *csv);
void ResetRouteRuntimeState(void);
void EspSendText(const char *text);
void EspSendEvent(const char *eventName);
void UpdateTimedRouteEvents(void);
uint8_t DetectDestinationTableFromRoute(void);
void ReportRouteJunctionEvent(void);
void ReportRouteTableEvent(void);
void SendHallSnapshot(const char *prefix);

int clamp_int(int value, int minVal, int maxVal);
int move_towards_int(int current, int target, int step);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* ============================================================
   HALL SENSOR SETUP

   ADC pins:
   H0 = PA0
   H1 = PA1
   H2 = PA2
   H3 = PA3
   H4 = PA4
   H5 = PA5
   H6 = PA6

   Physical order:
   Left to right:
   L0 = H6
   L1 = H5
   L2 = H2
   L3 = H1
   L4 = H0
   L5 = H4
   L6 = H3

   Error convention:
   Left side  = negative
   Center     = 0
   Right side = positive
   ============================================================ */

uint32_t hallChannels[7] = {
  ADC_CHANNEL_0,
  ADC_CHANNEL_1,
  ADC_CHANNEL_2,
  ADC_CHANNEL_3,
  ADC_CHANNEL_4,
  ADC_CHANNEL_5,
  ADC_CHANNEL_6
};

uint16_t hallRaw[7];

uint16_t hallBaseline[7] = {
  2050,  // H0
  2047,  // H1
  2037,  // H2
  2115,  // H3
  2080,  // H4
  2082,  // H5
  2075   // H6
};

uint8_t physicalOrder[7] = {
  6, 5, 2, 1, 0, 4, 3
};

int weights[7] = {
  -300, -200, -100, 0, 100, 200, 300
};

/* Hall detection */
#define MAG_THRESHOLD       35
#define MIN_LINE_STRENGTH   55

/* Temporarily disable broken H6 / L0 sensor only.
   No other line-following logic is changed. */
#define DISABLE_H6_SENSOR   0

/* JGB37 minimum drive */
#define MIN_DRIVE_SPEED     30

/* Wide marker / T-junction detection
   Because H6/L0 is disabled, active count 4 is used instead of 5.
   This is used only as a marker/junction event now.
   It is NOT used directly as an automatic stop. */
#define T_ACTIVE_COUNT      5
#define T_MIN_STRENGTH      260
#define T_CONFIRM_COUNT     3

uint8_t activePhysical[7] = {0};
int activeCount = 0;
int lastStrengthSum = 0;

/* kept name for debug compatibility; now means final route stop, not T-stop */
uint8_t stoppedAtTJunction = 0;
uint8_t tJunctionCounter = 0;

/* ============================================================
   ROUTE COMMAND SETUP

   Only this array is hardcoded for now.
   Later the ESP/frontend can fill routeCommands[] and routeCommandCount.

   Current evaluation map requested:
   Dock/start -> normal 30-degree segmented bends handled by PID
   -> decision junction / clear 90 joint -> STRAIGHT
   -> table T marker -> SERVE_STOP
   -> wait, reverse a little, 180 turn
   -> return path -> pass decision junction STRAIGHT
   -> dock T marker -> DOCK stop + 180 turn + final stop
   ============================================================ */

#define ROUTE_MAX_COMMANDS  10

RouteCommand routeCommands[ROUTE_MAX_COMMANDS] = {
  ROUTE_CMD_RIGHT,       /* first junction: take right path */
  ROUTE_CMD_STRAIGHT,    /* second junction: pass table 2 */
  ROUTE_CMD_SERVE_STOP,  /* table 3 T marker: stop, wait, reverse, 180 turn */
  ROUTE_CMD_STRAIGHT,    /* return: pass table 2 junction */
  ROUTE_CMD_LEFT,        /* return: turn back toward dock at first junction */
  ROUTE_CMD_DOCK         /* dock T marker */
};


uint8_t routeCommandCount = 0;  /* Filled by ESP32 ROUTE command */
uint8_t routeIndex = 0;
uint8_t routeFinished = 1;     /* Wait stopped until ESP32 sends a route */
uint8_t routePaused = 0;
uint8_t routeError = 0;
uint8_t routeJunctionDirectCounter = 0;
uint8_t executingRouteStartExit = 0;
uint32_t routeStartExitStartTime = 0;
uint32_t routeStartIgnoreUntil = 0;

/* Marker behavior */
#define STOP_MARKER_REQUIRED_CROSSES     1
#define MARKER_CLEAR_CONFIRM_COUNT       2
#define MARKER_COOLDOWN_MS               350
#define MARKER_IGNORE_AFTER_COMMAND_MS   1200
/*
   When a new route is received from ESP32, the robot may already be sitting
   on the dock/start T marker. If route detection is enabled immediately,
   that start marker can be mistaken as the first route junction and the robot
   will start turning at the dock.

   So after loading a route:
   - drive straight briefly to leave the dock marker
   - ignore route marker consumption for a little longer
*/
#define ROUTE_START_EXIT_MS              650  /* drive forward briefly to leave dock/start T */
#define ROUTE_START_MARKER_IGNORE_MS     1800
#define ROUTE_STRAIGHT_PASS_MS           800

/* STOP marker detection uses a stricter cross pattern than junction detection.
   This prevents a normal L/right-angle path turn from being counted as a stop marker. */
#define STOP_CROSS_ACTIVE_COUNT          5
#define STOP_CROSS_MIN_STRENGTH          340
#define STOP_CROSS_CONFIRM_COUNT         2
#define STOP_CROSS_CLEAR_CONFIRM_COUNT   2

/* Clear 90-degree / branch-joint detection.
   This is for bus-topology decision points seen from the head side, where
   the robot may see a side path before it can prove a full T.
   Gentle 30-degree segmented bends should not be shaped strongly enough to
   trigger this. */
#define CLEAR_90_MIN_STRENGTH            210
#define CLEAR_90_MIN_ACTIVE              3

/* Serve / dock sequence behavior */
/* Wait at table until ESP32 says FOOD_TAKEN, or timeout after 5 minutes. */
#define SERVE_WAIT_MS                    300000
#define SERVE_REVERSE_MS                 600
#define SERVE_REVERSE_SPEED              32
#define SERVE_UTURN_SPEED                30
#define SERVE_UTURN_MIN_MS               2200
#define SERVE_UTURN_MAX_MS               11000
#define SERVE_UTURN_DIRECTION            1    /* 1 or -1; change if it turns wrong way */

#define DOCK_STOP_WAIT_MS                350
/* Dock now uses the same stable method as the table serve turn:
   stop -> reverse a little -> 180 turn -> stop/settle -> reverse to T. */
#define DOCK_PRE_TURN_REVERSE_MS         SERVE_REVERSE_MS
#define DOCK_PRE_TURN_REVERSE_SPEED      SERVE_REVERSE_SPEED
#define DOCK_UTURN_SPEED                 SERVE_UTURN_SPEED
#define DOCK_UTURN_MIN_MS                SERVE_UTURN_MIN_MS
#define DOCK_UTURN_MAX_MS                SERVE_UTURN_MAX_MS
#define DOCK_UTURN_DIRECTION             SERVE_UTURN_DIRECTION
#define DOCK_AFTER_UTURN_SETTLE_MS       350  /* stop fully before reverse-to-T */
#define DOCK_REVERSE_SPEED               26   /* slow reverse after dock turn */
#define DOCK_REVERSE_MIN_MS              250  /* ignore very early T readings */
#define DOCK_REVERSE_MAX_MS              7000 /* safety timeout if T is not found */
#define DOCK_REVERSE_T_CONFIRM_COUNT     1

/* Dock parking reverse steering only.
   This is active only in DOCK_STATE_REVERSE_TO_T.
   It uses the front Hall array error while reversing so the robot can correct
   the small angle left after the dock 180-degree turn.

   If the robot corrects in the wrong direction during reverse parking, change
   DOCK_REVERSE_STEER_INVERT from 0 to 1. */
#define DOCK_REVERSE_STEER_ENABLE        1
#define DOCK_REVERSE_STEER_KP            0.07f
#define DOCK_REVERSE_STEER_MAX           10
#define DOCK_REVERSE_STEER_INVERT        0

#define UTURN_CENTER_CONFIRM_COUNT       3
#define UTURN_LOCK_ERROR                 115
#define UTURN_LOCK_MAX_ACTIVE            3

/* Commanded turn behavior at route junctions
   New method:
   - Stop immediately at a confirmed T/wide marker.
   - After a short pause, use the same gentle base-code turn recovery style
     that already works on normal path turns: one wheel moves forward, the
     other wheel is stopped. No reverse pivot and no fast circular turn. */
#define ROUTE_JUNCTION_STOP_MS           320
/* After stopping at the T, keep the commanded turn until the chosen
   branch is center-locked. Then release back to normal PID/turn-hold. */
#define ROUTE_TURN_MIN_MS                1400
#define ROUTE_TURN_MAX_MS                10500
#define ROUTE_TURN_CENTER_CONFIRM_COUNT  3
#define ROUTE_TURN_RAMP_STEP             RECOVERY_RAMP_STEP
#define ROUTE_TURN_SPEED                 RECOVERY_SPEED
#define ROUTE_AFTER_TURN_IGNORE_MS       0
#define ROUTE_BRANCH_LOCK_ERROR          115
#define ROUTE_BRANCH_LOCK_MAX_ACTIVE     3

/* Direct route-junction confirmation.
   The uploaded log showed W:1/SC:1 but RIdx stayed 0, so relying only
   on one wideMarkerEvent was not robust enough. This counter consumes
   LEFT/RIGHT/STRAIGHT as soon as a strong junction/cross is seen. */
#define ROUTE_JUNCTION_DIRECT_CONFIRM_COUNT 1

uint8_t wideMarkerActive = 0;
uint8_t wideMarkerClearCounter = 0;
uint32_t markerIgnoreUntil = 0;
uint32_t lastMarkerEventTime = 0;

uint8_t stopMarkerCount = 0;

uint8_t stopCrossActive = 0;
uint8_t stopCrossConfirmCounter = 0;
uint8_t stopCrossClearCounter = 0;
uint8_t lastStopCrossDetected = 0;

uint8_t executingRouteJunctionStop = 0;
int8_t routeJunctionStopDirection = 0;  // -1 = left, +1 = right
uint32_t routeJunctionStopStartTime = 0;

uint8_t executingRouteTurn = 0;
int8_t routeTurnDirection = 0;    // -1 = left, +1 = right
uint32_t routeTurnStartTime = 0;
uint8_t routeTurnCenterConfirmCount = 0;

uint8_t executingRouteStraight = 0;
uint32_t routeStraightStartTime = 0;

typedef enum
{
  SERVE_STATE_NONE = 0,
  SERVE_STATE_WAIT,
  SERVE_STATE_REVERSE,
  SERVE_STATE_UTURN
} ServeState;

typedef enum
{
  DOCK_STATE_NONE = 0,
  DOCK_STATE_WAIT,
  DOCK_STATE_PRE_TURN_REVERSE,
  DOCK_STATE_UTURN,
  DOCK_STATE_REVERSE_SETTLE,
  DOCK_STATE_REVERSE_TO_T
} DockState;

uint8_t serveSequenceActive = 0;
ServeState serveState = SERVE_STATE_NONE;
uint32_t serveStateStartTime = 0;
uint8_t serveUturnCenterConfirmCount = 0;
uint8_t serveFoodTakenReleased = 0;

uint8_t dockSequenceActive = 0;
DockState dockState = DOCK_STATE_NONE;
uint32_t dockStateStartTime = 0;
uint8_t dockUturnCenterConfirmCount = 0;
uint8_t dockReverseTConfirmCount = 0;

/* ============================================================
   MOTOR SETUP

   Right motor = Motor 1:
   PB6 = TIM4_CH1 = Right motor IN1 / RPWM
   PB7 = TIM4_CH2 = Right motor IN2 / LPWM
   PB13 = Right motor EN

   Left motor = Motor 2:
   PB8 = TIM4_CH3 = Left motor IN1 / RPWM
   PB9 = TIM4_CH4 = Left motor IN2 / LPWM
   PB14 = Left motor EN
   ============================================================ */

#define PWM_MAX_COUNT 399

#define RIGHT_MOTOR_INVERT 0
#define LEFT_MOTOR_INVERT  1

#define RIGHT_EN_GPIO_Port GPIOB
#define RIGHT_EN_Pin       GPIO_PIN_13

#define LEFT_EN_GPIO_Port  GPIOB
#define LEFT_EN_Pin        GPIO_PIN_14

/* ============================================================
   SOFTWARE ENCODERS

   Right motor encoder:
   A = PB5
   B = PB15

   Left motor encoder:
   A = PA8
   B = PA12
   ============================================================ */

volatile int32_t rightEncoderCount = 0;
volatile int32_t leftEncoderCount  = 0;

uint8_t lastRightEncA = 0;
uint8_t lastLeftEncA  = 0;

/* ============================================================
   PID SETTINGS FOR JGB37

   If correction is still slow, increase Kp slightly.
   If it shakes left/right, reduce Kd or Kp slightly.
   ============================================================ */

float Kp = 0.068f;
float Ki = 0.00f;
float Kd = 0.016f;

float integral = 0.0f;
float lastError = 0.0f;

/* Smoothing variables:
   lastError keeps raw direction for turn recovery.
   filteredError/lastPidError are only used to make PID correction smoother. */
float filteredError = 0.0f;
float lastPidError = 0.0f;
float filteredCorrection = 0.0f;

int baseSpeed = 36;
int maxSpeed  = 68;

/* Turning support
   HARD_TURN_ERROR is set above the possible line-error range so normal
   bends are handled by PID only. Route junction turns still use
   ROUTE_TURN_* settings, and line-lost recovery still remains active. */
#define TURN_ERROR          120
#define HARD_TURN_ERROR     999
#define LINE_LOST_HOLD_MS   1600
#define TURN_OUTER_SPEED    66
#define TURN_INNER_SPEED    0
#define RECOVERY_SPEED      55

/* Smooth line adjustment.
   Normal PID uses small ramp to avoid jerky/skidding corrections.
   Turns/recovery use larger ramp so turns still complete. */
#define NORMAL_RAMP_STEP    5
#define TURN_RAMP_STEP      8
#define RECOVERY_RAMP_STEP  8

#define ERROR_FILTER_ALPHA       0.35f
#define CORRECTION_FILTER_ALPHA  0.35f

/* Turn-hold logic only.
   H6 is disabled, so H5 is leftmost working sensor.
   Start turn hold when edge/large-error turn is detected.
   Keep recovery active until center sensor catches the line, or timeout. */
#define TURN_HOLD_MS        8000
#define TURN_EXIT_ERROR     70
#define TURN_REENTRY_ERROR  90
#define TURN_CENTER_CONFIRM_COUNT 3
#define LEFT_EDGE_INDEX     1   // L1/H5, effective leftmost working sensor
#define CENTER_INDEX        3   // L3/H1, center sensor
#define RIGHT_EDGE_INDEX    6   // L6/H3, rightmost sensor

uint8_t hasSeenLine = 0;
uint32_t lastLineSeenTime = 0;
uint32_t lastDebugPrint = 0;

uint8_t turnHoldActive = 0;
int8_t turnHoldDirection = 0;   // -1 = left turn, +1 = right turn
uint32_t turnHoldStartTime = 0;  // refreshed while the robot is still seeing turn evidence
uint8_t turnCenterConfirmCount = 0;
uint8_t pidInitialized = 0;

/* Actual commanded motor speeds used for ramping. */
int currentLeftSpeed = 0;
int currentRightSpeed = 0;

/* ============================================================
   ESP32 UART COMMAND PROTOCOL

   USART1 PA9  = STM32 TX -> ESP32 RX IO18
   USART1 PA10 = STM32 RX <- ESP32 TX IO17

   Supported commands from ESP32:
   PING\n                  -> PONG\n
   ROUTE:2,0,3,0,1,4\n  -> loads route and starts robot

   STOP\n                  -> stops robot and finishes route

   Command values follow RouteCommand enum:
   0 = STRAIGHT
   1 = LEFT
   2 = RIGHT
   3 = SERVE_STOP
   4 = DOCK
   ============================================================ */

#define ESP_RX_LINE_MAX 96

char espRxLine[ESP_RX_LINE_MAX];
uint8_t espRxIndex = 0;

/* Live navigation event reporting to ESP32 / web app */
uint8_t routeDestinationTable = 0;
uint32_t routeJourneyStartTime = 0;
uint8_t routeEventSeq = 0;
uint8_t routeBend1Sent = 0;
uint8_t routeLastBendSent = 0;
uint8_t manualReturnActive = 0;
uint8_t chargerDockDetected = 0;   /* Set by ESP32 CHARGER_ON command during dock parking */


/* ============================================================
   ESP32 UART COMMAND HANDLING
   ============================================================ */

void EspSendText(const char *text)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)text, strlen(text), 100);
}

void EspSendEvent(const char *eventName)
{
  char msg[64];
  routeEventSeq++;
  snprintf(msg, sizeof(msg), "EVT:%u:%s\n", routeEventSeq, eventName);
  EspSendText(msg);
}

uint8_t DetectDestinationTableFromRoute(void)
{
  /*
    Route patterns used by ESP32:
    Table 1: ROUTE:0,3,0,4
    Table 2: ROUTE:2,2,3,1,1,4
    Table 3: ROUTE:2,0,3,0,1,4
  */
  if (routeCommandCount >= 4 &&
      routeCommands[0] == ROUTE_CMD_STRAIGHT &&
      routeCommands[1] == ROUTE_CMD_SERVE_STOP)
  {
    return 1;
  }

  if (routeCommandCount >= 6 &&
      routeCommands[0] == ROUTE_CMD_RIGHT &&
      routeCommands[1] == ROUTE_CMD_RIGHT)
  {
    return 2;
  }

  if (routeCommandCount >= 6 &&
      routeCommands[0] == ROUTE_CMD_RIGHT &&
      routeCommands[1] == ROUTE_CMD_STRAIGHT)
  {
    return 3;
  }

  return 0;
}

void ReportRouteJunctionEvent(void)
{
  if (manualReturnActive)
  {
    EspSendEvent("RETURN_JUNCTION");
    return;
  }

  /*
    routeIndex is already incremented before this function is called.

    Forward:
      Table 1 route: 0,3,0,4
        routeIndex 1 = J1
      Table 2 route: 2,2,3,1,1,4
        routeIndex 1 = J1, routeIndex 2 = J2
      Table 3 route: 2,0,3,0,1,4
        routeIndex 1 = J1, routeIndex 2 = J2

    Return:
      Table 1: routeIndex 3 = RETURN_J1
      Table 2/3: routeIndex 4 = RETURN_J2, routeIndex 5 = RETURN_J1
  */
  if (routeIndex == 1)
  {
    EspSendEvent("J1");
  }
  else if (routeIndex == 2 && routeDestinationTable != 1)
  {
    EspSendEvent("J2");
  }
  else if (routeDestinationTable == 1 && routeIndex == 3)
  {
    EspSendEvent("RETURN_J1");
  }
  else if ((routeDestinationTable == 2 || routeDestinationTable == 3) && routeIndex == 4)
  {
    EspSendEvent("RETURN_J2");
  }
  else if ((routeDestinationTable == 2 || routeDestinationTable == 3) && routeIndex == 5)
  {
    EspSendEvent("RETURN_J1");
  }
}

void ReportRouteTableEvent(void)
{
  if (routeDestinationTable == 1)
  {
    EspSendEvent("TABLE1");
  }
  else if (routeDestinationTable == 2)
  {
    EspSendEvent("TABLE2");
  }
  else if (routeDestinationTable == 3)
  {
    EspSendEvent("TABLE3");
  }
  else
  {
    EspSendEvent("TABLE");
  }
}

void UpdateTimedRouteEvents(void)
{
  if (routeFinished || routeError || routeJourneyStartTime == 0)
  {
    return;
  }

  uint32_t elapsed = HAL_GetTick() - routeJourneyStartTime;

  /*
    First bend is not a marker; it is a normal curve handled by PID.
    So this event is time-based, using the measured 7 second value.
  */
  if (!routeBend1Sent && elapsed >= 7000)
  {
    routeBend1Sent = 1;
    EspSendEvent("BEND1");
  }

  /*
    Table 3 has route: RIGHT, STRAIGHT, SERVE_STOP, STRAIGHT, LEFT, DOCK.

    The second junction command is STRAIGHT. Because the robot physically
    passes straight through this point, the Hall pattern can sometimes look
    like a normal line/bend instead of a strong T/cross/clear-90 marker.
    When that happens, routeIndex stays at 1. Later the Table 3 stop T is
    incorrectly consumed as J2, so the web app shows J2 at the table and the
    robot does not stop until the marker is seen again.

    LAST_BEND is already a time-based event placed after J2. Therefore, if
    Table 3 reaches LAST_BEND time while J2 is still not consumed, safely
    consume the missed STRAIGHT command first and report J2, then report
    LAST_BEND. This does not change the route/mapping numbers; it only fixes
    the missed straight-through J2 case.
  */
  if (routeDestinationTable == 3 && !routeLastBendSent && elapsed >= 27500)
  {
    if (routeIndex == 1 && routeCommandCount >= 3 &&
        routeCommands[1] == ROUTE_CMD_STRAIGHT &&
        routeCommands[2] == ROUTE_CMD_SERVE_STOP)
    {
      routeIndex++;              /* consume missed J2 STRAIGHT command */
      ReportRouteJunctionEvent(); /* sends J2 */
    }

    routeLastBendSent = 1;
    EspSendEvent("LAST_BEND");
  }
}

void ResetRouteRuntimeState(void)
{
  uint32_t now = HAL_GetTick();

  routeIndex = 0;
  routePaused = 0;
  manualReturnActive = 0;
  chargerDockDetected = 0;
  routeFinished = 0;
  routeError = 0;
  stoppedAtTJunction = 0;
  stopMarkerCount = 0;
  routeJunctionDirectCounter = 0;

  routeJourneyStartTime = now;
  routeEventSeq = 0;
  routeBend1Sent = 0;
  routeLastBendSent = 0;

  wideMarkerActive = 0;
  wideMarkerClearCounter = 0;
  markerIgnoreUntil = now + ROUTE_START_MARKER_IGNORE_MS;
  routeStartIgnoreUntil = now + ROUTE_START_MARKER_IGNORE_MS;
  lastMarkerEventTime = now;
  tJunctionCounter = 0;

  stopCrossActive = 0;
  stopCrossConfirmCounter = 0;
  stopCrossClearCounter = 0;
  lastStopCrossDetected = 0;

  executingRouteJunctionStop = 0;
  routeJunctionStopDirection = 0;
  routeJunctionStopStartTime = 0;

  executingRouteTurn = 0;
  routeTurnDirection = 0;
  routeTurnStartTime = 0;
  routeTurnCenterConfirmCount = 0;

  executingRouteStraight = 0;
  routeStraightStartTime = 0;

  /* New route starts from the dock/start T marker.
     Move forward briefly first and ignore markers, so the dock T is not
     consumed as the first route junction. */
  executingRouteStartExit = 1;
  routeStartExitStartTime = now;

  serveSequenceActive = 0;
  serveState = SERVE_STATE_NONE;
  serveStateStartTime = 0;
  serveUturnCenterConfirmCount = 0;
  serveFoodTakenReleased = 0;

  dockSequenceActive = 0;
  dockState = DOCK_STATE_NONE;
  dockStateStartTime = 0;
  dockUturnCenterConfirmCount = 0;
  dockReverseTConfirmCount = 0;

  MotorStop();
  ResetFollowStateAfterStop();
}

uint8_t LoadRouteFromCsv(char *csv)
{
  uint8_t count = 0;
  char *token = strtok(csv, ",");

  while (token != NULL && count < ROUTE_MAX_COMMANDS)
  {
    while (*token == ' ' || *token == '\t') token++;

    int value = atoi(token);

    if (value < ROUTE_CMD_STRAIGHT || value > ROUTE_CMD_DOCK)
    {
      return 0;
    }

    routeCommands[count++] = (RouteCommand)value;
    token = strtok(NULL, ",");
  }

  if (count == 0)
  {
    return 0;
  }

  routeCommandCount = count;
  routeDestinationTable = DetectDestinationTableFromRoute();
  ResetRouteRuntimeState();
  return 1;
}


uint8_t BuildReturnRouteFromCurrentPosition(void)
{
  uint8_t oldDestination = routeDestinationTable;
  uint8_t oldIndex = routeIndex;
  uint8_t count = 0;

  /*
    Best-effort return path from latest consumed route index.
    This assumes the robot is still on the line.

    routeIndex meaning:
      0 = before first junction command consumed
      1 = after first junction command consumed
      2 = after second junction command consumed
      3+ = table / return phase
  */

  if (oldIndex == 0)
  {
    routeCommands[count++] = ROUTE_CMD_DOCK;
  }
  else if (oldDestination == 1)
  {
    routeCommands[count++] = ROUTE_CMD_STRAIGHT;
    routeCommands[count++] = ROUTE_CMD_DOCK;
  }
  else if (oldDestination == 2)
  {
    if (oldIndex <= 1)
    {
      routeCommands[count++] = ROUTE_CMD_LEFT;
      routeCommands[count++] = ROUTE_CMD_DOCK;
    }
    else
    {
      routeCommands[count++] = ROUTE_CMD_LEFT;
      routeCommands[count++] = ROUTE_CMD_LEFT;
      routeCommands[count++] = ROUTE_CMD_DOCK;
    }
  }
  else if (oldDestination == 3)
  {
    if (oldIndex <= 1)
    {
      routeCommands[count++] = ROUTE_CMD_LEFT;
      routeCommands[count++] = ROUTE_CMD_DOCK;
    }
    else
    {
      routeCommands[count++] = ROUTE_CMD_STRAIGHT;
      routeCommands[count++] = ROUTE_CMD_LEFT;
      routeCommands[count++] = ROUTE_CMD_DOCK;
    }
  }
  else
  {
    routeCommands[count++] = ROUTE_CMD_DOCK;
  }

  routeCommandCount = count;
  routeIndex = 0;
  routeDestinationTable = oldDestination;
  manualReturnActive = 1;
  chargerDockDetected = 0;

  return (count > 0);
}

uint8_t StartImmediateReturnToDock(void)
{
  uint32_t now = HAL_GetTick();

  if (!BuildReturnRouteFromCurrentPosition())
  {
    return 0;
  }

  routeFinished = 0;
  routeError = 0;
  stoppedAtTJunction = 0;
  stopMarkerCount = 0;
  routeJunctionDirectCounter = 0;

  executingRouteStartExit = 0;
  routeStartExitStartTime = 0;
  routeStartIgnoreUntil = 0;

  executingRouteJunctionStop = 0;
  routeJunctionStopDirection = 0;
  routeJunctionStopStartTime = 0;

  executingRouteTurn = 0;
  routeTurnDirection = 0;
  routeTurnStartTime = 0;
  routeTurnCenterConfirmCount = 0;

  executingRouteStraight = 0;
  routeStraightStartTime = 0;

  dockSequenceActive = 0;
  dockState = DOCK_STATE_NONE;
  dockStateStartTime = 0;
  dockUturnCenterConfirmCount = 0;
  dockReverseTConfirmCount = 0;

  serveSequenceActive = 1;
  serveState = SERVE_STATE_REVERSE;
  serveStateStartTime = now;
  serveUturnCenterConfirmCount = 0;
  serveFoodTakenReleased = 0;

  markerIgnoreUntil = now + MARKER_IGNORE_AFTER_COMMAND_MS;
  lastMarkerEventTime = now;
  wideMarkerActive = 1;
  wideMarkerClearCounter = 0;
  stopCrossActive = 1;
  stopCrossClearCounter = 0;

  ClearTurnHoldState();
  ResetPidFiltersFromError(0);
  MotorStop();

  EspSendEvent("RETURN_REQUESTED");
  EspSendEvent("REVERSING");

  return 1;
}


void ProcessEspCommand(char *line)
{
  /* Remove carriage return / newline. */
  for (uint8_t i = 0; line[i] != '\0'; i++)
  {
    if (line[i] == '\r' || line[i] == '\n')
    {
      line[i] = '\0';
      break;
    }
  }

  if (strncmp(line, "PING", 4) == 0)
  {
    EspSendText("PONG\n");
    return;
  }

  if (strncmp(line, "PAUSE", 5) == 0)
  {
    if (!routeFinished && !routeError)
    {
      routePaused = 1;
      MotorStop();
      EspSendText("OK:PAUSE\n");
      EspSendEvent("PAUSED");
    }
    else
    {
      EspSendText("ERR:NOT_RUNNING\n");
    }
    return;
  }

  if (strncmp(line, "RESUME", 6) == 0)
  {
    if (routePaused)
    {
      routePaused = 0;
      ResetFollowStateAfterStop();
      ResetPidFiltersFromError(0);
      EspSendText("OK:RESUME\n");
      EspSendEvent("RESUMED");
    }
    else
    {
      EspSendText("OK:RESUME\n");
    }
    return;
  }

  if (strncmp(line, "HALL_READ", 9) == 0)
  {
    SendHallSnapshot("HALL");
    return;
  }

  if (strncmp(line, "HALL_UPDATE", 11) == 0)
  {
    Read_All_Hall_Sensors();

    for (uint8_t i = 0; i < 7; i++)
    {
      hallBaseline[i] = hallRaw[i];
    }

    SendHallSnapshot("HALL_UPDATED");
    EspSendText("OK:HALL_UPDATE\n");
    return;
  }

  if (strncmp(line, "RETURN_NOW", 10) == 0)
  {
    if (serveSequenceActive && serveState == SERVE_STATE_WAIT)
    {
      serveFoodTakenReleased = 1;
      EspSendText("OK:RETURN\n");
      EspSendEvent("RETURN_REQUESTED");
    }
    else if (routeFinished && !routeError)
    {
      EspSendText("OK:ALREADY_DONE\n");
    }
    else
    {
      if (StartImmediateReturnToDock())
      {
        EspSendText("OK:RETURN\n");
      }
      else
      {
        EspSendText("ERR:RETURN_FAILED\n");
      }
    }
    return;
  }

  if (strncmp(line, "FOOD_TAKEN", 10) == 0 || strncmp(line, "CONTINUE", 8) == 0)
  {
    if (serveSequenceActive && serveState == SERVE_STATE_WAIT)
    {
      serveFoodTakenReleased = 1;
      EspSendText("OK:FOOD_TAKEN\n");
      EspSendEvent("FOOD_TAKEN");
    }
    else
    {
      EspSendText("ERR:NOT_SERVING\n");
    }
    return;
  }

  if (strncmp(line, "CHARGER_ON", 10) == 0)
  {
    /* ESP32 IO41 detected charger/pogo voltage.
       Do not disturb normal route logic; this flag is only used during
       DOCK_STATE_REVERSE_TO_T so parking can finish by charger contact
       even if the T marker angle is not perfect. */
    chargerDockDetected = 1;
    EspSendText("OK:CHARGER_ON\n");
    return;
  }

  if (strncmp(line, "CHARGER_OFF", 11) == 0)
  {
    chargerDockDetected = 0;
    EspSendText("OK:CHARGER_OFF\n");
    return;
  }

  if (strncmp(line, "STOP", 4) == 0)
  {
    routeFinished = 1;
    routePaused = 0;
    stoppedAtTJunction = 1;
    routeError = 0;
    routeIndex = 0;
    routeCommandCount = 0;
    executingRouteStartExit = 0;
    routeStartExitStartTime = 0;
    routeStartIgnoreUntil = 0;
    ClearTurnHoldState();
    ResetFollowStateAfterStop();
    routeJourneyStartTime = 0;
    MotorStop();
    EspSendText("OK:STOP\n");
    EspSendEvent("STOPPED");
    return;
  }

  if (strncmp(line, "ROUTE:", 6) == 0)
  {
    if (LoadRouteFromCsv(line + 6))
    {
      EspSendText("OK:ROUTE\n");
      EspSendEvent("START");
    }
    else
    {
      EspSendText("ERR:BAD_ROUTE\n");
    }
    return;
  }

  EspSendText("ERR:UNKNOWN\n");
}

void CheckEspCommand(void)
{
  uint8_t ch;

  while (HAL_UART_Receive(&huart1, &ch, 1, 0) == HAL_OK)
  {
    if (ch == '\n' || ch == '\r')
    {
      if (espRxIndex > 0)
      {
        espRxLine[espRxIndex] = '\0';
        ProcessEspCommand(espRxLine);
        espRxIndex = 0;
      }
    }
    else
    {
      if (espRxIndex < (ESP_RX_LINE_MAX - 1))
      {
        espRxLine[espRxIndex++] = (char)ch;
      }
      else
      {
        espRxIndex = 0;
        EspSendText("ERR:LINE_TOO_LONG\n");
      }
    }
  }
}

/* ============================================================
   ADC READ
   ============================================================ */

uint16_t Read_ADC_Channel(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_ADC_Start(&hadc1);

  if (HAL_ADC_PollForConversion(&hadc1, 20) != HAL_OK)
  {
    HAL_ADC_Stop(&hadc1);
    return 0;
  }

  uint16_t value = HAL_ADC_GetValue(&hadc1);

  HAL_ADC_Stop(&hadc1);

  return value;
}

void Read_All_Hall_Sensors(void)
{
  for (int i = 0; i < 7; i++)
  {
    hallRaw[i] = Read_ADC_Channel(hallChannels[i]);
  }
}


void SendHallSnapshot(const char *prefix)
{
  char msg[180];

  Read_All_Hall_Sensors();

  snprintf(msg, sizeof(msg),
           "%s:H0=%u,H1=%u,H2=%u,H3=%u,H4=%u,H5=%u,H6=%u,B0=%u,B1=%u,B2=%u,B3=%u,B4=%u,B5=%u,B6=%u\n",
           prefix,
           hallRaw[0], hallRaw[1], hallRaw[2], hallRaw[3], hallRaw[4], hallRaw[5], hallRaw[6],
           hallBaseline[0], hallBaseline[1], hallBaseline[2], hallBaseline[3], hallBaseline[4], hallBaseline[5], hallBaseline[6]);

  EspSendText(msg);
}


/* ============================================================
   LINE ERROR CALCULATION
   ============================================================ */

int Calculate_Line_Error(int *errorOut, int *strengthOut)
{
  int32_t weightedSum = 0;
  int32_t strengthSum = 0;

  activeCount = 0;

  for (int i = 0; i < 7; i++)
  {
#if DISABLE_H6_SENSOR
    /*
      Disable L0/H6 only.
      physicalOrder[0] = H6, so H6 will not affect:
      - error calculation
      - line detection strength
      - active sensor count
      - T-junction detection
    */
    if (i == 0)
    {
      activePhysical[i] = 0;
      continue;
    }
#endif

    uint8_t h = physicalOrder[i];

    int diff = (int)hallRaw[h] - (int)hallBaseline[h];

    if (diff < 0)
    {
      diff = -diff;
    }

    if (diff >= MAG_THRESHOLD)
    {
      activePhysical[i] = 1;
      activeCount++;

      weightedSum += diff * weights[i];
      strengthSum += diff;
    }
    else
    {
      activePhysical[i] = 0;
    }
  }

  lastStrengthSum = strengthSum;
  *strengthOut = strengthSum;

  if (strengthSum < MIN_LINE_STRENGTH)
  {
    *errorOut = 0;
    return 0;
  }

  *errorOut = weightedSum / strengthSum;

  return 1;
}

/* ============================================================
   T JUNCTION DETECTION
   ============================================================ */

int Detect_T_Junction(void)
{
  int leftGroup  = activePhysical[0] + activePhysical[1] + activePhysical[2];
  int center     = activePhysical[3];
  int rightGroup = activePhysical[4] + activePhysical[5] + activePhysical[6];

  if (lastStrengthSum < T_MIN_STRENGTH)
  {
    return 0;
  }

  if (activeCount >= T_ACTIVE_COUNT && center && leftGroup >= 1 && rightGroup >= 1)
  {
    return 1;
  }

  if (activeCount >= 6)
  {
    return 1;
  }

  return 0;
}

int Detect_Stop_Cross_Marker(void)
{
  /*
    STOP marker must be stricter than a normal T/L bend.

    Natural right/left turns usually activate center + one side only.
    A stop cross should activate both left and right groups together,
    with a stronger total magnetic strength.

    H6/L0 is disabled, so the effective left side starts from L1/H5.
  */
  int leftGroup  = activePhysical[1] + activePhysical[2];
  int center     = activePhysical[CENTER_INDEX];
  int rightGroup = activePhysical[4] + activePhysical[5] + activePhysical[6];

  if (lastStrengthSum < STOP_CROSS_MIN_STRENGTH)
  {
    return 0;
  }

  if (center && leftGroup >= 1 && rightGroup >= 1 &&
      activeCount >= STOP_CROSS_ACTIVE_COUNT)
  {
    return 1;
  }

  /* Very strong edge-to-edge cross case. */
  if (activePhysical[LEFT_EDGE_INDEX] && activePhysical[RIGHT_EDGE_INDEX] &&
      activeCount >= 4)
  {
    return 1;
  }

  return 0;
}

int Detect_Clear_90_Joint(void)
{
  /*
    Detect a clear 90-degree / branch decision joint.

    This covers the case where the robot travels along the head of a T/bus
    branch. At first it may only see center + one side path, not both left
    and right at the same time.

    To avoid consuming route commands on normal 30-degree segmented bends,
    this requires stronger side evidence: center line present, multiple
    active sensors, and one edge/side group active.
  */
  int leftGroup  = activePhysical[LEFT_EDGE_INDEX] + activePhysical[2];
  int center     = activePhysical[CENTER_INDEX];
  int rightGroup = activePhysical[4] + activePhysical[5] + activePhysical[RIGHT_EDGE_INDEX];

  if (lastStrengthSum < CLEAR_90_MIN_STRENGTH)
  {
    return 0;
  }

  if (!center || activeCount < CLEAR_90_MIN_ACTIVE)
  {
    return 0;
  }

  /* Left branch / left 90 joint. */
  if (activePhysical[LEFT_EDGE_INDEX] && leftGroup >= 2)
  {
    return 1;
  }

  /* Right branch / right 90 joint. */
  if (activePhysical[RIGHT_EDGE_INDEX] && rightGroup >= 2)
  {
    return 1;
  }

  return 0;
}

/* ============================================================
   ROUTE / MARKER HELPERS
   ============================================================ */

RouteCommand GetCurrentRouteCommand(void)
{
  if (routeIndex < routeCommandCount)
  {
    return routeCommands[routeIndex];
  }

  /* Safety: if route is finished/missing, stop instead of guessing. */
  return ROUTE_CMD_STOP;
}

uint8_t IsJunctionRouteCommand(RouteCommand cmd)
{
  if (cmd == ROUTE_CMD_LEFT || cmd == ROUTE_CMD_RIGHT || cmd == ROUTE_CMD_STRAIGHT)
  {
    return 1;
  }

  return 0;
}

uint8_t IsStrongRouteJunctionCandidate(int tDetected, int stopCrossDetected, int clear90Detected)
{
  /*
    Route junctions and cross markers both make a wide sensor pattern.
    For LEFT/RIGHT/STRAIGHT commands, this is used only to consume the
    current route command and stop before executing the turn.

    This fixes the case from the log where the wide/cross shape was detected
    (W:1 and SC:1), but routeIndex stayed at 0, so the final stop marker
    was not counted as STOP.
  */
  if (tDetected)
  {
    return 1;
  }

  if (stopCrossDetected && activeCount >= STOP_CROSS_ACTIVE_COUNT)
  {
    return 1;
  }

  if (clear90Detected)
  {
    return 1;
  }

  return 0;
}

void ClearTurnHoldState(void)
{
  turnHoldActive = 0;
  turnHoldDirection = 0;
  turnHoldStartTime = 0;
  turnCenterConfirmCount = 0;
}

void ResetPidFiltersFromError(int error)
{
  integral = 0.0f;
  lastError = (float)error;
  filteredError = (float)error;
  lastPidError = (float)error;
  filteredCorrection = 0.0f;
  pidInitialized = 1;
}

void StartRouteJunctionStop(int8_t direction)
{
  /*
    The robot may be slightly tilted when it reaches a T-junction.
    Stop first, then execute the route command. This prevents normal PID from
    accidentally entering the wrong branch before the command is applied.
  */
  executingRouteJunctionStop = 1;
  routeJunctionStopDirection = direction;
  routeJunctionStopStartTime = HAL_GetTick();

  if (direction > 0)
  {
    EspSendEvent("TURN_RIGHT");
  }
  else
  {
    EspSendEvent("TURN_LEFT");
  }

  executingRouteTurn = 0;
  routeTurnDirection = 0;
  routeTurnCenterConfirmCount = 0;

  executingRouteStraight = 0;

  ClearTurnHoldState();
  ResetPidFiltersFromError(0);

  wideMarkerActive = 1;
  wideMarkerClearCounter = 0;
  lastMarkerEventTime = HAL_GetTick();
  markerIgnoreUntil = HAL_GetTick() + MARKER_IGNORE_AFTER_COMMAND_MS;

  stopCrossActive = 0;
  stopCrossConfirmCounter = 0;
  stopCrossClearCounter = 0;

  MotorStop();
}

void StartRouteTurn(int8_t direction)
{
  executingRouteJunctionStop = 0;
  routeJunctionStopDirection = 0;

  executingRouteTurn = 1;
  routeTurnDirection = direction;
  routeTurnStartTime = HAL_GetTick();
  routeTurnCenterConfirmCount = 0;

  executingRouteStraight = 0;

  ClearTurnHoldState();
  ResetPidFiltersFromError(direction < 0 ? -TURN_ERROR : TURN_ERROR);

  /* Do not count the same wide T/cross as the next marker. */
  wideMarkerActive = 1;
  wideMarkerClearCounter = 0;
  lastMarkerEventTime = HAL_GetTick();
  markerIgnoreUntil = HAL_GetTick() + MARKER_IGNORE_AFTER_COMMAND_MS;
}

void FinishRouteTurn(int error)
{
  executingRouteJunctionStop = 0;
  routeJunctionStopDirection = 0;

  executingRouteTurn = 0;
  routeTurnDirection = 0;
  routeTurnCenterConfirmCount = 0;

  ClearTurnHoldState();
  ResetPidFiltersFromError(error);

  /* After the commanded T-turn has found the branch, let normal PID handle
     the following L/right turn. Do not count that immediate line pattern as
     a destination stop cross. */
  markerIgnoreUntil = HAL_GetTick() + ROUTE_AFTER_TURN_IGNORE_MS;
  stopCrossActive = 0;
  stopCrossConfirmCounter = 0;
  stopCrossClearCounter = 0;
}

void StartRouteStraightPass(void)
{
  executingRouteJunctionStop = 0;
  routeJunctionStopDirection = 0;

  executingRouteStraight = 1;
  routeStraightStartTime = HAL_GetTick();

  ClearTurnHoldState();

  /* Do not count the same cross/junction again while passing over it. */
  wideMarkerActive = 1;
  wideMarkerClearCounter = 0;
  lastMarkerEventTime = HAL_GetTick();
  markerIgnoreUntil = HAL_GetTick() + MARKER_COOLDOWN_MS;
}

void StartServeStopSequence(void)
{
  serveSequenceActive = 1;
  serveState = SERVE_STATE_WAIT;
  serveStateStartTime = HAL_GetTick();
  serveUturnCenterConfirmCount = 0;
  serveFoodTakenReleased = 0;

  executingRouteJunctionStop = 0;
  executingRouteTurn = 0;
  executingRouteStraight = 0;

  ClearTurnHoldState();
  ResetPidFiltersFromError(0);

  /* Do not recount the same table T marker while serving/reversing. */
  wideMarkerActive = 1;
  wideMarkerClearCounter = 0;
  stopCrossActive = 1;
  stopCrossClearCounter = 0;
  lastMarkerEventTime = HAL_GetTick();
  markerIgnoreUntil = HAL_GetTick() + MARKER_IGNORE_AFTER_COMMAND_MS;

  EspSendEvent("SERVING");

  MotorStop();
}

void StartDockStopTurnSequence(void)
{
  dockSequenceActive = 1;
  dockState = DOCK_STATE_WAIT;
  dockStateStartTime = HAL_GetTick();
  dockUturnCenterConfirmCount = 0;
  dockReverseTConfirmCount = 0;

  EspSendEvent("DOCK_REVERSING");

  executingRouteJunctionStop = 0;
  executingRouteTurn = 0;
  executingRouteStraight = 0;

  ClearTurnHoldState();
  ResetPidFiltersFromError(0);

  wideMarkerActive = 1;
  wideMarkerClearCounter = 0;
  stopCrossActive = 1;
  stopCrossClearCounter = 0;
  lastMarkerEventTime = HAL_GetTick();
  markerIgnoreUntil = HAL_GetTick() + MARKER_IGNORE_AFTER_COMMAND_MS;

  EspSendEvent("DOCK_TURNING");

  MotorStop();
}

void StartDockReverseToT(void)
{
  /*
    After the dock U-turn, the front sensor array may be ahead of the dock T.
    Reverse slowly until the same dock T/wide marker comes under the front
    sensor array again. Then the robot stops with its back side placed toward
    the charger/pogo pins.

    Start reverse from a full motor stop. If we ramp directly from pivot speed
    to reverse speed, one wheel can momentarily act like a center pivot.
  */
  MotorStop();
  dockSequenceActive = 1;
  dockState = DOCK_STATE_REVERSE_TO_T;
  dockStateStartTime = HAL_GetTick();
  dockReverseTConfirmCount = 0;

  executingRouteJunctionStop = 0;
  executingRouteTurn = 0;
  executingRouteStraight = 0;

  ClearTurnHoldState();
  ResetPidFiltersFromError(0);

  /* Use raw T detection during reverse docking, not normal marker event pulses. */
  wideMarkerActive = 0;
  wideMarkerClearCounter = 0;
  stopCrossActive = 0;
  stopCrossClearCounter = 0;
}

void FinishDockParking(void)
{
  dockSequenceActive = 0;
  dockState = DOCK_STATE_NONE;
  routeFinished = 1;
  stoppedAtTJunction = 1;
  manualReturnActive = 0;
  chargerDockDetected = 0;
  EspSendEvent("DOCKED");
  routeJourneyStartTime = 0;
  MotorStop();
}

uint8_t IsUTurnLineLocked(int lineDetected, int tDetected, int error)
{
  if (!lineDetected)
  {
    return 0;
  }

  /* Do not finish while still on the wide T/cross marker. */
  if (tDetected || activeCount > UTURN_LOCK_MAX_ACTIVE)
  {
    return 0;
  }

  if (activePhysical[CENTER_INDEX] && abs(error) <= UTURN_LOCK_ERROR)
  {
    return 1;
  }

  if (activeCount >= 1 && abs(error) <= UTURN_LOCK_ERROR)
  {
    return 1;
  }

  return 0;
}

uint8_t UpdateWideMarkerDetection(int tDetected)
{
  uint8_t markerEvent = 0;
  uint32_t now = HAL_GetTick();

  /* Confirm wide/T marker for a few loops. */
  if (tDetected)
  {
    if (tJunctionCounter < 255)
    {
      tJunctionCounter++;
    }
  }
  else
  {
    tJunctionCounter = 0;
  }

  uint8_t wideConfirmed = (tJunctionCounter >= T_CONFIRM_COUNT);

  if (wideConfirmed)
  {
    if (!wideMarkerActive && now >= markerIgnoreUntil &&
        ((now - lastMarkerEventTime) >= MARKER_COOLDOWN_MS))
    {
      markerEvent = 1;
      wideMarkerActive = 1;
      wideMarkerClearCounter = 0;
      lastMarkerEventTime = now;
    }
  }
  else
  {
    if (wideMarkerActive)
    {
      if (wideMarkerClearCounter < 255)
      {
        wideMarkerClearCounter++;
      }

      if (wideMarkerClearCounter >= MARKER_CLEAR_CONFIRM_COUNT)
      {
        wideMarkerActive = 0;
        wideMarkerClearCounter = 0;
      }
    }
    else
    {
      wideMarkerClearCounter = 0;
    }
  }

  return markerEvent;
}

uint8_t UpdateStopCrossMarkerDetection(int crossDetected)
{
  uint8_t markerEvent = 0;
  uint32_t now = HAL_GetTick();

  lastStopCrossDetected = crossDetected ? 1 : 0;

  if (crossDetected)
  {
    if (stopCrossConfirmCounter < 255)
    {
      stopCrossConfirmCounter++;
    }
  }
  else
  {
    stopCrossConfirmCounter = 0;
  }

  uint8_t crossConfirmed = (stopCrossConfirmCounter >= STOP_CROSS_CONFIRM_COUNT);

  if (crossConfirmed)
  {
    if (!stopCrossActive && now >= markerIgnoreUntil &&
        ((now - lastMarkerEventTime) >= MARKER_COOLDOWN_MS))
    {
      markerEvent = 1;
      stopCrossActive = 1;
      stopCrossClearCounter = 0;
      lastMarkerEventTime = now;
    }
  }
  else
  {
    if (stopCrossActive)
    {
      if (stopCrossClearCounter < 255)
      {
        stopCrossClearCounter++;
      }

      if (stopCrossClearCounter >= STOP_CROSS_CLEAR_CONFIRM_COUNT)
      {
        stopCrossActive = 0;
        stopCrossClearCounter = 0;
      }
    }
    else
    {
      stopCrossClearCounter = 0;
    }
  }

  return markerEvent;
}

/* ============================================================
   MOTOR CONTROL
   Robot speed convention:
   leftSpeed  > 0 means left wheel forward
   rightSpeed > 0 means right wheel forward
   ============================================================ */

int clamp_int(int value, int minVal, int maxVal)
{
  if (value > maxVal) return maxVal;
  if (value < minVal) return minVal;
  return value;
}

int move_towards_int(int current, int target, int step)
{
  if (step < 1) step = 1;

  if (current < target)
  {
    current += step;
    if (current > target) current = target;
  }
  else if (current > target)
  {
    current -= step;
    if (current < target) current = target;
  }

  return current;
}

void ResetFollowStateAfterStop(void)
{
  /*
    Important after the robot is lifted or taken out of the line:
    clear stale lastError/turn state so when it is placed back,
    it starts fresh and does not recover in the old/opposite direction.
  */
  hasSeenLine = 0;
  lastLineSeenTime = 0;

  integral = 0.0f;
  lastError = 0.0f;
  filteredError = 0.0f;
  lastPidError = 0.0f;
  filteredCorrection = 0.0f;

  turnHoldActive = 0;
  turnHoldDirection = 0;
  turnHoldStartTime = 0;
  turnCenterConfirmCount = 0;
  pidInitialized = 0;
}

void DriveRightMotor(int speed)
{
  if (RIGHT_MOTOR_INVERT)
  {
    speed = -speed;
  }

  speed = clamp_int(speed, -100, 100);

  int absSpeed = speed;
  if (absSpeed < 0) absSpeed = -absSpeed;

  if (absSpeed > 0 && absSpeed < MIN_DRIVE_SPEED)
  {
    absSpeed = MIN_DRIVE_SPEED;
  }

  uint32_t pwm = (absSpeed * PWM_MAX_COUNT) / 100;

  if (speed > 0)
  {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm); // PB6
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);   // PB7
  }
  else if (speed < 0)
  {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  }
}

void DriveLeftMotor(int speed)
{
  if (LEFT_MOTOR_INVERT)
  {
    speed = -speed;
  }

  speed = clamp_int(speed, -100, 100);

  int absSpeed = speed;
  if (absSpeed < 0) absSpeed = -absSpeed;

  if (absSpeed > 0 && absSpeed < MIN_DRIVE_SPEED)
  {
    absSpeed = MIN_DRIVE_SPEED;
  }

  uint32_t pwm = (absSpeed * PWM_MAX_COUNT) / 100;

  if (speed > 0)
  {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm); // PB8
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);   // PB9
  }
  else if (speed < 0)
  {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
  }
}

void SetRobotSpeed(int leftSpeed, int rightSpeed)
{
  HAL_GPIO_WritePin(RIGHT_EN_GPIO_Port, RIGHT_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LEFT_EN_GPIO_Port, LEFT_EN_Pin, GPIO_PIN_SET);

  DriveLeftMotor(leftSpeed);
  DriveRightMotor(rightSpeed);
}

void SetRobotSpeedSmooth(int leftSpeed, int rightSpeed, int step)
{
  currentLeftSpeed = move_towards_int(currentLeftSpeed, leftSpeed, step);
  currentRightSpeed = move_towards_int(currentRightSpeed, rightSpeed, step);

  SetRobotSpeed(currentLeftSpeed, currentRightSpeed);
}

void MotorStop(void)
{
  currentLeftSpeed = 0;
  currentRightSpeed = 0;

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

  HAL_GPIO_WritePin(RIGHT_EN_GPIO_Port, RIGHT_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LEFT_EN_GPIO_Port, LEFT_EN_Pin, GPIO_PIN_RESET);
}

/* ============================================================
   SOFTWARE ENCODER UPDATE
   ============================================================ */

void UpdateEncoders(void)
{
  uint8_t rightA = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
  uint8_t rightB = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);

  uint8_t leftA = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
  uint8_t leftB = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);

  if (rightA != lastRightEncA)
  {
    if (rightA == rightB)
      rightEncoderCount++;
    else
      rightEncoderCount--;

    lastRightEncA = rightA;
  }

  if (leftA != lastLeftEncA)
  {
    if (leftA == leftB)
      leftEncoderCount++;
    else
      leftEncoderCount--;

    lastLeftEncA = leftA;
  }
}

/* ============================================================
   DEBUG PRINT
   ============================================================ */

void Print_Debug(int error, int strength, int leftSpeed, int rightSpeed, int lineDetected)
{
  char msg[1200];

  int diffPhysical[7];

  for (int i = 0; i < 7; i++)
  {
    uint8_t h = physicalOrder[i];

    int diff = (int)hallRaw[h] - (int)hallBaseline[h];

    if (diff < 0)
    {
      diff = -diff;
    }

    diffPhysical[i] = diff;
  }

  snprintf(msg, sizeof(msg),
           "L0/H6:%u(%d) L1/H5:%u(%d) L2/H2:%u(%d) L3/H1:%u(%d) L4/H0:%u(%d) L5/H4:%u(%d) L6/H3:%u(%d) | A:%d%d%d%d%d%d%d Cnt:%d | Det:%d W:%d M:%d SC:%d SCA:%d CJ:%d RIdx:%d Cmd:%d X:%d Stop:%d JS:%d RT:%d RDir:%d RS:%d SV:%d DS:%d Hold:%d Dir:%d Err:%d Str:%d | Left:%d Right:%d | CmdL:%d CmdR:%d | Lenc:%ld Renc:%ld\r\n",
           hallRaw[6], diffPhysical[0],
           hallRaw[5], diffPhysical[1],
           hallRaw[2], diffPhysical[2],
           hallRaw[1], diffPhysical[3],
           hallRaw[0], diffPhysical[4],
           hallRaw[4], diffPhysical[5],
           hallRaw[3], diffPhysical[6],

           activePhysical[0],
           activePhysical[1],
           activePhysical[2],
           activePhysical[3],
           activePhysical[4],
           activePhysical[5],
           activePhysical[6],
           activeCount,

           lineDetected,
           Detect_T_Junction(),
           wideMarkerActive,
           lastStopCrossDetected,
           stopCrossActive,
           Detect_Clear_90_Joint(),
           routeIndex,
           GetCurrentRouteCommand(),
           stopMarkerCount,
           stoppedAtTJunction,
           executingRouteJunctionStop,
           executingRouteTurn,
           routeTurnDirection,
           executingRouteStraight,
           serveSequenceActive ? serveState : 0,
           dockSequenceActive ? dockState : 0,
           chargerDockDetected,
           turnHoldActive,
           turnHoldDirection,
           error,
           strength,
           leftSpeed,
           rightSpeed,
           currentLeftSpeed,
           currentRightSpeed,
           (long)leftEncoderCount,
           (long)rightEncoderCount);

  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
}

/* USER CODE END 0 */

/* ============================================================
   MAIN
   ============================================================ */

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start(&hadc1);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  MotorStop();

  lastRightEncA = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
  lastLeftEncA  = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);

  char startMsg[] =
      "\r\nLine follower PID started - JGB37 table1 serve-return-dock version\r\n"
      "Right motor: PB6/PB7, encoder PB5/PB15\r\n"
      "Left motor : PB8/PB9, encoder PA8/PA12\r\n"
      "Waiting for ESP32 route command on USART1 PA9/PA10.\r\n"
      "After receiving route, robot moves forward briefly; start markers are ignored.\r\n"
      "Normal 30-degree bends are PID only; T/clear-90 joints consume commands.\r\n"
      "SERVE_STOP: wait, reverse little, 180 turn, then return.\r\n"
      "DOCK: T/cross/clear-90 marker starts dock sequence, then turn and reverse until same T is detected.\r\n";

  HAL_UART_Transmit(&huart3, (uint8_t*)startMsg, strlen(startMsg), 200);

  EspSendText("STM32_READY\n");

  /* USER CODE END 2 */

  while (1)
  {
    uint32_t loopStart = HAL_GetTick();

    CheckEspCommand();

    int error = 0;
    int strength = 0;
    int leftSpeed = 0;
    int rightSpeed = 0;

    UpdateEncoders();
    Read_All_Hall_Sensors();

    int lineDetected = Calculate_Line_Error(&error, &strength);
    int tDetected = Detect_T_Junction();
    int stopCrossDetected = Detect_Stop_Cross_Marker();
    int clear90Detected = Detect_Clear_90_Joint();
    uint8_t wideMarkerEvent = UpdateWideMarkerDetection(tDetected);
    uint8_t stopCrossEvent = UpdateStopCrossMarkerDetection(stopCrossDetected);
    uint8_t routeStartMarkerIgnoreActive = (!routeFinished && !routeError &&
                                            HAL_GetTick() < routeStartIgnoreUntil);

    UpdateTimedRouteEvents();
    if (routePaused)
    {
      MotorStop();

      if (HAL_GetTick() - lastDebugPrint >= 200)
      {
        Print_Debug(error, strength, 0, 0, lineDetected);
        lastDebugPrint = HAL_GetTick();
      }

      while ((HAL_GetTick() - loopStart) < 20)
      {
        UpdateEncoders();
        CheckEspCommand();
      }

      continue;
    }


    /* ========================================================
       ROUTE COMMAND EVENT HANDLING

       Logic is generic:
       - LEFT/RIGHT/STRAIGHT commands are consumed only at a T/wide/clear-90 decision point.
       - SERVE_STOP stops at the first table T/cross marker, waits, reverses, and turns.
       - DOCK stops at the dock T/cross marker, turns, and finishes.
       - Normal 30-degree segmented bends do not consume commands; PID handles them.
       ======================================================== */

    if (!stoppedAtTJunction && !routeFinished && !routeError && !routeStartMarkerIgnoreActive)
    {
      RouteCommand cmd = GetCurrentRouteCommand();
      uint8_t strongJunctionCandidate = IsStrongRouteJunctionCandidate(tDetected, stopCrossDetected, clear90Detected);

      /*
        LEFT / RIGHT / STRAIGHT are consumed at a strong wide/T/cross pattern.

        Important fix:
        The log showed W:1 and SC:1 while RIdx stayed 0. That meant the first
        LEFT command was never consumed, so the final stop marker was still
        interpreted while Cmd was LEFT instead of STOP.

        Therefore route-junction commands no longer depend only on a single
        wideMarkerEvent pulse. A strong direct candidate is enough after this
        small confirmation counter.
      */
      if (IsJunctionRouteCommand(cmd) &&
          !executingRouteJunctionStop && !executingRouteTurn && !executingRouteStraight &&
          !serveSequenceActive && !dockSequenceActive)
      {
        if (wideMarkerEvent || strongJunctionCandidate)
        {
          if (routeJunctionDirectCounter < 255)
          {
            routeJunctionDirectCounter++;
          }
        }
        else
        {
          routeJunctionDirectCounter = 0;
        }

        if (wideMarkerEvent || routeJunctionDirectCounter >= ROUTE_JUNCTION_DIRECT_CONFIRM_COUNT)
        {
          routeJunctionDirectCounter = 0;

          if (cmd == ROUTE_CMD_LEFT)
          {
            if (routeIndex < routeCommandCount) routeIndex++;
            ReportRouteJunctionEvent();
            StartRouteJunctionStop(-1);
          }
          else if (cmd == ROUTE_CMD_RIGHT)
          {
            if (routeIndex < routeCommandCount) routeIndex++;
            ReportRouteJunctionEvent();
            StartRouteJunctionStop(1);
          }
          else if (cmd == ROUTE_CMD_STRAIGHT)
          {
            if (routeIndex < routeCommandCount) routeIndex++;
            ReportRouteJunctionEvent();
            StartRouteStraightPass();
          }
        }
      }
      else
      {
        routeJunctionDirectCounter = 0;
      }

      uint8_t confirmedStopMarker = 0;

      if (wideMarkerEvent || stopCrossEvent ||
          (tDetected && tJunctionCounter >= T_CONFIRM_COUNT) ||
          (stopCrossDetected && stopCrossConfirmCounter >= STOP_CROSS_CONFIRM_COUNT))
      {
        confirmedStopMarker = 1;
      }

      if (cmd == ROUTE_CMD_SERVE_STOP && confirmedStopMarker &&
          !executingRouteJunctionStop && !executingRouteTurn && !executingRouteStraight &&
          !serveSequenceActive && !dockSequenceActive)
      {
        if (routeIndex < routeCommandCount) routeIndex++;
        stopMarkerCount = 1;
        ReportRouteTableEvent();
        StartServeStopSequence();
      }

      /*
        Dock can be seen differently depending on approach angle.
        In the latest log the robot reached the dock with RIdx:3 Cmd:4,
        but the dock marker appeared only as CJ:1 / clear-90 for a short time,
        not as a fully confirmed W/SC marker. Because DOCK ignored CJ, normal
        turn-hold took over and the robot pivoted around one tire.

        So DOCK accepts a clear 90 / branch-joint candidate directly.
        This is only for DOCK; SERVE_STOP still uses the stricter T/cross marker.
      */
      uint8_t confirmedDockMarker = confirmedStopMarker;
      if (cmd == ROUTE_CMD_DOCK && clear90Detected)
      {
        confirmedDockMarker = 1;
      }

      if (cmd == ROUTE_CMD_DOCK && confirmedDockMarker &&
          !executingRouteJunctionStop && !executingRouteTurn && !executingRouteStraight &&
          !serveSequenceActive && !dockSequenceActive)
      {
        stopMarkerCount = 1;
        EspSendEvent("DOCK_MARKER");
        StartDockStopTurnSequence();
      }
    }

    /*
      As soon as a T/wide marker is seen and the next command is a junction
      command, hold the robot stopped while the marker is being confirmed.
      This avoids the earlier problem where a tilted robot entered the right
      branch before the LEFT route command was applied.
    */
    uint8_t routeJunctionCandidate = 0;
    uint8_t routeStopCandidate = 0;

    if (!stoppedAtTJunction && !routeFinished && !routeError && !routeStartMarkerIgnoreActive &&
        !executingRouteJunctionStop && !executingRouteTurn && !executingRouteStraight &&
        !serveSequenceActive && !dockSequenceActive &&
        IsJunctionRouteCommand(GetCurrentRouteCommand()) &&
        (tDetected || stopCrossDetected || clear90Detected || tJunctionCounter > 0 || routeJunctionDirectCounter > 0))
    {
      routeJunctionCandidate = 1;
    }

    /*
      When the next command is STOP and the strict cross pattern is being seen,
      hold the motors stopped while the cross is being confirmed. Without this,
      normal turn-hold can rotate away from the stop mark before X increments.
    */
    if (!stoppedAtTJunction && !routeFinished && !routeError && !routeStartMarkerIgnoreActive &&
        !executingRouteJunctionStop && !executingRouteTurn && !executingRouteStraight &&
        (GetCurrentRouteCommand() == ROUTE_CMD_SERVE_STOP || GetCurrentRouteCommand() == ROUTE_CMD_DOCK) &&
        (tDetected || stopCrossDetected || tJunctionCounter > 0 || stopCrossConfirmCounter > 0 ||
         (GetCurrentRouteCommand() == ROUTE_CMD_DOCK && clear90Detected)))
    {
      routeStopCandidate = 1;
    }

    if (stoppedAtTJunction || routeFinished || routeError)
    {
      leftSpeed = 0;
      rightSpeed = 0;
      integral = 0;
      executingRouteJunctionStop = 0;
      executingRouteTurn = 0;
      executingRouteStraight = 0;
      executingRouteStartExit = 0;
      serveSequenceActive = 0;
      dockSequenceActive = 0;
      ClearTurnHoldState();
      MotorStop();
    }
    else if (serveSequenceActive)
    {
      uint32_t now = HAL_GetTick();
      uint32_t elapsed = now - serveStateStartTime;

      executingRouteJunctionStop = 0;
      executingRouteTurn = 0;
      executingRouteStraight = 0;
      ClearTurnHoldState();

      if (serveState == SERVE_STATE_WAIT)
      {
        leftSpeed = 0;
        rightSpeed = 0;
        MotorStop();

        if (serveFoodTakenReleased || elapsed >= SERVE_WAIT_MS)
        {
          if (!serveFoodTakenReleased)
          {
            EspSendEvent("SERVE_TIMEOUT");
          }

          serveFoodTakenReleased = 0;
          serveState = SERVE_STATE_REVERSE;
          serveStateStartTime = now;
          EspSendEvent("REVERSING");
        }
      }
      else if (serveState == SERVE_STATE_REVERSE)
      {
        if (elapsed < SERVE_REVERSE_MS)
        {
          leftSpeed = -SERVE_REVERSE_SPEED;
          rightSpeed = -SERVE_REVERSE_SPEED;
          SetRobotSpeedSmooth(leftSpeed, rightSpeed, RECOVERY_RAMP_STEP);
        }
        else
        {
          serveState = SERVE_STATE_UTURN;
          serveStateStartTime = now;
          serveUturnCenterConfirmCount = 0;
          ResetPidFiltersFromError(0);
          EspSendEvent("UTURN");
        }
      }
      else if (serveState == SERVE_STATE_UTURN)
      {
        if (SERVE_UTURN_DIRECTION > 0)
        {
          leftSpeed = SERVE_UTURN_SPEED;
          rightSpeed = -SERVE_UTURN_SPEED;
        }
        else
        {
          leftSpeed = -SERVE_UTURN_SPEED;
          rightSpeed = SERVE_UTURN_SPEED;
        }

        SetRobotSpeedSmooth(leftSpeed, rightSpeed, RECOVERY_RAMP_STEP);

        if (elapsed >= SERVE_UTURN_MIN_MS && IsUTurnLineLocked(lineDetected, tDetected, error))
        {
          if (serveUturnCenterConfirmCount < 255)
          {
            serveUturnCenterConfirmCount++;
          }

          if (serveUturnCenterConfirmCount >= UTURN_CENTER_CONFIRM_COUNT)
          {
            serveSequenceActive = 0;
            serveState = SERVE_STATE_NONE;
            serveUturnCenterConfirmCount = 0;
            ResetPidFiltersFromError(error);
            ClearTurnHoldState();
            markerIgnoreUntil = HAL_GetTick() + MARKER_IGNORE_AFTER_COMMAND_MS;
            EspSendEvent("RETURNING");
          }
        }
        else
        {
          serveUturnCenterConfirmCount = 0;
        }

        if (elapsed >= SERVE_UTURN_MAX_MS)
        {
          /* If line could not be locked, stop safely instead of spinning forever. */
          if (lineDetected)
          {
            serveSequenceActive = 0;
            serveState = SERVE_STATE_NONE;
            ResetPidFiltersFromError(error);
            ClearTurnHoldState();
            markerIgnoreUntil = HAL_GetTick() + MARKER_IGNORE_AFTER_COMMAND_MS;
          }
          else
          {
            routeError = 1;
            EspSendEvent("ERROR");
            MotorStop();
          }
        }
      }
    }
    else if (dockSequenceActive)
    {
      uint32_t now = HAL_GetTick();
      uint32_t elapsed = now - dockStateStartTime;

      executingRouteJunctionStop = 0;
      executingRouteTurn = 0;
      executingRouteStraight = 0;
      ClearTurnHoldState();

      if (dockState == DOCK_STATE_WAIT)
      {
        leftSpeed = 0;
        rightSpeed = 0;
        MotorStop();

        if (elapsed >= DOCK_STOP_WAIT_MS)
        {
          /* Same as the table serve turn: first reverse a little to give room. */
          dockState = DOCK_STATE_PRE_TURN_REVERSE;
          dockStateStartTime = now;
          dockUturnCenterConfirmCount = 0;
          dockReverseTConfirmCount = 0;
          ResetPidFiltersFromError(0);
        }
      }
      else if (dockState == DOCK_STATE_PRE_TURN_REVERSE)
      {
        if (elapsed < DOCK_PRE_TURN_REVERSE_MS)
        {
          leftSpeed = -DOCK_PRE_TURN_REVERSE_SPEED;
          rightSpeed = -DOCK_PRE_TURN_REVERSE_SPEED;
          SetRobotSpeedSmooth(leftSpeed, rightSpeed, RECOVERY_RAMP_STEP);
        }
        else
        {
          MotorStop();
          dockState = DOCK_STATE_UTURN;
          dockStateStartTime = now;
          dockUturnCenterConfirmCount = 0;
          ResetPidFiltersFromError(0);
          EspSendEvent("DOCK_UTURN");
        }
      }
      else if (dockState == DOCK_STATE_UTURN)
      {
        /* Use the same U-turn behavior/constants as the table serve point. */
        if (DOCK_UTURN_DIRECTION > 0)
        {
          leftSpeed = DOCK_UTURN_SPEED;
          rightSpeed = -DOCK_UTURN_SPEED;
        }
        else
        {
          leftSpeed = -DOCK_UTURN_SPEED;
          rightSpeed = DOCK_UTURN_SPEED;
        }

        SetRobotSpeedSmooth(leftSpeed, rightSpeed, RECOVERY_RAMP_STEP);

        if (elapsed >= DOCK_UTURN_MIN_MS && IsUTurnLineLocked(lineDetected, tDetected, error))
        {
          if (dockUturnCenterConfirmCount < 255)
          {
            dockUturnCenterConfirmCount++;
          }

          if (dockUturnCenterConfirmCount >= UTURN_CENTER_CONFIRM_COUNT)
          {
            /* Stop first, then reverse-to-T. Do not ramp directly from pivot to reverse. */
            MotorStop();
            dockState = DOCK_STATE_REVERSE_SETTLE;
            dockStateStartTime = HAL_GetTick();
            dockUturnCenterConfirmCount = 0;
          }
        }
        else
        {
          dockUturnCenterConfirmCount = 0;
        }

        if (elapsed >= DOCK_UTURN_MAX_MS)
        {
          /* If the U-turn cannot lock the line, stop safely instead of reversing blindly. */
          dockSequenceActive = 0;
          dockState = DOCK_STATE_NONE;
          routeFinished = 1;
          stoppedAtTJunction = 1;
          MotorStop();
        }
      }
      else if (dockState == DOCK_STATE_REVERSE_SETTLE)
      {
        leftSpeed = 0;
        rightSpeed = 0;
        MotorStop();

        if (elapsed >= DOCK_AFTER_UTURN_SETTLE_MS)
        {
          StartDockReverseToT();
        }
      }
      else if (dockState == DOCK_STATE_REVERSE_TO_T)
      {
        /* Dock parking reverse only:
           - Normally reverse slowly until the front Hall array sees the dock T.
           - If ESP32 detects charger/pogo voltage on IO41, it sends CHARGER_ON
             and this same state finishes docking even if the robot is slightly angled.
           - While reversing, use small Hall-error steering so the robot can correct
             the imperfect 180-degree turn angle before it reaches the dock. */
        leftSpeed = -DOCK_REVERSE_SPEED;
        rightSpeed = -DOCK_REVERSE_SPEED;

#if DOCK_REVERSE_STEER_ENABLE
        if (lineDetected)
        {
          int reverseCorrection = (int)(DOCK_REVERSE_STEER_KP * (float)error);
          reverseCorrection = clamp_int(reverseCorrection, -DOCK_REVERSE_STEER_MAX, DOCK_REVERSE_STEER_MAX);

#if DOCK_REVERSE_STEER_INVERT
          reverseCorrection = -reverseCorrection;
#endif

          leftSpeed  = -DOCK_REVERSE_SPEED + reverseCorrection;
          rightSpeed = -DOCK_REVERSE_SPEED - reverseCorrection;
        }
#endif

        SetRobotSpeedSmooth(leftSpeed, rightSpeed, RECOVERY_RAMP_STEP);

        if (elapsed >= DOCK_REVERSE_MIN_MS && (tDetected || chargerDockDetected))
        {
          if (dockReverseTConfirmCount < 255)
          {
            dockReverseTConfirmCount++;
          }

          if (dockReverseTConfirmCount >= DOCK_REVERSE_T_CONFIRM_COUNT)
          {
            FinishDockParking();
          }
        }
        else
        {
          dockReverseTConfirmCount = 0;
        }

        if (elapsed >= DOCK_REVERSE_MAX_MS)
        {
          /* Safety: if neither the dock T nor charger contact is found, stop instead of reversing forever. */
          dockSequenceActive = 0;
          dockState = DOCK_STATE_NONE;
          routeFinished = 1;
          stoppedAtTJunction = 1;
          chargerDockDetected = 0;
          MotorStop();
        }
      }
    }
    else if (executingRouteStartExit)
    {
      uint32_t elapsed = HAL_GetTick() - routeStartExitStartTime;

      /* Leave the dock/start marker before allowing the first route junction
         to be detected. This prevents the dock T from consuming the first
         RIGHT/LEFT/STRAIGHT command immediately after selecting a table. */
      ClearTurnHoldState();
      integral = 0;

      if (elapsed < ROUTE_START_EXIT_MS)
      {
        leftSpeed = baseSpeed;
        rightSpeed = baseSpeed;
        SetRobotSpeedSmooth(leftSpeed, rightSpeed, NORMAL_RAMP_STEP);
      }
      else
      {
        executingRouteStartExit = 0;
        ResetPidFiltersFromError(error);
      }
    }
    else if (executingRouteJunctionStop)
    {
      uint32_t elapsed = HAL_GetTick() - routeJunctionStopStartTime;

      leftSpeed = 0;
      rightSpeed = 0;
      integral = 0;
      ClearTurnHoldState();
      MotorStop();

      if (elapsed >= ROUTE_JUNCTION_STOP_MS)
      {
        StartRouteTurn(routeJunctionStopDirection);
      }
    }
    else if (routeJunctionCandidate)
    {
      /* Stop immediately at the T while waiting for/doing route confirmation. */
      leftSpeed = 0;
      rightSpeed = 0;
      integral = 0;
      ClearTurnHoldState();
      MotorStop();
    }
    else if (routeStopCandidate)
    {
      /* Stop immediately on a STOP-marker candidate while X is being confirmed. */
      leftSpeed = 0;
      rightSpeed = 0;
      integral = 0;
      ClearTurnHoldState();
      MotorStop();
    }
    else if (executingRouteTurn)
    {
      uint32_t now = HAL_GetTick();
      uint32_t elapsed = now - routeTurnStartTime;
      int absError = abs(error);

      if (lineDetected)
      {
        hasSeenLine = 1;
        lastLineSeenTime = now;
      }

      /*
        Route turn finish rule:
        Do NOT release just because any sensor sees the line. That made the
        robot stop/release before fully completing the T turn.

        Also do NOT wait for a perfect condition forever. Finish only when the
        new branch is reasonably centered and narrow enough to be a normal line,
        not the wide T/cross marker.
      */
      uint8_t routeBranchLocked = 0;

      if (elapsed >= ROUTE_TURN_MIN_MS && lineDetected && !tDetected &&
          activeCount <= ROUTE_BRANCH_LOCK_MAX_ACTIVE)
      {
        if (activePhysical[CENTER_INDEX] && absError <= ROUTE_BRANCH_LOCK_ERROR)
        {
          routeBranchLocked = 1;
        }
        else if (absError <= ROUTE_BRANCH_LOCK_ERROR && activeCount >= 1)
        {
          routeBranchLocked = 1;
        }
      }

      if (routeBranchLocked)
      {
        if (routeTurnCenterConfirmCount < 255)
        {
          routeTurnCenterConfirmCount++;
        }

        if (routeTurnCenterConfirmCount >= ROUTE_TURN_CENTER_CONFIRM_COUNT)
        {
          FinishRouteTurn(error);
        }
      }
      else
      {
        routeTurnCenterConfirmCount = 0;
      }

      if (elapsed >= ROUTE_TURN_MAX_MS)
      {
        /* Safety timeout: stop instead of spinning a large circle. */
        routeError = 1;
        EspSendEvent("ERROR");
        stoppedAtTJunction = 1;
        executingRouteTurn = 0;
        leftSpeed = 0;
        rightSpeed = 0;
        MotorStop();
      }

      if (executingRouteTurn)
      {
        if (routeTurnDirection > 0)
        {
          /*
            Route command RIGHT.
            Use the same gentle recovery style as the base code:
            one wheel moves forward, the other is stopped. No reverse pivot.
          */
          leftSpeed  = 0;
          rightSpeed = ROUTE_TURN_SPEED;
          lastError = TURN_ERROR;
          turnHoldActive = 1;
          turnHoldDirection = 1;
          turnHoldStartTime = HAL_GetTick();
        }
        else
        {
          /* Route command LEFT, using base-code left recovery style. */
          leftSpeed  = ROUTE_TURN_SPEED;
          rightSpeed = 0;
          lastError = -TURN_ERROR;
          turnHoldActive = 1;
          turnHoldDirection = -1;
          turnHoldStartTime = HAL_GetTick();
        }

        SetRobotSpeedSmooth(leftSpeed, rightSpeed, ROUTE_TURN_RAMP_STEP);
      }
    }
    else if (executingRouteStraight)
    {
      uint32_t elapsed = HAL_GetTick() - routeStraightStartTime;

      if (elapsed < ROUTE_STRAIGHT_PASS_MS)
      {
        leftSpeed = baseSpeed;
        rightSpeed = baseSpeed;
        SetRobotSpeedSmooth(leftSpeed, rightSpeed, NORMAL_RAMP_STEP);
      }
      else
      {
        executingRouteStraight = 0;
      }
    }
    else
    {
      if (lineDetected)
      {
        uint8_t firstLineAfterStop = (hasSeenLine == 0);

        hasSeenLine = 1;
        lastLineSeenTime = HAL_GetTick();

        /*
          After the robot has stopped outside the line and is placed back,
          do not let the smoothing filters start from zero. Start them from
          the current error so PID reacts immediately but still ramps smoothly.
        */
        if (firstLineAfterStop || !pidInitialized)
        {
          filteredError = (float)error;
          lastPidError = (float)error;
          filteredCorrection = 0.0f;
          pidInitialized = 1;
        }

        integral += error;

        if (integral > 3000) integral = 3000;
        if (integral < -3000) integral = -3000;

        /*
          Smooth PID input and output.
          This reduces sudden discrete left/right corrections that can cause skidding.
          Hard-turn detection still uses the raw error below.
        */
        filteredError = ((1.0f - ERROR_FILTER_ALPHA) * filteredError) +
                        (ERROR_FILTER_ALPHA * (float)error);

        float derivative = filteredError - lastPidError;
        float correctionRaw = (Kp * filteredError) + (Ki * integral) + (Kd * derivative);

        filteredCorrection = ((1.0f - CORRECTION_FILTER_ALPHA) * filteredCorrection) +
                             (CORRECTION_FILTER_ALPHA * correctionRaw);

        int absError = abs(error);

        /*
          Turn-hold state for natural bends/turns only.
          Route junction turns are handled above by route commands.
        */
        if ((activePhysical[RIGHT_EDGE_INDEX] && error > 0) || (error >= TURN_ERROR))
        {
          turnHoldActive = 1;
          turnHoldDirection = 1;
          turnHoldStartTime = HAL_GetTick();
          turnCenterConfirmCount = 0;
        }
        else if ((activePhysical[LEFT_EDGE_INDEX] && error < 0) || (error <= -TURN_ERROR))
        {
          turnHoldActive = 1;
          turnHoldDirection = -1;
          turnHoldStartTime = HAL_GetTick();
          turnCenterConfirmCount = 0;
        }

        if (turnHoldActive && activePhysical[CENTER_INDEX] && absError <= TURN_EXIT_ERROR)
        {
          if (turnCenterConfirmCount < 255)
          {
            turnCenterConfirmCount++;
          }

          if (turnCenterConfirmCount >= TURN_CENTER_CONFIRM_COUNT)
          {
            turnHoldActive = 0;
            turnHoldDirection = 0;
            turnCenterConfirmCount = 0;
          }
        }
        else if (turnHoldActive)
        {
          turnCenterConfirmCount = 0;
        }

        /*
          HARD TURN MODE is practically disabled by HARD_TURN_ERROR = 999.
          Normal mapped bends should be long/gentle and handled by PID only.
          Junction choice is still handled by route commands above.
        */
        if (absError >= HARD_TURN_ERROR)
        {
          if (error > 0)
          {
            /* Line is strongly on right */
            leftSpeed  = TURN_INNER_SPEED;
            rightSpeed = TURN_OUTER_SPEED;
          }
          else
          {
            /* Line is strongly on left */
            leftSpeed  = TURN_OUTER_SPEED;
            rightSpeed = TURN_INNER_SPEED;
          }
        }
        else
        {
          int dynamicBase = baseSpeed;

          /* Slow down during medium errors so the robot gets time to turn. */
          if (absError >= TURN_ERROR)
          {
            dynamicBase = baseSpeed - 4;
          }

          leftSpeed  = dynamicBase - (int)filteredCorrection;
          rightSpeed = dynamicBase + (int)filteredCorrection;

          leftSpeed  = clamp_int(leftSpeed, 0, maxSpeed);
          rightSpeed = clamp_int(rightSpeed, 0, maxSpeed);
        }

        int rampStep = NORMAL_RAMP_STEP;
        if (absError >= TURN_ERROR)
        {
          rampStep = TURN_RAMP_STEP;
        }

        SetRobotSpeedSmooth(leftSpeed, rightSpeed, rampStep);

        lastPidError = filteredError;
        lastError = error;
      }
      else
      {
        integral = 0;

        /*
          TURN RECOVERY:
          Normal line loss uses LINE_LOST_HOLD_MS.
          If a turn was already detected by edge sensor / large error,
          allow longer recovery until center sensor catches the line again
          or TURN_HOLD_MS expires.
        */
        uint8_t allowRecovery = 0;
        int8_t recoveryDirection = 0;

        if (turnHoldActive && ((HAL_GetTick() - turnHoldStartTime) < TURN_HOLD_MS))
        {
          allowRecovery = 1;
          recoveryDirection = turnHoldDirection;
        }
        else if (hasSeenLine && (abs((int)lastError) >= TURN_REENTRY_ERROR) &&
                 ((HAL_GetTick() - lastLineSeenTime) < TURN_HOLD_MS))
        {
          allowRecovery = 1;

          if (lastError > 0)
            recoveryDirection = 1;
          else if (lastError < 0)
            recoveryDirection = -1;
          else
            recoveryDirection = 0;

          turnHoldActive = 1;
          turnHoldDirection = recoveryDirection;
          turnHoldStartTime = lastLineSeenTime;
          turnCenterConfirmCount = 0;
        }
        else if (hasSeenLine && ((HAL_GetTick() - lastLineSeenTime) < LINE_LOST_HOLD_MS))
        {
          allowRecovery = 1;

          if (lastError > 0)
            recoveryDirection = 1;
          else if (lastError < 0)
            recoveryDirection = -1;
          else
            recoveryDirection = 0;
        }

        if (allowRecovery)
        {
          if (recoveryDirection > 0)
          {
            /* Last seen / turn direction is right */
            leftSpeed  = 0;
            rightSpeed = RECOVERY_SPEED;
          }
          else if (recoveryDirection < 0)
          {
            /* Last seen / turn direction is left */
            leftSpeed  = RECOVERY_SPEED;
            rightSpeed = 0;
          }
          else
          {
            leftSpeed = baseSpeed / 2;
            rightSpeed = baseSpeed / 2;
          }

          SetRobotSpeedSmooth(leftSpeed, rightSpeed, RECOVERY_RAMP_STEP);
        }
        else
        {
          turnHoldActive = 0;
          turnHoldDirection = 0;

          leftSpeed = 0;
          rightSpeed = 0;
          MotorStop();
          ResetFollowStateAfterStop();
        }
      }
    }

    if ((HAL_GetTick() - lastDebugPrint) >= 200)
    {
      Print_Debug(error, strength, leftSpeed, rightSpeed, lineDetected);
      lastDebugPrint = HAL_GetTick();
    }

    while ((HAL_GetTick() - loopStart) < 20)
    {
      UpdateEncoders();
      CheckEspCommand();
    }
  }
}

/* ============================================================
   CLOCK CONFIG
   ============================================================ */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* ============================================================
   ADC1 INIT
   ============================================================ */

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  __HAL_RCC_ADC1_CLK_ENABLE();

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;

  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/* ============================================================
   USART1 INIT
   PA9  = TX to ESP32 RX IO18
   PA10 = RX from ESP32 TX IO17
   ============================================================ */

static void MX_USART1_UART_Init(void)
{
  __HAL_RCC_USART1_CLK_ENABLE();

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* ============================================================
   USART3 INIT
   PB10 = TX
   PB11 = RX
   ============================================================ */

static void MX_USART3_UART_Init(void)
{
  __HAL_RCC_USART3_CLK_ENABLE();

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* ============================================================
   TIM4 INIT
   PB6 = CH1
   PB7 = CH2
   PB8 = CH3
   PB9 = CH4

   PWM frequency:
   8 MHz / (0 + 1) / (399 + 1) = 20 kHz
   ============================================================ */

static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  __HAL_RCC_TIM4_CLK_ENABLE();

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = PWM_MAX_COUNT;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* ============================================================
   GPIO INIT
   ============================================================ */

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Hall sensors PA0-PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 |
                        GPIO_PIN_1 |
                        GPIO_PIN_2 |
                        GPIO_PIN_3 |
                        GPIO_PIN_4 |
                        GPIO_PIN_5 |
                        GPIO_PIN_6;

  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* TIM4 PWM pins PB6-PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6 |
                        GPIO_PIN_7 |
                        GPIO_PIN_8 |
                        GPIO_PIN_9;

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Motor enable pins PB13, PB14 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Right encoder inputs PB5, PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Left encoder inputs PA8, PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 PA9 TX to ESP32 IO18 RX */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 PA10 RX from ESP32 IO17 TX */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART3 PB10 TX */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART3 PB11 RX */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* ============================================================
   ERROR HANDLER
   ============================================================ */

void Error_Handler(void)
{
  __disable_irq();

  MotorStop();

  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
