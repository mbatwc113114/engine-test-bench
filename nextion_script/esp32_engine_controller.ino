/*
 * =====================================================================
 *  Engine Test Bench DAQ - ESP32-C6 Firmware
 * =====================================================================
 *  Board:   Waveshare ESP32-C6-DEV-KIT-N8 / ESP32-C6-WROOM-1
 *  Role:    Transparent Nextion <-> PC bridge, plus servo actuator.
 *
 *  ARCHITECTURE (do not violate):
 *    - Python (PC) is the MASTER controller.
 *    - Nextion throtleSlider is the OPERATOR throttle input.
 *    - Python reads the slider value via the ESP32 bridge.
 *    - ESP32 NEVER polls "get throtleSlider.val" on its own.
 *    - ESP32 NEVER calculates engine physics.
 *    - Python calculates servo angle and sends "servo=XX" + FF FF FF.
 *    - ESP32 intercepts servo=XX commands and drives GPIO10 PWM.
 *    - All other commands from Python are forwarded to the Nextion
 *      display unchanged. All Nextion responses are forwarded
 *      unchanged to Python over USB Serial.
 *
 *  Pins:
 *    Nextion RX <- ESP32 GPIO16 (TX)
 *    Nextion TX -> ESP32 GPIO17 (RX)
 *    Servo signal -> ESP32 GPIO10 (external 5V supply, common GND)
 *
 *  Arduino-ESP32 core: 3.x  (uses ledcAttach()/ledcWrite(), NOT the
 *  obsolete ledcSetup()/ledcAttachPin() APIs).
 * =====================================================================
 */

#include <Arduino.h>

// ---------------------------------------------------------------------
// Pin / UART configuration
// ---------------------------------------------------------------------
#define NEXTION_RX_PIN   17   // ESP32 RX  <- Nextion TX
#define NEXTION_TX_PIN   16   // ESP32 TX  -> Nextion RX
#define NEXTION_BAUD     115200
#define PC_BAUD          115200

#define SERVO_PIN        10   // GPIO10 - DO NOT CHANGE (physical header position 10)
#define SERVO_FREQ       50   // 50 Hz
#define SERVO_RES        13   // 13-bit resolution
#define SERVO_MIN_US     500
#define SERVO_CENTER_US  1500
#define SERVO_MAX_US     2500
#define SERVO_PERIOD_US  20000UL
#define SERVO_INITIAL_ANGLE 90

HardwareSerial NextionSerial(1);   // UART1 for the Nextion display

// ---------------------------------------------------------------------
// Servo state
// ---------------------------------------------------------------------
int requestedServoAngle = SERVO_INITIAL_ANGLE;
int currentServoAngle   = -1;      // force first write to occur

// ---------------------------------------------------------------------
// PC command receive buffer (USB Serial)
// ---------------------------------------------------------------------
#define PC_BUF_MAX 128
String pcBuffer = "";
int ffCount = 0;                   // persistent FF-terminator counter

// =====================================================================
//  SERVO FUNCTIONS
// =====================================================================

void writeServoUs(uint32_t pulse_us) {
  if (pulse_us < SERVO_MIN_US) pulse_us = SERVO_MIN_US;
  if (pulse_us > SERVO_MAX_US) pulse_us = SERVO_MAX_US;

  uint32_t maxDuty = (1UL << SERVO_RES) - 1UL;
  uint32_t duty = (pulse_us * maxDuty) / SERVO_PERIOD_US;

  ledcWrite(SERVO_PIN, duty);
}

void writeServoAngle(int angle) {
  if (angle < 0)   angle = 0;
  if (angle > 180) angle = 180;

  uint32_t pulse_us = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  writeServoUs(pulse_us);
}

// Called every loop(). Only writes PWM when the requested angle changes.
void updateServo() {
  if (requestedServoAngle != currentServoAngle) {
    writeServoAngle(requestedServoAngle);
    currentServoAngle = requestedServoAngle;
    Serial.print("[SERVO UPDATED] ");
    Serial.print(currentServoAngle);
    Serial.println(" deg");
  }
}

// =====================================================================
//  NEXTION FORWARDING
// =====================================================================

void sendNextionCommand(const String &command) {
  NextionSerial.print(command);
  NextionSerial.write(0xFF);
  NextionSerial.write(0xFF);
  NextionSerial.write(0xFF);
}

// Forward every raw byte coming from the Nextion display straight to
// the PC over USB Serial, unchanged. This carries 0x71 numeric
// responses (e.g. throttle slider reads) and any other Nextion replies.
void readNextion() {
  while (NextionSerial.available()) {
    uint8_t b = NextionSerial.read();
    Serial.write(b);
  }
}

// =====================================================================
//  PC COMMAND PARSING
// =====================================================================

// Handles one complete command (terminator already stripped).
void processPCCommand(String command) {
  command.trim();
  if (command.length() == 0) return;

  int servoIdx = command.indexOf("servo=");
  if (servoIdx >= 0) {
    // Extract digits following "servo="
    int start = servoIdx + 6; // length of "servo="
    int i = start;
    String digits = "";
    while (i < (int)command.length() &&
           (isDigit(command.charAt(i)) || command.charAt(i) == '-')) {
      digits += command.charAt(i);
      i++;
    }

    Serial.print("[SERVO COMMAND] servo=");
    Serial.println(digits);

    if (digits.length() > 0) {
      int angle = digits.toInt();
      if (angle < 0)   angle = 0;
      if (angle > 180) angle = 180;
      requestedServoAngle = angle;
    }
    // Servo commands are intercepted and NEVER forwarded to Nextion.
    return;
  }

  // Not a servo command -> forward unchanged to the Nextion display.
  Serial.print("[NEXTION TX] ");
  Serial.println(command);
  sendNextionCommand(command);
}

// Reads bytes from USB Serial (PC), detects FF FF FF termination
// (also accepts '\n' for manual Serial Monitor testing), and passes
// complete commands to processPCCommand(). Non-blocking.
void readPC() {
  while (Serial.available()) {
    char c = Serial.read();

    if ((uint8_t)c == 0xFF) {
      ffCount++;
      if (ffCount >= 3) {
        // Complete FF FF FF terminated command
        Serial.print("[PC RX] ");
        Serial.println(pcBuffer);
        processPCCommand(pcBuffer);
        pcBuffer = "";
        ffCount = 0;
      }
      continue;
    }

    // Any non-FF byte resets the FF counter
    ffCount = 0;

    if (c == '\n' || c == '\r') {
      // Support newline termination for manual testing
      if (pcBuffer.length() > 0) {
        Serial.print("[PC RX] ");
        Serial.println(pcBuffer);
        processPCCommand(pcBuffer);
        pcBuffer = "";
      }
      continue;
    }

    if (pcBuffer.length() < PC_BUF_MAX) {
      pcBuffer += c;
    } else {
      // Safety: prevent unbounded buffer growth
      pcBuffer = "";
    }
  }
}

// =====================================================================
//  SETUP / LOOP
// =====================================================================

void setup() {
  Serial.begin(PC_BAUD);
  delay(200);

  NextionSerial.begin(NEXTION_BAUD, SERIAL_8N1, NEXTION_RX_PIN, NEXTION_TX_PIN);

  // LEDC PWM setup (Arduino-ESP32 3.x API)
  bool attachOk = ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_RES);

  Serial.println();
  Serial.println("=====================================");
  Serial.print("LEDC ATTACH = ");
  Serial.println(attachOk ? "SUCCESS" : "FAILED");
  Serial.print("Actual PWM frequency = ");
  Serial.println(SERVO_FREQ);

  // Initialize servo to center/initial angle
  requestedServoAngle = SERVO_INITIAL_ANGLE;
  writeServoAngle(requestedServoAngle);
  currentServoAngle = requestedServoAngle;

  Serial.println("SYSTEM READY");
  Serial.println("Waiting for servo=XX commands...");
  Serial.println("=====================================");
}

void loop() {
  readPC();
  updateServo();
  readNextion();
  delay(1); // small non-blocking yield
}
