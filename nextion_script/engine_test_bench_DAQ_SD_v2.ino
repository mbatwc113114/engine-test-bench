#include <SPI.h>
#include <SD.h>
#include <Adafruit_MAX31856.h>
#include "HX711.h"
#include <string.h>
#include <strings.h>
#include <Servo.h>
#include <ctype.h>

// ======================================================
// TEENSY 4.1 ENGINE TEST-BENCH DAQ
// 2x HX711 + 2x MAX31856 + ADXL335
//
// ADXL335: continuous 2 kHz acquisition
// USB: binary acceleration telemetry
// SD: binary logging controlled by LOGSTART / LOGSTOP
// Existing tare + MAX31856 fault features retained.
// Firmware revision: SD transfer v2.1 - fixed SD command linkage and transfer stack usage.
// ======================================================


// ======================================================
// GLOBAL CONFIGURATION - CHANGE SETTINGS HERE
// ======================================================

// ---------------------- USB / SERIAL -------------------
#define SERIAL_BAUD                 921600

// ---------------------- NEXTION DISPLAY -------------------
// Dedicated hardware UART. Existing USB DAQ (Serial) remains unchanged.
// Teensy 4.1 Serial6: RX=25, TX=24.
#define NEXTION_BAUD                921600
#define NEXTION_SERIAL              Serial6
#define NEXTION_RX_PIN              25
#define NEXTION_TX_PIN              24
#define NEXTION_UPDATE_PERIOD_MS    100
#define NEXTION_THROTTLE_READ_MS    100

// Exact component names from the supplied Python engine-model reference.
#define NEXTION_RPM                 "rpm"
#define NEXTION_THRUST              "thrust"
#define NEXTION_FUEL                "fule"
#define NEXTION_CHT                 "cht"
#define NEXTION_EGT                 "eht"
#define NEXTION_RPM_GAUGE           "rpmGauge"
#define NEXTION_THRUST_GAUGE        "thrustGauge"
#define NEXTION_FUEL_GAUGE          "fuleGauge"
#define NEXTION_CHT_GAUGE           "chtGauge"
#define NEXTION_EGT_GAUGE           "ehtGauge"
#define NEXTION_THROTTLE_SLIDER     "throtleSlider"
#define NEXTION_THROTTLE_TEXT       "throtle"

// ---------------------- THROTTLE SERVO ----------------------
// The single global throttlePercent controls the physical servo.
// Teensy 4.1 GPIO 26 is the servo signal output.
#define THROTTLE_SERVO_PIN          26
#define THROTTLE_SERVO_MIN_US       1000
#define THROTTLE_SERVO_MAX_US       2000
#define THROTTLE_SERVO_UPDATE_US    1000
#define NEXTION_THROTTLE_VALUE      "throtle"   // throtle.val
#define NEXTION_VIB_X               "vibX"
#define NEXTION_VIB_Y               "vibY"
#define NEXTION_VIB_Z               "vibZ"
#define NEXTION_RPM_WAVEFORM_ID     2
#define NEXTION_VIB_X_WAVEFORM_ID   3
#define NEXTION_VIB_Y_WAVEFORM_ID   4
#define NEXTION_VIB_Z_WAVEFORM_ID   5

// ---------------------- MAX31856 -----------------------
#define MAX31856_1_CS               10
#define MAX31856_2_CS               9

// Hardware SPI pins on Teensy 4.1
#define SPI_SCK_PIN                 13
#define SPI_MISO_PIN                12
#define SPI_MOSI_PIN                11

// India uses 50 Hz mains. Change to 60HZ for maximum MAX31856 conversion speed.
#define MAX_NOISE_FILTER            MAX31856_NOISE_FILTER_50HZ

// ---------------------- HX711 --------------------------
#define HX1_DOUT                    2       // 10 kg load cell
#define HX1_SCK                     3

#define HX2_DOUT                    4       // 1 kg load cell
#define HX2_SCK                     5

// Optional HX711 RATE control. Set to the Teensy pin physically wired to RATE.
// -1 = RATE is fixed by the HX711 module. HIGH = 80 SPS, LOW = 10 SPS.
#define HX1_RATE_PIN                -1
#define HX2_RATE_PIN                -1
#define HX711_FAST_RATE_ENABLED     true

// ---------------------- ADXL335 -----------------------
// User-requested mapping:
// ADXL XOUT -> Teensy 15 / A1
// ADXL YOUT -> Teensy 14 / A0
// ADXL ZOUT -> Teensy 16 / A2
#define ADXL_X_PIN                  15
#define ADXL_Y_PIN                  14
#define ADXL_Z_PIN                  16

#define ACC_SAMPLE_RATE_HZ          2000
#define ACC_SAMPLE_PERIOD_US        (1000000UL / ACC_SAMPLE_RATE_HZ)

// Teensy ADC
#define ADC_RESOLUTION_BITS         12
#define ADC_MAX_COUNTS              ((1UL << ADC_RESOLUTION_BITS) - 1)
#define ADC_REFERENCE_V             3.3f

// ---------------------- USB BINARY TELEMETRY ----------
#define TELEMETRY_RATE_HZ           100
#define TELEMETRY_PACKET_SAMPLES    (ACC_SAMPLE_RATE_HZ / TELEMETRY_RATE_HZ) // 20 samples @ 100 Hz
#define TELEMETRY_BYTES_PER_SAMPLE  6
#define TELEMETRY_ENABLED_DEFAULT   true

// Packet:
// AA 55 TYPE COUNT_L COUNT_H PAYLOAD CRC_L CRC_H
#define TELEMETRY_SYNC_1            0xAA
#define TELEMETRY_SYNC_2            0x55
#define TELEMETRY_TYPE_ACCEL        0x01
#define TELEMETRY_TYPE_STATUS       0x02
#define TELEMETRY_STATUS_BYTES      2   // uint16 throttle percent x10

// ---------------------- SD LOGGING --------------------
#define SD_LOG_ENABLED_DEFAULT      true
#define SD_FLUSH_RECORDS            256

// SD card USB transfer protocol
#define SD_TRANSFER_SYNC_1          0xD5
#define SD_TRANSFER_SYNC_2          0xA5
#define SD_TRANSFER_TYPE_DATA       0x01
#define SD_TRANSFER_CHUNK_SIZE      1024
#define SD_TRANSFER_MAX_FILENAME    63

// ---------------------- SENSOR / DISPLAY RATES -------
#define SENSOR_UPDATE_PERIOD_MS     10    // Poll fast; hardware sets true conversion rate
#define DISPLAY_PERIOD_MS           100

// ---------------------- ACCEL RAM BUFFER --------------
#define ACC_BUFFER_SAMPLES          4096

// ---------------------- ADXL DEFAULT CALIBRATION -----
#define ADXL_DEFAULT_OFFSET_V       1.65f
#define ADXL_DEFAULT_SENS_V_PER_G   0.330f
// ---------------------- ADXL335 NOISE FILTER ----------------
// Teensy-side 2nd-order Butterworth low-pass filter.
// Sampling remains at 2 kHz.
#define VIBRATION_FILTER_ENABLED    true
#define VIBRATION_FILTER_CUTOFF_HZ  500.0f

// false = telemetry + SD use filtered samples
// true  = telemetry + SD use raw ADC samples
#define VIBRATION_LOG_RAW           false

// ---------------------- HX711 CALIBRATION -------------
#define HX1_CALIBRATION_FACTOR      210.251343f   // 10 kg, grams
#define HX2_CALIBRATION_FACTOR      1036.294067f  // 1 kg, grams


// ======================================================
// SENSOR OBJECTS
// ======================================================

Adafruit_MAX31856 thermocouple1(MAX31856_1_CS);
Adafruit_MAX31856 thermocouple2(MAX31856_2_CS);

HX711 loadCell1;
HX711 loadCell2;


// ======================================================
// CALIBRATION
// ======================================================

float calibrationFactor1 = HX1_CALIBRATION_FACTOR;
float calibrationFactor2 = HX2_CALIBRATION_FACTOR;

float adxlXOffset = ADXL_DEFAULT_OFFSET_V;
float adxlYOffset = ADXL_DEFAULT_OFFSET_V;
float adxlZOffset = ADXL_DEFAULT_OFFSET_V;

float adxlXSensitivity = ADXL_DEFAULT_SENS_V_PER_G;
float adxlYSensitivity = ADXL_DEFAULT_SENS_V_PER_G;
float adxlZSensitivity = ADXL_DEFAULT_SENS_V_PER_G;


// ======================================================
// 2 kHz ACCELERATION BUFFER
// ======================================================

struct AccelSample
{
  uint16_t x;
  uint16_t y;
  uint16_t z;
};

// ======================================================
// TEENSY-SIDE VIBRATION FILTER
// ======================================================

struct BiquadLowPass
{
  float b0, b1, b2, a1, a2;
  float z1, z2;

  void reset(float initialValue)
  {
    z1 = initialValue * (1.0f - b0);
    z2 = initialValue * (b2 - a2);
  }

  float process(float input)
  {
    float output = b0 * input + z1;
    z1 = b1 * input - a1 * output + z2;
    z2 = b2 * input - a2 * output;
    return output;
  }
};

BiquadLowPass vibFilterX;
BiquadLowPass vibFilterY;
BiquadLowPass vibFilterZ;

void configureVibrationFilter();
void resetVibrationFilter(const AccelSample &sample);
AccelSample getFilteredAccelSample(const AccelSample &sample);

// Large buffer in Teensy RAM2.
DMAMEM AccelSample accBuffer[ACC_BUFFER_SAMPLES];

volatile uint32_t accWriteIndex = 0;
volatile uint32_t accReadIndex = 0;
volatile uint32_t accSampleCount = 0;
volatile uint32_t accOverflowCount = 0;
volatile bool accSamplingEnabled = true;

IntervalTimer accTimer;
uint32_t accSamplingStartUs = 0;


// ======================================================
// CACHED SLOW SENSOR VALUES
// ======================================================

float latestWeight1 = 0.0f;
float latestWeight2 = 0.0f;

long latestRaw1 = 0;
long latestRaw2 = 0;

bool latestHX1Ready = false;
bool latestHX2Ready = false;
bool hx1EverRead = false;
bool hx2EverRead = false;

float latestTemp1 = NAN;
float latestTemp2 = NAN;

float latestCJ1 = NAN;
float latestCJ2 = NAN;

uint8_t latestFault1 = 0;
uint8_t latestFault2 = 0;

uint32_t lastSensorUpdateMs = 0;
uint32_t lastDisplayMs = 0;

// ======================================================
// GLOBAL ENGINE / DISPLAY VALUES
// ======================================================
// One global throttle value is shared by Nextion and the laptop DAQ.
// Nextion is the physical input; the laptop can also issue THROTTLE <0..100>.
float throttlePercent = 0.0f;

Servo throttleServo;
bool throttleServoReady = false;
int lastThrottleServoValue = -1;

float latestVibrationX_g = 0.0f;
float latestVibrationY_g = 0.0f;
float latestVibrationZ_g = 0.0f;

// The supplied firmware has no RPM or fuel-level sensor input yet.
// Keep these explicit at zero rather than generating simulated values.
float latestRPM = 0.0f;
float latestFuelPercent = 0.0f;

uint32_t lastNextionDisplayMs = 0;
uint32_t lastNextionThrottleRequestMs = 0;
int lastNextionThrottleSent = -1;



// ======================================================
// THROTTLE SERVO OUTPUT
// ======================================================
// IMPORTANT: every source of throttle (Nextion or laptop DAQ)
// updates the same global throttlePercent. This function converts
// that global value to the servo PWM pulse on Teensy pin 26.
//
// 0%   -> 1000 us
// 50%  -> 1500 us
// 100% -> 2000 us
// ======================================================

void updateThrottleServo()
{
  if (!throttleServoReady)
    return;

  float percent = constrain(throttlePercent, 0.0f, 100.0f);
  int value = (int)lroundf(percent);

  // Avoid continuously rewriting the same pulse.
  if (value == lastThrottleServoValue)
    return;

  uint16_t pulseUs = (uint16_t)lroundf(
      THROTTLE_SERVO_MIN_US +
      (percent / 100.0f) *
      (float)(THROTTLE_SERVO_MAX_US - THROTTLE_SERVO_MIN_US));

  pulseUs = constrain(
      pulseUs,
      (uint16_t)THROTTLE_SERVO_MIN_US,
      (uint16_t)THROTTLE_SERVO_MAX_US);

  throttleServo.writeMicroseconds(pulseUs);
  lastThrottleServoValue = value;
}

void initThrottleServo()
{
  throttleServo.attach(
      THROTTLE_SERVO_PIN,
      THROTTLE_SERVO_MIN_US,
      THROTTLE_SERVO_MAX_US);

  throttleServoReady = true;
  lastThrottleServoValue = -1;

  // Start from the current global throttle.
  updateThrottleServo();

  Serial.print("THROTTLE SERVO: pin ");
  Serial.print(THROTTLE_SERVO_PIN);
  Serial.print(" initial throttle = ");
  Serial.print(throttlePercent, 1);
  Serial.println(" %");
}

// ======================================================
// NEXTION DISPLAY - DIRECT TEENSY UART
// ======================================================
// Exact variable/component names are taken from the supplied Python engine
// simulator. The ESP32 bridge is no longer needed for the display path.
//
// Nextion command terminator = FF FF FF
// "get throtleSlider.val" returns:
// 71 <4-byte little-endian uint32> FF FF FF
// ======================================================

static void nextionWriteTerminator()
{
  NEXTION_SERIAL.write((uint8_t)0xFF);
  NEXTION_SERIAL.write((uint8_t)0xFF);
  NEXTION_SERIAL.write((uint8_t)0xFF);
}

void nextionSendCommand(const char *command)
{
  if (!command)
    return;

  NEXTION_SERIAL.print(command);
  nextionWriteTerminator();
}

void nextionSetNumeric(const char *component, long value)
{
  if (!component)
    return;

  char command[64];
  snprintf(command, sizeof(command), "%s.val=%ld", component, value);
  nextionSendCommand(command);
}

void nextionSetText(const char *component, const char *text)
{
  if (!component || !text)
    return;

  char command[96];
  snprintf(command, sizeof(command), "%s.txt=\"%s\"", component, text);
  nextionSendCommand(command);
}

static long nextionScale(float value, float inMin, float inMax,
                         long outMin, long outMax)
{
  if (inMax <= inMin)
    return outMin;

  value = constrain(value, inMin, inMax);
  float ratio = (value - inMin) / (inMax - inMin);
  return (long)lroundf(outMin + ratio * (float)(outMax - outMin));
}

void nextionSetThrottleSlider(float percent)
{
  percent = constrain(percent, 0.0f, 100.0f);
  int value = (int)lroundf(percent);

  // Only write when the value actually changed, so the operator can touch
  // the slider without the MCU continuously fighting the touch input.
  if (lastNextionThrottleSent != value)
  {
    // Keep BOTH Nextion numeric controls synchronized:
    //   throtleSlider.val = global throttle
    //   throtle.val       = global throttle
    nextionSetNumeric(NEXTION_THROTTLE_SLIDER, value);
    nextionSetNumeric(NEXTION_THROTTLE_VALUE, value);

    // Preserve the existing text update feature as well.
    char text[16];
    snprintf(text, sizeof(text), "%d%%", value);
    nextionSetText(NEXTION_THROTTLE_TEXT, text);

    lastNextionThrottleSent = value;
  }
}

void nextionRequestThrottle()
{
  nextionSendCommand("get throtleSlider.val");
}

void nextionProcessRx()
{
  static uint8_t buffer[8];
  static uint8_t count = 0;

  while (NEXTION_SERIAL.available())
  {
    uint8_t c = (uint8_t)NEXTION_SERIAL.read();

    if (count == 0)
    {
      if (c == 0x71)
        buffer[count++] = c;
      continue;
    }

    buffer[count++] = c;

    if (count == 8)
    {
      if (buffer[0] == 0x71 &&
          buffer[5] == 0xFF &&
          buffer[6] == 0xFF &&
          buffer[7] == 0xFF)
      {
        uint32_t value =
            ((uint32_t)buffer[1]) |
            ((uint32_t)buffer[2] << 8) |
            ((uint32_t)buffer[3] << 16) |
            ((uint32_t)buffer[4] << 24);

        throttlePercent = constrain((float)value, 0.0f, 100.0f);

        // Nextion -> global throttle -> physical servo.
        updateThrottleServo();
      }

      count = 0;
    }
  }
}

// ======================================================
// COMPILE FIX: forward declarations
// ======================================================
extern bool telemetryEnabled;
uint16_t crc16_ccitt(const uint8_t *data, uint16_t length);

void sendThrottleStatusTelemetry()
{
  if (!telemetryEnabled)
    return;

  // Type 0x02. Payload = uint16 throttle percent x10.
  uint16_t throttle10 =
      (uint16_t)constrain(lroundf(throttlePercent * 10.0f), 0L, 1000L);

  // CRC must cover exactly the same bytes as the Python decoder:
  // TYPE + COUNT_L + COUNT_H + PAYLOAD.
  uint8_t crcData[5];
  crcData[0] = TELEMETRY_TYPE_STATUS;
  crcData[1] = TELEMETRY_STATUS_BYTES & 0xFF;
  crcData[2] = TELEMETRY_STATUS_BYTES >> 8;
  crcData[3] = throttle10 & 0xFF;
  crcData[4] = throttle10 >> 8;

  uint16_t crc = crc16_ccitt(crcData, sizeof(crcData));

  uint8_t header[5] = {
    TELEMETRY_SYNC_1,
    TELEMETRY_SYNC_2,
    TELEMETRY_TYPE_STATUS,
    TELEMETRY_STATUS_BYTES & 0xFF,
    TELEMETRY_STATUS_BYTES >> 8
  };

  uint8_t crcBytes[2] = {
    (uint8_t)(crc & 0xFF),
    (uint8_t)(crc >> 8)
  };

  Serial.write(header, sizeof(header));
  Serial.write(&crcData[3], TELEMETRY_STATUS_BYTES);
  Serial.write(crcBytes, sizeof(crcBytes));
}

void nextionUpdateDisplay()
{
  // Real sensors available in the current firmware:
  // load cell #1 -> thrust, MAX31856 #1/#2 -> EGT/CHT,
  // ADXL335 -> vibration. No simulated engine values are used.
  float thrustKg = max(0.0f, latestWeight1 / 1000.0f);
  float egt = isnan(latestTemp1) ? 0.0f : latestTemp1;
  float cht = isnan(latestTemp2) ? 0.0f : latestTemp2;

  // Exact numeric component names/scaling from the reference simulator.
  nextionSetNumeric(NEXTION_RPM, (long)lroundf(latestRPM));
  nextionSetNumeric(NEXTION_THRUST, (long)lroundf(thrustKg * 100.0f));
  nextionSetNumeric(NEXTION_FUEL, (long)lroundf(latestFuelPercent));
  nextionSetNumeric(NEXTION_CHT, (long)lroundf(cht));
  nextionSetNumeric(NEXTION_EGT, (long)lroundf(egt));

  // Exact gauge ranges from the reference simulator.
  nextionSetNumeric(NEXTION_RPM_GAUGE,
                    nextionScale(latestRPM, 0.0f, 8000.0f, 0, 270));
  nextionSetNumeric(NEXTION_THRUST_GAUGE,
                    nextionScale(thrustKg, 0.0f, 7.0f, 0, 245));
  nextionSetNumeric(NEXTION_FUEL_GAUGE,
                    nextionScale(latestFuelPercent, 0.0f, 100.0f, 0, 245));
  nextionSetNumeric(NEXTION_CHT_GAUGE,
                    nextionScale(cht, 0.0f, 250.0f, 0, 245));
  nextionSetNumeric(NEXTION_EGT_GAUGE,
                    nextionScale(egt, 0.0f, 800.0f, 0, 245));

  // Vibration values are in hundredths of g because Nextion .val is integer.
  nextionSetNumeric(NEXTION_VIB_X,
                    (long)lroundf(latestVibrationX_g * 100.0f));
  nextionSetNumeric(NEXTION_VIB_Y,
                    (long)lroundf(latestVibrationY_g * 100.0f));
  nextionSetNumeric(NEXTION_VIB_Z,
                    (long)lroundf(latestVibrationZ_g * 100.0f));

  // Global throttle synchronization.
  nextionSetThrottleSlider(throttlePercent);

  // Waveforms use the exact object IDs from the reference code.
  uint8_t rp = (uint8_t)nextionScale(latestRPM, 0.0f, 8000.0f, 0, 255);
  uint8_t vx = (uint8_t)nextionScale(latestVibrationX_g, -60.0f, 60.0f, 0, 255);
  uint8_t vy = (uint8_t)nextionScale(latestVibrationY_g, -60.0f, 60.0f, 0, 255);
  uint8_t vz = (uint8_t)nextionScale(latestVibrationZ_g, -60.0f, 60.0f, 0, 255);

  char command[48];
  snprintf(command, sizeof(command), "add %d,0,%u", NEXTION_RPM_WAVEFORM_ID, rp);
  nextionSendCommand(command);
  snprintf(command, sizeof(command), "add %d,0,%u", NEXTION_VIB_X_WAVEFORM_ID, vx);
  nextionSendCommand(command);
  snprintf(command, sizeof(command), "add %d,0,%u", NEXTION_VIB_Y_WAVEFORM_ID, vy);
  nextionSendCommand(command);
  snprintf(command, sizeof(command), "add %d,0,%u", NEXTION_VIB_Z_WAVEFORM_ID, vz);
  nextionSendCommand(command);

  // Laptop receives the same global throttle value.
  sendThrottleStatusTelemetry();
}

void nextionBegin()
{
  NEXTION_SERIAL.begin(NEXTION_BAUD);
  delay(50);

  while (NEXTION_SERIAL.available())
    NEXTION_SERIAL.read();

  nextionSetThrottleSlider(throttlePercent);

  Serial.println("NEXTION: Serial6 RX=25 TX=24 @ 115200");
}

void serviceNextion()
{
  nextionProcessRx();

  uint32_t now = millis();

  if ((uint32_t)(now - lastNextionThrottleRequestMs) >= NEXTION_THROTTLE_READ_MS)
  {
    lastNextionThrottleRequestMs = now;
    nextionRequestThrottle();
  }

  if ((uint32_t)(now - lastNextionDisplayMs) >= NEXTION_UPDATE_PERIOD_MS)
  {
    lastNextionDisplayMs = now;
    nextionUpdateDisplay();
  }
}


// ======================================================
// SERIAL COMMAND
// ======================================================

String serialCommand = "";


// ======================================================
// BINARY TELEMETRY
// ======================================================

bool telemetryEnabled = TELEMETRY_ENABLED_DEFAULT;
uint16_t telemetrySampleCount = 0;

uint8_t telemetryPayload[TELEMETRY_PACKET_SAMPLES * 6];
uint8_t telemetryCRCBuffer[3 + TELEMETRY_PACKET_SAMPLES * 6];


// ======================================================
// SD LOGGING
// ======================================================

struct __attribute__((packed)) LogRecord
{
  uint32_t sampleIndex;
  uint32_t timestampUs;

  uint16_t x;
  uint16_t y;
  uint16_t z;

  float weight1_g;
  float weight2_g;

  float temp1_C;
  float temp2_C;

  uint8_t fault1;
  uint8_t fault2;
};

static_assert(sizeof(LogRecord) == 32, "LogRecord must be 32 bytes");

File logFile;

bool loggingActive = false;
bool sdCardReady = false;

char activeLogFileName[80] = "";

uint32_t logRecordCount = 0;
uint32_t logLastFlushRecord = 0;
uint32_t logStartMillis = 0;


// ======================================================
// FORWARD DECLARATIONS
// ======================================================

void handleSerialCommand();

void printFault1(uint8_t fault);
void printFault2(uint8_t fault);

void startAccelerationSampling();
void stopAccelerationSampling();
void clearAccelerationBuffer();
void processAccelerationBuffer();

void sendAccelTelemetrySample(const AccelSample &sample);
void flushTelemetryPacket();
uint16_t crc16_ccitt(const uint8_t *data, uint16_t length);

void updateSlowSensors();
void displaySlowSensors();

void startLogging(const char *requestedName);
void stopLogging();
void logAccelerationSample(const AccelSample &sample,
                           uint32_t sampleIndex,
                           uint32_t timestampUs);
void writeLogHeader(const char *filename);
static bool ensureSDCard(bool verbose = true);
static void handleSDInitCommand();
static void handleSDListCommand();
static bool handleSDDownloadCommand(const char *requestedName);
static void sendSDDataFrame(uint32_t offset, const uint8_t *data, uint16_t length, uint16_t &fileCRC);
static bool waitForSDDownloadAck(uint32_t expectedOffset, uint32_t timeoutMs = 1000);

void performADXLCalibration();

void nextionBegin();
void nextionProcessRx();
void nextionRequestThrottle();
void nextionUpdateDisplay();
void nextionSendCommand(const char *command);
void nextionSetNumeric(const char *component, long value);
void nextionSetText(const char *component, const char *text);
void nextionSetThrottleSlider(float percent);
void sendThrottleStatusTelemetry();
void updateThrottleServo();
void initThrottleServo();
void serviceNextion();

float rawToVoltage(uint16_t raw);
float rawToG(uint16_t raw, float offset, float sensitivity);
bool waitForFloat(float &value, const char *prompt);


// ======================================================
// ADXL335 2 kHz ISR
// ======================================================
//
// DO NOT use Serial, SD, String, File, or floating point
// calculations inside this ISR.
// ======================================================

void accSampleISR()
{
  if (!accSamplingEnabled)
    return;

  uint32_t writeIndex = accWriteIndex;

  accBuffer[writeIndex].x = analogRead(ADXL_X_PIN);
  accBuffer[writeIndex].y = analogRead(ADXL_Y_PIN);
  accBuffer[writeIndex].z = analogRead(ADXL_Z_PIN);

  writeIndex++;

  if (writeIndex >= ACC_BUFFER_SAMPLES)
    writeIndex = 0;

  accWriteIndex = writeIndex;

  uint32_t count = accSampleCount;

  if (count < ACC_BUFFER_SAMPLES)
  {
    accSampleCount = count + 1;
  }
  else
  {
    uint32_t nextRead = accReadIndex + 1;

    if (nextRead >= ACC_BUFFER_SAMPLES)
      nextRead = 0;

    accReadIndex = nextRead;
    accOverflowCount++;
  }
}


// ======================================================
// ACCELERATION TIMER CONTROL
// ======================================================

void startAccelerationSampling()
{
  accSamplingEnabled = true;

  // Capture the sampling epoch. Individual samples are timestamped
  // from this fixed 2 kHz clock, not from delayed buffer processing.
  accSamplingStartUs = micros();

  AccelSample initialFilterSample;
  initialFilterSample.x = (uint16_t)(ADXL_DEFAULT_OFFSET_V *
                                     (float)ADC_MAX_COUNTS /
                                     ADC_REFERENCE_V);
  initialFilterSample.y = initialFilterSample.x;
  initialFilterSample.z = initialFilterSample.x;
  resetVibrationFilter(initialFilterSample);

  if (!accTimer.begin(accSampleISR, ACC_SAMPLE_PERIOD_US))
  {
    Serial.println("ERROR: ADXL335 IntervalTimer failed!");
    accSamplingEnabled = false;
    return;
  }

  Serial.print("ADXL335 sampling: ");
  Serial.print(ACC_SAMPLE_RATE_HZ);
  Serial.println(" Hz");
}


void stopAccelerationSampling()
{
  accSamplingEnabled = false;
  accTimer.end();
}


void clearAccelerationBuffer()
{
  noInterrupts();

  accWriteIndex = 0;
  accReadIndex = 0;
  accSampleCount = 0;
  accOverflowCount = 0;

  interrupts();
}


// ======================================================

// ======================================================
// VIBRATION FILTER IMPLEMENTATION
// ======================================================

void configureVibrationFilter()
{
  const float fs = (float)ACC_SAMPLE_RATE_HZ;
  const float fc = VIBRATION_FILTER_CUTOFF_HZ;
  const float PI_F = 3.14159265358979323846f;

  if (fc <= 0.0f || fc >= fs * 0.5f)
  {
    Serial.println("ERROR: Invalid vibration filter cutoff.");
    Serial.println("Filter coefficients set to bypass.");

    vibFilterX = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    vibFilterY = vibFilterX;
    vibFilterZ = vibFilterX;
    return;
  }

  const float omega = 2.0f * PI_F * fc / fs;
  const float sn = sinf(omega);
  const float cs = cosf(omega);

  const float Q = 0.7071067811865475f;
  const float alpha = sn / (2.0f * Q);
  const float a0 = 1.0f + alpha;

  const float b0 = ((1.0f - cs) * 0.5f) / a0;
  const float b1 = (1.0f - cs) / a0;
  const float b2 = b0;
  const float a1 = (-2.0f * cs) / a0;
  const float a2 = (1.0f - alpha) / a0;

  vibFilterX = {b0, b1, b2, a1, a2, 0.0f, 0.0f};
  vibFilterY = vibFilterX;
  vibFilterZ = vibFilterX;

  Serial.println();
  Serial.println("ADXL335 digital filter:");
  Serial.println("Type: 2nd-order Butterworth low-pass");
  Serial.print("Cutoff: ");
  Serial.print(fc, 1);
  Serial.println(" Hz");
  Serial.print("Filter: ");
  Serial.println(VIBRATION_FILTER_ENABLED ? "ON" : "OFF");
  Serial.print("Output: ");
  Serial.println(VIBRATION_LOG_RAW ? "RAW" : "FILTERED");
}

void resetVibrationFilter(const AccelSample &sample)
{
  vibFilterX.reset((float)sample.x);
  vibFilterY.reset((float)sample.y);
  vibFilterZ.reset((float)sample.z);
}

AccelSample getFilteredAccelSample(const AccelSample &sample)
{
  if (!VIBRATION_FILTER_ENABLED)
    return sample;

  AccelSample filtered;

  float x = vibFilterX.process((float)sample.x);
  float y = vibFilterY.process((float)sample.y);
  float z = vibFilterZ.process((float)sample.z);

  x = constrain(x, 0.0f, (float)ADC_MAX_COUNTS);
  y = constrain(y, 0.0f, (float)ADC_MAX_COUNTS);
  z = constrain(z, 0.0f, (float)ADC_MAX_COUNTS);

  filtered.x = (uint16_t)(x + 0.5f);
  filtered.y = (uint16_t)(y + 0.5f);
  filtered.z = (uint16_t)(z + 0.5f);

  return filtered;
}

// ADXL CONVERSION
// ======================================================

float rawToVoltage(uint16_t raw)
{
  return ((float)raw * ADC_REFERENCE_V) /
         (float)ADC_MAX_COUNTS;
}


float rawToG(uint16_t raw, float offset, float sensitivity)
{
  if (sensitivity == 0.0f)
    return 0.0f;

  return (rawToVoltage(raw) - offset) / sensitivity;
}


// ======================================================
// CALIBRATION INPUT
// ======================================================

bool waitForFloat(float &value, const char *prompt)
{
  Serial.println(prompt);

  String input = "";
  uint32_t start = millis();

  while (millis() - start < 60000UL)
  {
    while (Serial.available())
    {
      char c = Serial.read();

      if (c == '\n' || c == '\r')
      {
        input.trim();

        if (input.length() == 0)
          continue;

        value = input.toFloat();

        Serial.print("Received: ");
        Serial.println(value, 6);

        return true;
      }

      if (input.length() < 30)
        input += c;
    }
  }

  Serial.println("Calibration timeout.");
  return false;
}


// ======================================================
// ADXL335 CALIBRATION
// ======================================================

void performADXLCalibration()
{
  Serial.println();
  Serial.println("==============================================");
  Serial.println("ADXL335 CALIBRATION");
  Serial.println("==============================================");

  stopAccelerationSampling();

  float xPlus = 0.0f, xMinus = 0.0f;
  float yPlus = 0.0f, yMinus = 0.0f;
  float zPlus = 0.0f, zMinus = 0.0f;

  if (!waitForFloat(xPlus,
                    "X axis: place +1g and enter voltage:"))
    goto calibration_end;

  if (!waitForFloat(xMinus,
                    "X axis: place -1g and enter voltage:"))
    goto calibration_end;

  if (!waitForFloat(yPlus,
                    "Y axis: place +1g and enter voltage:"))
    goto calibration_end;

  if (!waitForFloat(yMinus,
                    "Y axis: place -1g and enter voltage:"))
    goto calibration_end;

  if (!waitForFloat(zPlus,
                    "Z axis: place +1g and enter voltage:"))
    goto calibration_end;

  if (!waitForFloat(zMinus,
                    "Z axis: place -1g and enter voltage:"))
    goto calibration_end;

  adxlXOffset = (xPlus + xMinus) * 0.5f;
  adxlYOffset = (yPlus + yMinus) * 0.5f;
  adxlZOffset = (zPlus + zMinus) * 0.5f;

  adxlXSensitivity = fabsf(xPlus - xMinus) * 0.5f;
  adxlYSensitivity = fabsf(yPlus - yMinus) * 0.5f;
  adxlZSensitivity = fabsf(zPlus - zMinus) * 0.5f;

  Serial.println();
  Serial.println("ADXL335 CALIBRATION COMPLETE");

  Serial.print("X offset = ");
  Serial.println(adxlXOffset, 6);
  Serial.print("X sensitivity = ");
  Serial.println(adxlXSensitivity, 6);

  Serial.print("Y offset = ");
  Serial.println(adxlYOffset, 6);
  Serial.print("Y sensitivity = ");
  Serial.println(adxlYSensitivity, 6);

  Serial.print("Z offset = ");
  Serial.println(adxlZOffset, 6);
  Serial.print("Z sensitivity = ");
  Serial.println(adxlZSensitivity, 6);

calibration_end:

  configureVibrationFilter();
  clearAccelerationBuffer();
  startAccelerationSampling();

  Serial.println("2 kHz acceleration sampling restarted.");
}


// ======================================================
// CRC16-CCITT
// ======================================================

uint16_t crc16_ccitt(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFF;

  for (uint16_t i = 0; i < length; i++)
  {
    crc ^= (uint16_t)data[i] << 8;

    for (uint8_t bit = 0; bit < 8; bit++)
    {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }

  return crc;
}


// ======================================================
// BINARY USB TELEMETRY
// ======================================================
//
// Every sample = 3 x uint16 ADC values = 6 bytes.
//
// At 2000 Hz and 20 samples/packet:
// 100 packets/s, 12,000 payload bytes/s plus packet overhead.
// ======================================================

void sendAccelTelemetrySample(const AccelSample &sample)
{
  if (!telemetryEnabled)
    return;

  uint16_t pos = telemetrySampleCount * 6;

  telemetryPayload[pos + 0] = sample.x & 0xFF;
  telemetryPayload[pos + 1] = sample.x >> 8;

  telemetryPayload[pos + 2] = sample.y & 0xFF;
  telemetryPayload[pos + 3] = sample.y >> 8;

  telemetryPayload[pos + 4] = sample.z & 0xFF;
  telemetryPayload[pos + 5] = sample.z >> 8;

  telemetrySampleCount++;

  if (telemetrySampleCount >= TELEMETRY_PACKET_SAMPLES)
    flushTelemetryPacket();
}


void flushTelemetryPacket()
{
  if (!telemetryEnabled || telemetrySampleCount == 0)
    return;

  uint16_t payloadBytes = telemetrySampleCount * 6;

  telemetryCRCBuffer[0] = TELEMETRY_TYPE_ACCEL;
  telemetryCRCBuffer[1] = telemetrySampleCount & 0xFF;
  telemetryCRCBuffer[2] = telemetrySampleCount >> 8;

  memcpy(&telemetryCRCBuffer[3],
         telemetryPayload,
         payloadBytes);

  uint16_t crc = crc16_ccitt(
      telemetryCRCBuffer,
      3 + payloadBytes);

  uint8_t header[5];

  header[0] = TELEMETRY_SYNC_1;
  header[1] = TELEMETRY_SYNC_2;
  header[2] = TELEMETRY_TYPE_ACCEL;
  header[3] = telemetrySampleCount & 0xFF;
  header[4] = telemetrySampleCount >> 8;

  uint8_t crcBytes[2];
  crcBytes[0] = crc & 0xFF;
  crcBytes[1] = crc >> 8;

  Serial.write(header, sizeof(header));
  Serial.write(telemetryPayload, payloadBytes);
  Serial.write(crcBytes, sizeof(crcBytes));

  telemetrySampleCount = 0;
}


// ======================================================
// SD LOG HEADER
// ======================================================

void writeLogHeader(const char *filename)
{
  uint8_t header[64];
  memset(header, 0, sizeof(header));

  const char magic[8] =
  {
    'E','N','G','D','A','Q','0','1'
  };

  memcpy(header, magic, 8);

  uint16_t version = 1;
  uint16_t recordSize = sizeof(LogRecord);
  uint32_t sampleRate = ACC_SAMPLE_RATE_HZ;

  memcpy(header + 8,  &version,    sizeof(version));
  memcpy(header + 10, &recordSize, sizeof(recordSize));
  memcpy(header + 12, &sampleRate, sizeof(sampleRate));

  strncpy((char *)(header + 16), filename, 47);

  logFile.write(header, sizeof(header));
}


// ======================================================
// START LOGGING
// ======================================================

void startLogging(const char *requestedName)
{
  if (!SD_LOG_ENABLED_DEFAULT)
  {
    Serial.println("SD logging disabled in configuration.");
    return;
  }

  if (loggingActive)
  {
    Serial.println("A log is already active. Use LOGSTOP first.");
    return;
  }

  if (!ensureSDCard(true))
  {
    Serial.println("ERROR: Cannot start logging because the SD card is not ready.");
    return;
  }

  char filename[80];
  memset(filename, 0, sizeof(filename));

  if (requestedName == nullptr)
  {
    Serial.println("ERROR: Invalid log filename.");
    return;
  }

  strncpy(filename, requestedName, sizeof(filename) - 1);
  filename[sizeof(filename) - 1] = '\0';

  // If no extension is given, use .bin.
  if (strchr(filename, '.') == nullptr)
  {
    size_t used = strlen(filename);
    if (used + 4 >= sizeof(filename))
    {
      Serial.println("ERROR: Log filename is too long.");
      return;
    }
    memcpy(filename + used, ".bin", 5);
  }

  logFile = SD.open(filename, FILE_WRITE);

  if (!logFile)
  {
    Serial.print("ERROR: Could not open log file: ");
    Serial.println(filename);
    return;
  }

  strncpy(activeLogFileName, filename, sizeof(activeLogFileName) - 1);
  activeLogFileName[sizeof(activeLogFileName) - 1] = '\0';

  logRecordCount = 0;
  logLastFlushRecord = 0;
  logStartMillis = millis();

  writeLogHeader(filename);

  loggingActive = true;

  Serial.println();
  Serial.println("==============================================");
  Serial.println("SD LOGGING STARTED");
  Serial.print("File: ");
  Serial.println(filename);
  Serial.print("Acceleration rate: ");
  Serial.print(ACC_SAMPLE_RATE_HZ);
  Serial.println(" Hz");
  Serial.print("Record size: ");
  Serial.print(sizeof(LogRecord));
  Serial.println(" bytes");
  Serial.println("==============================================");
}


// ======================================================
// STOP LOGGING
// ======================================================

void stopLogging()
{
  if (!loggingActive)
  {
    Serial.println("No active SD log.");
    return;
  }

  logFile.flush();
  logFile.close();

  loggingActive = false;

  float seconds =
      (millis() - logStartMillis) / 1000.0f;

  Serial.println();
  Serial.println("==============================================");
  Serial.println("SD LOGGING STOPPED");
  Serial.print("File: ");
  Serial.println(activeLogFileName);
  Serial.print("Records: ");
  Serial.println(logRecordCount);
  Serial.print("Duration: ");
  Serial.print(seconds, 2);
  Serial.println(" s");
  Serial.print("Buffer overflows: ");
  Serial.println(accOverflowCount);
  Serial.println("==============================================");
}


// ======================================================
// WRITE ONE 32-BYTE LOG RECORD
// ======================================================
//
// 32 bytes/sample at 2000 Hz = 64 kB/s.
// Approximately 230.4 MB/hour.
// ======================================================

void logAccelerationSample(const AccelSample &sample,
                           uint32_t sampleIndex,
                           uint32_t timestampUs)
{
  if (!loggingActive)
    return;

  LogRecord record;

  record.sampleIndex = sampleIndex;
  record.timestampUs = timestampUs;

  record.x = sample.x;
  record.y = sample.y;
  record.z = sample.z;

  record.weight1_g = latestWeight1;
  record.weight2_g = latestWeight2;

  record.temp1_C = latestTemp1;
  record.temp2_C = latestTemp2;

  record.fault1 = latestFault1;
  record.fault2 = latestFault2;

  logFile.write((const uint8_t *)&record,
                sizeof(record));

  logRecordCount++;

  if ((logRecordCount - logLastFlushRecord) >= SD_FLUSH_RECORDS)
  {
    logFile.flush();
    logLastFlushRecord = logRecordCount;
  }
}


// ======================================================
// PROCESS ACCELERATION BUFFER
// ======================================================

void processAccelerationBuffer()
{
  static uint32_t totalSampleIndex = 0;

  uint16_t processed = 0;

  // Limit work per loop so the other sensors/commands
  // continue to get CPU time.
  while (processed < 256)
  {
    uint32_t readIndex = 0;
    bool haveSample = false;

    noInterrupts();

    if (accSampleCount > 0)
    {
      haveSample = true;

      readIndex = accReadIndex;

      accReadIndex++;

      if (accReadIndex >= ACC_BUFFER_SAMPLES)
        accReadIndex = 0;

      accSampleCount--;
    }

    interrupts();

    if (!haveSample)
      break;


    AccelSample rawSample = accBuffer[readIndex];

    // Filter outside the 2 kHz ISR.
    AccelSample filteredSample = getFilteredAccelSample(rawSample);

    // Select raw or filtered data for BOTH telemetry and SD.
    AccelSample outputSample =
        VIBRATION_LOG_RAW ? rawSample : filteredSample;

    // Exact nominal acquisition timestamp.
    uint32_t timestampUs =
        accSamplingStartUs +
        (uint32_t)((uint64_t)totalSampleIndex *
                   (uint64_t)ACC_SAMPLE_PERIOD_US);

    sendAccelTelemetrySample(outputSample);

    logAccelerationSample(
        outputSample,
        totalSampleIndex,
        timestampUs);

    // Convert the exact sample sent to USB/SD into calibrated g.
    latestVibrationX_g = rawToG(outputSample.x, adxlXOffset, adxlXSensitivity);
    latestVibrationY_g = rawToG(outputSample.y, adxlYOffset, adxlYSensitivity);
    latestVibrationZ_g = rawToG(outputSample.z, adxlZOffset, adxlZSensitivity);

    totalSampleIndex++;
    processed++;
  }
}


// ======================================================
// UPDATE SLOW SENSORS
// ======================================================

void updateSlowSensors()
{
  uint32_t now = millis();

  if ((uint32_t)(now - lastSensorUpdateMs) < SENSOR_UPDATE_PERIOD_MS)
    return;

  lastSensorUpdateMs = now;

  // HX711: read one conversion whenever it is ready.
  // Do not use get_units(3), because it performs extra ADC reads.
  if (loadCell1.is_ready())
  {
    latestRaw1 = loadCell1.read();
    latestWeight1 = ((float)latestRaw1 - (float)loadCell1.get_offset()) /
                    loadCell1.get_scale();
    hx1EverRead = true;
    latestHX1Ready = true;
  }
  else if (!hx1EverRead)
  {
    latestHX1Ready = false;
  }

  if (loadCell2.is_ready())
  {
    latestRaw2 = loadCell2.read();
    latestWeight2 = ((float)latestRaw2 - (float)loadCell2.get_offset()) /
                    loadCell2.get_scale();
    hx2EverRead = true;
    latestHX2Ready = true;
  }
  else if (!hx2EverRead)
  {
    latestHX2Ready = false;
  }

  // MAX31856 runs continuously; these are non-blocking register reads.
  latestTemp1 = thermocouple1.readThermocoupleTemperature();
  latestCJ1 = thermocouple1.readCJTemperature();
  latestFault1 = thermocouple1.readFault();

  latestTemp2 = thermocouple2.readThermocoupleTemperature();
  latestCJ2 = thermocouple2.readCJTemperature();
  latestFault2 = thermocouple2.readFault();
}


// ======================================================
// DISPLAY SLOW SENSOR VALUES
// ======================================================

void displaySlowSensors()
{
  uint32_t now = millis();

  if ((uint32_t)(now - lastDisplayMs) <
      DISPLAY_PERIOD_MS)
    return;

  lastDisplayMs = now;

  Serial.println();
  Serial.println("==============================================");

  Serial.println("LOAD CELL #1 - 10 kg");

  if (latestHX1Ready)
  {
    Serial.print("Raw ADC: ");
    Serial.println(latestRaw1);

    Serial.print("Weight: ");
    Serial.print(latestWeight1, 2);
    Serial.println(" g");
  }
  else
  {
    Serial.println("HX711 #1 NOT READY!");
  }

  Serial.println();
  Serial.println("LOAD CELL #2 - 1 kg");

  if (latestHX2Ready)
  {
    Serial.print("Raw ADC: ");
    Serial.println(latestRaw2);

    Serial.print("Weight: ");
    Serial.print(latestWeight2, 2);
    Serial.println(" g");
  }
  else
  {
    Serial.println("HX711 #2 NOT READY!");
  }

  Serial.println();
  Serial.println("MAX31856 #1");

  Serial.print("Thermocouple: ");
  Serial.print(latestTemp1, 2);
  Serial.println(" °C");

  Serial.print("Cold Junction: ");
  Serial.print(latestCJ1, 2);
  Serial.println(" °C");

  printFault1(latestFault1);

  Serial.println();
  Serial.println("MAX31856 #2");

  Serial.print("Thermocouple: ");
  Serial.print(latestTemp2, 2);
  Serial.println(" °C");

  Serial.print("Cold Junction: ");
  Serial.print(latestCJ2, 2);
  Serial.println(" °C");

  printFault2(latestFault2);

  Serial.println();
  Serial.print("ADXL buffer: ");
  Serial.print(accSampleCount);
  Serial.print("/");
  Serial.println(ACC_BUFFER_SAMPLES);

  Serial.print("ADXL overflows: ");
  Serial.println(accOverflowCount);

  Serial.print("Logging: ");
  Serial.println(loggingActive ? "ACTIVE" : "OFF");

  Serial.print("Telemetry: ");
  Serial.println(telemetryEnabled ? "ON" : "OFF");
}


// ======================================================
// MAX31856 FAULT 1
// ======================================================

void printFault1(uint8_t fault)
{
  if (fault == 0)
  {
    Serial.println("Status: OK");
    return;
  }

  Serial.print("FAULT 1 = 0x");
  Serial.println(fault, HEX);

  if (fault & MAX31856_FAULT_CJRANGE)
    Serial.println("  Cold junction range fault");

  if (fault & MAX31856_FAULT_TCRANGE)
    Serial.println("  Thermocouple range fault");

  if (fault & MAX31856_FAULT_CJHIGH)
    Serial.println("  Cold junction HIGH");

  if (fault & MAX31856_FAULT_CJLOW)
    Serial.println("  Cold junction LOW");

  if (fault & MAX31856_FAULT_TCHIGH)
    Serial.println("  Thermocouple HIGH");

  if (fault & MAX31856_FAULT_TCLOW)
    Serial.println("  Thermocouple LOW");

  if (fault & MAX31856_FAULT_OVUV)
    Serial.println("  Over/under voltage");

  if (fault & MAX31856_FAULT_OPEN)
    Serial.println("  Thermocouple OPEN");
}


// ======================================================
// MAX31856 FAULT 2
// ======================================================

void printFault2(uint8_t fault)
{
  if (fault == 0)
  {
    Serial.println("Status: OK");
    return;
  }

  Serial.print("FAULT 2 = 0x");
  Serial.println(fault, HEX);

  if (fault & MAX31856_FAULT_CJRANGE)
    Serial.println("  Cold junction range fault");

  if (fault & MAX31856_FAULT_TCRANGE)
    Serial.println("  Thermocouple range fault");

  if (fault & MAX31856_FAULT_CJHIGH)
    Serial.println("  Cold junction HIGH");

  if (fault & MAX31856_FAULT_CJLOW)
    Serial.println("  Cold junction LOW");

  if (fault & MAX31856_FAULT_TCHIGH)
    Serial.println("  Thermocouple HIGH");

  if (fault & MAX31856_FAULT_TCLOW)
    Serial.println("  Thermocouple LOW");

  if (fault & MAX31856_FAULT_OVUV)
    Serial.println("  Over/under voltage");

  if (fault & MAX31856_FAULT_OPEN)
    Serial.println("  Thermocouple OPEN");
}


// ======================================================
// SD CARD HEALTH / INITIALIZATION
// ======================================================
// The GUI can request SD_INIT at any time when logging is not active.
// This makes recovery from a card insertion/re-insertion possible without
// rebooting the Teensy. We also retry the SD initialization before logging,
// listing, and downloading so a transient initialization failure does not
// permanently disable the SD functions.
// ======================================================

static bool ensureSDCard(bool verbose)
{
  if (loggingActive)
  {
    return sdCardReady;
  }

  // Fast path: verify the root can actually be opened. This catches a card
  // that was removed after setup() even when sdCardReady was previously true.
  if (sdCardReady)
  {
    File root = SD.open("/");
    if (root)
    {
      root.close();
      return true;
    }
    sdCardReady = false;
  }

  if (verbose)
    Serial.println("SD: attempting built-in SD re-initialization...");

  // Retry a few times. Teensy 4.1 uses the dedicated built-in SDIO interface.
  for (uint8_t attempt = 1; attempt <= 3; attempt++)
  {
    if (SD.begin(BUILTIN_SDCARD))
    {
      // Confirm that the filesystem root is readable.
      File root = SD.open("/");
      if (root)
      {
        root.close();
        sdCardReady = true;
        if (verbose)
          Serial.println("SD CARD READY");
        return true;
      }
    }

    if (verbose)
    {
      Serial.print("SD init attempt ");
      Serial.print(attempt);
      Serial.println(" failed.");
    }
    delay(50);
  }

  sdCardReady = false;
  if (verbose)
    Serial.println("ERROR: SD CARD INIT FAILED - check card insertion/format.");
  return false;
}

static void handleSDInitCommand()
{
  if (loggingActive)
  {
    Serial.println("ERROR: SD_INIT rejected while logging is active.");
    return;
  }

  if (ensureSDCard(true))
    Serial.println("SD_INIT_OK");
  else
    Serial.println("SD_INIT_ERROR");
}


// ======================================================
// SD FILE NAME VALIDATION
// ======================================================

static bool validateSDDownloadFilename(const char *requestedName,
                                      char *safeName,
                                      size_t safeSize)
{
  if (!requestedName || !safeName || safeSize < 2)
    return false;

  size_t len = strlen(requestedName);
  if (len == 0 || len >= safeSize || len > SD_TRANSFER_MAX_FILENAME)
    return false;

  // Root-level filename only. No path separators or control characters.
  for (size_t i = 0; i < len; i++)
  {
    char c = requestedName[i];
    if (!(isalnum((unsigned char)c) || c == '_' || c == '-' || c == '.'))
      return false;
  }

  // Download/list interface is intentionally restricted to .bin files.
  if (len < 4 || strcasecmp(&requestedName[len - 4], ".bin") != 0)
    return false;

  strncpy(safeName, requestedName, safeSize - 1);
  safeName[safeSize - 1] = '\0';
  return true;
}


// ======================================================
// SD FILE LIST
// ======================================================

static void handleSDListCommand()
{
  if (loggingActive)
  {
    Serial.println("ERROR: SD_LIST rejected while logging is active. Use LOGSTOP first.");
    return;
  }

  if (!ensureSDCard(true))
  {
    Serial.println("ERROR: SD_LIST unavailable because the SD card is not ready.");
    return;
  }

  File root = SD.open("/");
  if (!root)
  {
    // Some SD library revisions are happier with an empty root path. Retry
    // once after reinitialization before reporting failure.
    sdCardReady = false;
    if (!ensureSDCard(true))
    {
      Serial.println("ERROR: Could not open SD root. Check card insertion/format.");
      return;
    }
    root = SD.open("/");
  }

  if (!root)
  {
    Serial.println("ERROR: Could not open SD root. Check card insertion/format.");
    return;
  }

  bool wasSampling = accSamplingEnabled;
  bool wasTelemetry = telemetryEnabled;

  stopAccelerationSampling();
  flushTelemetryPacket();
  telemetryEnabled = false;

  Serial.println("SD_LIST_BEGIN");

  uint32_t count = 0;
  File entry = root.openNextFile();

  while (entry)
  {
    if (!entry.isDirectory())
    {
      const char *name = entry.name();
      size_t len = strlen(name);

      if (len >= 4 && strcasecmp(&name[len - 4], ".bin") == 0)
      {
        Serial.print("FILE ");
        Serial.print(name);
        Serial.print(" ");
        Serial.println((uint32_t)entry.size());
        count++;
      }
    }

    entry.close();
    entry = root.openNextFile();
  }

  root.close();

  Serial.print("SD_LIST_COUNT ");
  Serial.println(count);
  Serial.println("SD_LIST_END");

  telemetryEnabled = wasTelemetry;

  if (wasSampling)
  {
    clearAccelerationBuffer();
    startAccelerationSampling();
  }
}


// ======================================================
// SD DOWNLOAD DATA FRAME
// ======================================================
// Frame:
//   D5 A5 TYPE OFFSET[4 LE] LENGTH[2 LE] DATA CRC16[2 LE]
// CRC covers TYPE + OFFSET + LENGTH + DATA.
// ======================================================

__attribute__((unused)) static void sendSDDataFrame(uint32_t offset,
                            const uint8_t *data,
                            uint16_t length,
                            uint16_t &fileCRC)
{
  uint8_t header[9];
  header[0] = SD_TRANSFER_SYNC_1;
  header[1] = SD_TRANSFER_SYNC_2;
  header[2] = SD_TRANSFER_TYPE_DATA;

  header[3] = offset & 0xFF;
  header[4] = (offset >> 8) & 0xFF;
  header[5] = (offset >> 16) & 0xFF;
  header[6] = (offset >> 24) & 0xFF;

  header[7] = length & 0xFF;
  header[8] = (length >> 8) & 0xFF;

  // CRC for the transfer frame excludes the two sync bytes.
  uint8_t crcBuffer[1 + 4 + 2 + SD_TRANSFER_CHUNK_SIZE];
  memcpy(crcBuffer, &header[2], 7);
  if (length > 0)
    memcpy(&crcBuffer[7], data, length);

  uint16_t frameCRC = crc16_ccitt(crcBuffer, 7 + length);

  Serial.write(header, sizeof(header));
  if (length > 0)
    Serial.write(data, length);

  uint8_t crcBytes[2];
  crcBytes[0] = frameCRC & 0xFF;
  crcBytes[1] = frameCRC >> 8;
  Serial.write(crcBytes, sizeof(crcBytes));
}


// Incremental CRC helper for the complete file.
uint16_t crc16_update(uint16_t crc,
                      const uint8_t *data,
                      uint16_t length)
{
  for (uint16_t i = 0; i < length; i++)
  {
    crc ^= (uint16_t)data[i] << 8;

    for (uint8_t bit = 0; bit < 8; bit++)
    {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }

  return crc;
}


// ======================================================
// SD DOWNLOAD HOST ACK
// ======================================================
// The PC acknowledges every completed data frame with:
//   SD_ACK <next byte offset>
// This makes the transfer stop-and-wait and prevents USB CDC
// buffering overruns on large files.
// ======================================================

static bool waitForSDDownloadAck(uint32_t expectedOffset, uint32_t timeoutMs)
{
  String ack = "";
  uint32_t start = millis();

  while ((uint32_t)(millis() - start) < timeoutMs)
  {
    while (Serial.available() > 0)
    {
      char c = (char)Serial.read();

      if (c == '\n')
      {
        ack.trim();

        if (ack.startsWith("SD_ACK "))
        {
          String value = ack.substring(7);
          value.trim();

          char *endPtr = nullptr;
          unsigned long parsed = strtoul(value.c_str(), &endPtr, 10);

          if (endPtr != value.c_str() && *endPtr == '\0' &&
              (uint32_t)parsed == expectedOffset)
          {
            return true;
          }
        }

        ack = "";
      }
      else if (c != '\r')
      {
        if (ack.length() < 48)
          ack += c;
        else
          ack = "";
      }
    }

    delayMicroseconds(100);
  }

  return false;
}


// ======================================================
// SD FILE DOWNLOAD
// ======================================================

static bool handleSDDownloadCommand(const char *requestedName)
{
  if (loggingActive)
  {
    Serial.print("SD_DOWNLOAD_ERROR ");
    Serial.println(requestedName ? requestedName : "");
    Serial.println("SD_DOWNLOAD rejected while logging is active. Use LOGSTOP first.");
    return false;
  }

  if (!ensureSDCard(true))
  {
    Serial.println("SD_DOWNLOAD_ERROR SD_CARD_NOT_READY");
    return false;
  }

  char safeName[SD_TRANSFER_MAX_FILENAME + 1];
  if (!validateSDDownloadFilename(requestedName, safeName, sizeof(safeName)))
  {
    Serial.print("SD_DOWNLOAD_ERROR ");
    Serial.println(requestedName ? requestedName : "");
    Serial.println("Invalid filename. Only root-level .bin filenames are allowed.");
    return false;
  }

  File file = SD.open(safeName, FILE_READ);
  if (!file)
  {
    Serial.print("SD_DOWNLOAD_ERROR ");
    Serial.println(safeName);
    Serial.println("File could not be opened.");
    return false;
  }

  bool wasSampling = accSamplingEnabled;
  bool wasTelemetry = telemetryEnabled;

  stopAccelerationSampling();
  flushTelemetryPacket();
  telemetryEnabled = false;

  uint32_t totalSize = (uint32_t)file.size();

  Serial.print("SD_DOWNLOAD_BEGIN ");
  Serial.print(safeName);
  Serial.print(" ");
  Serial.println(totalSize);

  uint8_t buffer[SD_TRANSFER_CHUNK_SIZE];
  uint32_t offset = 0;
  uint16_t wholeCRC = 0xFFFF;

  while (file.available())
  {
    int bytesRead = file.read(buffer, sizeof(buffer));
    if (bytesRead <= 0)
      break;

    uint16_t length = (uint16_t)bytesRead;

    // Build the 7-byte frame body without allocating another 1 KB+
    // temporary CRC buffer on the stack.
    uint8_t frameBody[7];
    frameBody[0] = SD_TRANSFER_TYPE_DATA;
    frameBody[1] = offset & 0xFF;
    frameBody[2] = (offset >> 8) & 0xFF;
    frameBody[3] = (offset >> 16) & 0xFF;
    frameBody[4] = (offset >> 24) & 0xFF;
    frameBody[5] = length & 0xFF;
    frameBody[6] = (length >> 8) & 0xFF;

    uint16_t frameCRC = crc16_update(0xFFFF, frameBody, sizeof(frameBody));
    frameCRC = crc16_update(frameCRC, buffer, length);
    wholeCRC = crc16_update(wholeCRC, buffer, length);

    uint8_t header[9];
    header[0] = SD_TRANSFER_SYNC_1;
    header[1] = SD_TRANSFER_SYNC_2;
    memcpy(&header[2], frameBody, sizeof(frameBody));

    uint8_t crcBytes[2];
    crcBytes[0] = frameCRC & 0xFF;
    crcBytes[1] = frameCRC >> 8;

    bool acknowledged = false;

    // Send the frame and wait for the PC to acknowledge the next offset.
    // Retry a few times if the ACK is lost/corrupted.
    for (uint8_t attempt = 0; attempt < 4 && !acknowledged; attempt++)
    {
      if (attempt > 0)
      {
        // Briefly yield before retransmitting the exact same frame.
        delay(2);
      }

      Serial.write(header, sizeof(header));
      Serial.write(buffer, length);
      Serial.write(crcBytes, sizeof(crcBytes));
      Serial.flush();

      acknowledged = waitForSDDownloadAck(offset + length, 1200);
    }

    if (!acknowledged)
    {
      Serial.print("SD_DOWNLOAD_ERROR ACK_TIMEOUT ");
      Serial.println(offset);
      file.close();

      telemetryEnabled = wasTelemetry;
      if (wasSampling)
      {
        clearAccelerationBuffer();
        startAccelerationSampling();
      }
      return false;
    }

    offset += length;
  }

  file.close();

  Serial.print("SD_DOWNLOAD_END ");
  Serial.print(safeName);
  Serial.print(" ");
  Serial.print(offset);
  Serial.print(" ");
  if (wholeCRC < 0x1000) Serial.print("0");
  if (wholeCRC < 0x0100) Serial.print("0");
  if (wholeCRC < 0x0010) Serial.print("0");
  Serial.println(wholeCRC, HEX);

  telemetryEnabled = wasTelemetry;

  if (wasSampling)
  {
    clearAccelerationBuffer();
    startAccelerationSampling();
  }

  return true;
}


// ======================================================
// SERIAL COMMAND HANDLER
// ======================================================
//
// TARE10
// TARE1
// TAREALL
// ACC_CAL
//
// LOGSTART test01
// LOGSTART test01.bin
// STARTLOG test01
// STARTLOG test01.bin
//
// LOGSTOP
//
// TELEMETRY ON
// TELEMETRY OFF
//
// STATUS
// ======================================================

void handleSerialCommand()
{
  while (Serial.available())
  {
    char c = Serial.read();

    if (c == '\n' || c == '\r')
    {
      if (serialCommand.length() == 0)
        continue;

      serialCommand.trim();

      String commandUpper = serialCommand;
      commandUpper.toUpperCase();

      // ==================================================
      // TARE 10 KG
      // ==================================================

      if (commandUpper == "TARE10")
      {
        Serial.println();
        Serial.println("==============================================");
        Serial.println("TARE COMMAND: 10 KG");
        Serial.println("Remove all force from 10 kg load cell.");
        Serial.println("Taring...");
        Serial.println("==============================================");

        if (loadCell1.is_ready())
        {
          loadCell1.set_scale(1.0);
          delay(300);
          loadCell1.tare(20);
          loadCell1.set_scale(calibrationFactor1);

          Serial.println("10 KG LOAD CELL TARE COMPLETE");
        }
        else
        {
          Serial.println("ERROR: 10 KG HX711 NOT READY!");
        }

        Serial.println();
      }

      // ==================================================
      // TARE 1 KG
      // ==================================================

      else if (commandUpper == "TARE1")
      {
        Serial.println();
        Serial.println("==============================================");
        Serial.println("TARE COMMAND: 1 KG");
        Serial.println("Remove all force from 1 kg load cell.");
        Serial.println("Taring...");
        Serial.println("==============================================");

        if (loadCell2.is_ready())
        {
          loadCell2.set_scale(1.0);
          delay(300);
          loadCell2.tare(20);
          loadCell2.set_scale(calibrationFactor2);

          Serial.println("1 KG LOAD CELL TARE COMPLETE");
        }
        else
        {
          Serial.println("ERROR: 1 KG HX711 NOT READY!");
        }

        Serial.println();
      }

      // ==================================================
      // TARE BOTH
      // ==================================================

      else if (commandUpper == "TAREALL")
      {
        Serial.println();
        Serial.println("==============================================");
        Serial.println("TARE COMMAND: BOTH LOAD CELLS");
        Serial.println("Remove all force from both load cells.");
        Serial.println("Taring...");
        Serial.println("==============================================");

        if (loadCell1.is_ready())
        {
          loadCell1.set_scale(1.0);
          delay(300);
          loadCell1.tare(20);
          loadCell1.set_scale(calibrationFactor1);

          Serial.println("10 KG LOAD CELL TARE COMPLETE");
        }
        else
        {
          Serial.println("10 KG HX711 NOT READY!");
        }

        if (loadCell2.is_ready())
        {
          loadCell2.set_scale(1.0);
          delay(300);
          loadCell2.tare(20);
          loadCell2.set_scale(calibrationFactor2);

          Serial.println("1 KG LOAD CELL TARE COMPLETE");
        }
        else
        {
          Serial.println("1 KG HX711 NOT READY!");
        }

        Serial.println("BOTH TARE OPERATIONS FINISHED");
        Serial.println();
      }

      // ==================================================
      // ADXL CALIBRATION
      // ==================================================

      else if (commandUpper == "ACC_CAL")
      {
        performADXLCalibration();
      }

      // ==================================================
      // GLOBAL THROTTLE
      // ==================================================
      // Example: THROTTLE 42.5
      // ==================================================

      else if (commandUpper.startsWith("THROTTLE "))
      {
        int spaceIndex = serialCommand.indexOf(' ');

        if (spaceIndex > 0)
        {
          float value = serialCommand.substring(spaceIndex + 1).toFloat();
          throttlePercent = constrain(value, 0.0f, 100.0f);

          // Laptop DAQ -> global throttle -> servo + Nextion.
          updateThrottleServo();
          nextionSetThrottleSlider(throttlePercent);

          Serial.print("THROTTLE GLOBAL = ");
          Serial.print(throttlePercent, 1);
          Serial.println(" %");
        }
        else
        {
          Serial.println("ERROR: THROTTLE requires 0..100");
        }
      }

      // ==================================================
      // LOGSTART / STARTLOG
      // ==================================================

      else if (commandUpper.startsWith("LOGSTART ") ||
               commandUpper.startsWith("STARTLOG "))
      {
        int spaceIndex = serialCommand.indexOf(' ');

        if (spaceIndex > 0)
        {
          String filename =
              serialCommand.substring(spaceIndex + 1);

          filename.trim();

          if (filename.length() > 0)
          {
            startLogging(filename.c_str());
          }
          else
          {
            Serial.println(
                "ERROR: LOGSTART requires a filename.");
          }
        }
      }

      // ==================================================
      // LOGSTOP
      // ==================================================

      else if (commandUpper == "LOGSTOP")
      {
        stopLogging();
      }

      // ==================================================
      // TELEMETRY ON
      // ==================================================

      else if (commandUpper == "TELEMETRY ON")
      {
        telemetryEnabled = true;
        Serial.println(
            "Binary acceleration telemetry: ON");
      }

      // ==================================================
      // TELEMETRY OFF
      // ==================================================

      else if (commandUpper == "TELEMETRY OFF")
      {
        flushTelemetryPacket();
        telemetryEnabled = false;

        Serial.println(
            "Binary acceleration telemetry: OFF");
      }

      // ==================================================
      // SD INIT / HEALTH
      // ==================================================

      else if (commandUpper == "SD_INIT" || commandUpper == "SDINIT")
      {
        handleSDInitCommand();
      }

      // ==================================================
      // STATUS
      // ==================================================

      else if (commandUpper == "SD_LIST" || commandUpper == "SDLIST")
      {
        handleSDListCommand();
      }

      // ==================================================
      // SD DOWNLOAD
      // ==================================================
      else if (commandUpper.startsWith("SD_DOWNLOAD ") || commandUpper.startsWith("SDDOWNLOAD "))
      {
        int spaceIndex = serialCommand.indexOf(' ');
        if (spaceIndex > 0)
        {
          String filename = serialCommand.substring(spaceIndex + 1);
          filename.trim();
          if (filename.length() > 0) handleSDDownloadCommand(filename.c_str());
          else Serial.println("ERROR: SD_DOWNLOAD requires a filename.");
        }
        else Serial.println("ERROR: SD_DOWNLOAD requires a filename.");
      }

      // ==================================================
      // STATUS
      // ==================================================
      else if (commandUpper == "STATUS")
      {
        Serial.println();
        Serial.println("=============== DAQ STATUS ===============");

        Serial.print("Acceleration sample rate: ");
        Serial.print(ACC_SAMPLE_RATE_HZ);
        Serial.println(" Hz");

  Serial.print("Noise filter = ");
  Serial.println(VIBRATION_FILTER_ENABLED ? "ON" : "OFF");

  Serial.print("Filter cutoff = ");
  Serial.print(VIBRATION_FILTER_CUTOFF_HZ, 1);
  Serial.println(" Hz");

  Serial.print("Vibration output = ");
  Serial.println(VIBRATION_LOG_RAW ? "RAW" : "FILTERED");

        Serial.print("Telemetry packet rate: ");
        Serial.print(TELEMETRY_RATE_HZ);
        Serial.println(" packets/s");

        Serial.print("Samples per packet: ");
        Serial.println(TELEMETRY_PACKET_SAMPLES);

        Serial.print("Acceleration payload rate: ");
        Serial.print((uint32_t)ACC_SAMPLE_RATE_HZ * TELEMETRY_BYTES_PER_SAMPLE);
        Serial.println(" bytes/s");

        Serial.println("HX711: hardware-limited (10/80 SPS)");

         Serial.print("Vibration filter: ");
         Serial.println(VIBRATION_FILTER_ENABLED ? "ON" : "OFF");

         Serial.print("Vibration cutoff: ");
         Serial.print(VIBRATION_FILTER_CUTOFF_HZ, 1);
         Serial.println(" Hz");

         Serial.print("Vibration output: ");
         Serial.println(VIBRATION_LOG_RAW ? "RAW" : "FILTERED");
        Serial.println("MAX31856: continuous conversion (~10-11 SPS)");

        Serial.print("Buffer occupancy: ");
        Serial.println(accSampleCount);

        Serial.print("Buffer overflows: ");
        Serial.println(accOverflowCount);

        Serial.print("Logging: ");
        Serial.println(
            loggingActive ? "ACTIVE" : "OFF");

        if (loggingActive)
        {
          Serial.print("Log file: ");
          Serial.println(activeLogFileName);

          Serial.print("Log records: ");
          Serial.println(logRecordCount);
        }

        Serial.println("===========================================");
      }

      // ==================================================
      // UNKNOWN COMMAND
      // ==================================================

      else
      {
        Serial.println();
        Serial.print("Unknown command: ");
        Serial.println(serialCommand);

        Serial.println();
        Serial.println("Available commands:");
        Serial.println("TARE10");
        Serial.println("TARE1");
        Serial.println("TAREALL");
        Serial.println("ACC_CAL");
        Serial.println("LOGSTART test01");
        Serial.println("LOGSTOP");
        Serial.println("SD_INIT");
        Serial.println("SD_LIST");
        Serial.println("SD_DOWNLOAD filename.bin");
        Serial.println("TELEMETRY ON");
        Serial.println("TELEMETRY OFF");
        Serial.println("THROTTLE 0..100");
        Serial.println("STATUS");
        Serial.println();
      }

      serialCommand = "";
    }
    else
    {
      if (serialCommand.length() < 100)
        serialCommand += c;
    }
  }
}


// ======================================================
// SETUP
// ======================================================

void setup()
{
  Serial.begin(SERIAL_BAUD);

  // USB Serial remains the laptop DAQ connection.

  // Initialize the physical throttle servo on Teensy pin 26.
  // It follows the same global throttlePercent used by Nextion and DAQ.
  initThrottleServo();

  // Nextion uses the separate Serial6 UART.
  nextionBegin();

  delay(1500);

  Serial.println();
  Serial.println("================================================");
  Serial.println(" TEENSY 4.1 ENGINE TEST-BENCH DAQ");
  Serial.println(" 2x HX711 + 2x MAX31856 + ADXL335");
  Serial.println("================================================");


  // ====================================================
  // ADC / ADXL335
  // ====================================================

  analogReadResolution(ADC_RESOLUTION_BITS);
  analogReadAveraging(1);

  pinMode(ADXL_X_PIN, INPUT);
  pinMode(ADXL_Y_PIN, INPUT);
  pinMode(ADXL_Z_PIN, INPUT);

  Serial.println();
  Serial.println("ADXL335:");
  Serial.print("XOUT = pin ");
  Serial.println(ADXL_X_PIN);

  Serial.print("YOUT = pin ");
  Serial.println(ADXL_Y_PIN);

  Serial.print("ZOUT = pin ");
  Serial.println(ADXL_Z_PIN);

  Serial.print("Sample rate = ");
  Serial.print(ACC_SAMPLE_RATE_HZ);
  Serial.println(" Hz");


  // ====================================================
  // HX711 #1 - 10 KG
  // ====================================================

  Serial.println();
  Serial.println("Initializing HX711 #1...");
  Serial.println("10 kg Load Cell");

  Serial.print("DOUT = Pin ");
  Serial.println(HX1_DOUT);

  Serial.print("SCK  = Pin ");
  Serial.println(HX1_SCK);

  // Optional HX711 RATE pins: HIGH = 80 SPS, LOW = 10 SPS.
#if HX1_RATE_PIN >= 0
  pinMode(HX1_RATE_PIN, OUTPUT);
  digitalWrite(HX1_RATE_PIN, HX711_FAST_RATE_ENABLED ? HIGH : LOW);
#endif
#if HX2_RATE_PIN >= 0
  pinMode(HX2_RATE_PIN, OUTPUT);
  digitalWrite(HX2_RATE_PIN, HX711_FAST_RATE_ENABLED ? HIGH : LOW);
#endif

  loadCell1.begin(HX1_DOUT, HX1_SCK);

  delay(500);

  if (loadCell1.is_ready())
  {
    Serial.println("HX711 #1 READY");
    loadCell1.set_scale(calibrationFactor1);
  }
  else
  {
    Serial.println(
        "ERROR: HX711 #1 NOT READY!");
  }


  // ====================================================
  // HX711 #2 - 1 KG
  // ====================================================

  Serial.println();
  Serial.println("Initializing HX711 #2...");
  Serial.println("1 kg Load Cell");

  Serial.print("DOUT = Pin ");
  Serial.println(HX2_DOUT);

  Serial.print("SCK  = Pin ");
  Serial.println(HX2_SCK);

  loadCell2.begin(HX2_DOUT, HX2_SCK);

  delay(500);

  if (loadCell2.is_ready())
  {
    Serial.println("HX711 #2 READY");
    loadCell2.set_scale(calibrationFactor2);
  }
  else
  {
    Serial.println(
        "ERROR: HX711 #2 NOT READY!");
  }


  // ====================================================
  // MAX31856 #1
  // ====================================================

  Serial.println();
  Serial.println("Initializing MAX31856 #1...");

  if (!thermocouple1.begin())
  {
    Serial.println(
        "ERROR: MAX31856 #1 NOT FOUND!");
  }
  else
  {
    Serial.println("MAX31856 #1 OK");

    thermocouple1.setThermocoupleType(
        MAX31856_TCTYPE_K);
    thermocouple1.setNoiseFilter(MAX_NOISE_FILTER);
    thermocouple1.setConversionMode(MAX31856_CONTINUOUS);

    Serial.println(
        "MAX31856 #1 = K-Type");
  }


  // ====================================================
  // MAX31856 #2
  // ====================================================

  Serial.println();
  Serial.println("Initializing MAX31856 #2...");

  if (!thermocouple2.begin())
  {
    Serial.println(
        "ERROR: MAX31856 #2 NOT FOUND!");
  }
  else
  {
    Serial.println("MAX31856 #2 OK");

    thermocouple2.setThermocoupleType(
        MAX31856_TCTYPE_K);
    thermocouple2.setNoiseFilter(MAX_NOISE_FILTER);
    thermocouple2.setConversionMode(MAX31856_CONTINUOUS);

    Serial.println(
        "MAX31856 #2 = K-Type");
  }


  // ====================================================
  // BUILT-IN MICROSD
  // ====================================================

  Serial.println();
  Serial.println(
      "Initializing built-in microSD...");

  if (ensureSDCard(true))
  {
    Serial.println("SD CARD READY");
  }
  else
  {
    Serial.println("WARNING: SD CARD NOT FOUND.");
    Serial.println("Insert/format the card, then use SD_INIT or reboot the Teensy.");
  }


  // ====================================================
  // INITIAL TARE
  // ====================================================

  Serial.println();
  Serial.println("==============================================");
  Serial.println("LOAD CELL TARE");
  Serial.println(
      "Remove ALL weight from both load cells.");
  Serial.println(
      "Taring in 3 seconds...");
  Serial.println("==============================================");

  delay(3000);

  if (loadCell1.is_ready())
  {
    loadCell1.set_scale(1.0);
    loadCell1.tare(20);
    loadCell1.set_scale(calibrationFactor1);

    Serial.println(
        "HX711 #1 - 10 KG TARE COMPLETE");
  }

  if (loadCell2.is_ready())
  {
    loadCell2.set_scale(1.0);
    loadCell2.tare(20);
    loadCell2.set_scale(calibrationFactor2);

    Serial.println(
        "HX711 #2 - 1 KG TARE COMPLETE");
  }


  // ====================================================
  // COMMAND INFORMATION
  // ====================================================

  Serial.println();
  Serial.println("==============================================");
  Serial.println("SERIAL COMMANDS");
  Serial.println("==============================================");
  Serial.println(
      "TARE10              -> Tare 10 kg");
  Serial.println(
      "TARE1               -> Tare 1 kg");
  Serial.println(
      "TAREALL             -> Tare both");
  Serial.println(
      "ACC_CAL             -> Calibrate ADXL335");
  Serial.println(
      "LOGSTART test01     -> Start SD binary log");
  Serial.println(
      "LOGSTOP             -> Stop SD binary log");
  Serial.println(
      "SD_INIT             -> Re-initialize/check SD card");
  Serial.println(
      "SD_LIST             -> List .bin files on SD");
  Serial.println(
      "SD_DOWNLOAD file    -> Download binary file");
  Serial.println(
      "SD_ACK offset       -> PC-only download acknowledgement");
  Serial.println(
      "TELEMETRY ON        -> Binary USB telemetry");
  Serial.println(
      "TELEMETRY OFF       -> Stop binary telemetry");
  Serial.println(
      "STATUS              -> DAQ status");
  Serial.println("==============================================");


  // ====================================================
  // START 2 kHz ACCELERATION
  // ====================================================

  // IMPORTANT: initialize Butterworth coefficients before the first sample.
  // Without this call the filter coefficients remain zero and vibration output
  // collapses to zero even though the ADC itself is reading correctly.
  configureVibrationFilter();

  clearAccelerationBuffer();
  startAccelerationSampling();

  lastSensorUpdateMs = millis();
  lastDisplayMs = millis();

  Serial.println();
  Serial.println("==============================================");
  Serial.println("STARTING MEASUREMENTS");
  Serial.println("==============================================");
}


// ======================================================
// LOOP
// ======================================================

void loop()
{
  // 1. Commands
  handleSerialCommand();

  // 2. Consume 2 kHz acceleration samples
  processAccelerationBuffer();

  // 3. Update HX711/MAX31856 at slower rate
  updateSlowSensors();

  // 4. Human-readable monitor output
  displaySlowSensors();

  // 5. Nextion display + global throttle synchronization
  serviceNextion();

  // 6. Keep the physical servo locked to the global throttle value.
  // This is intentionally non-blocking and only writes when the
  // integer throttle value changes.
  updateThrottleServo();
}
