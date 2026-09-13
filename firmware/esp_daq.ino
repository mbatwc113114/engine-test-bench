/*
 * ============================================================
 * ENGINE TEST BENCH DAQ - ESP32 COMMUNICATION FIRMWARE
 * ============================================================
 *
 * Features:
 *   - WiFi connection
 *   - mDNS discovery
 *   - UDP communication
 *   - Configuration upload
 *   - GUI handshake
 *   - State machine
 *   - 60 Hz telemetry transmission
 *   - Throttle servo control
 *   - Connection timeout safety
 *
 * Servo:
 *   GPIO 9
 *
 * UDP:
 *   ESP32 listens on UDP_PORT
 *
 * mDNS:
 *   engine-daq.local
 *
 * Service:
 *   _engine-daq._udp
 *
 * ============================================================
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

// ============================================================
// USER CONFIGURATION
// ============================================================

const char* WIFI_SSID     = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

const char* MDNS_HOSTNAME = "engine-daq";

const uint16_t UDP_PORT = 4210;

// Requested throttle servo pin
const int THROTTLE_SERVO_PIN = 9;

// Telemetry frequency
const uint32_t TELEMETRY_INTERVAL_US = 1000000UL / 60UL;

// Safety timeout
const uint32_t GUI_TIMEOUT_MS = 3000;

// Servo limits
const int SERVO_MIN_US = 1000;
const int SERVO_MAX_US = 2000;

// ============================================================
// OBJECTS
// ============================================================

WiFiUDP udp;

Servo throttleServo;

// ============================================================
// STATE MACHINE
// ============================================================

enum SystemState
{
    STATE_BOOT,
    STATE_WIFI_CONNECTING,
    STATE_READY,
    STATE_CONFIG_RECEIVED,
    STATE_DAQ_READY,
    STATE_RUNNING,
    STATE_STOPPED,
    STATE_ERROR
};

SystemState systemState = STATE_BOOT;

// ============================================================
// CONFIGURATION
// ============================================================

struct DAQConfig
{
    int loadCell1Data;
    int loadCell1Clock;

    int loadCell2Data;
    int loadCell2Clock;

    int thermocouple1CS;
    int thermocouple1SCK;
    int thermocouple1MISO;
    int thermocouple1MOSI;

    int thermocouple2CS;
    int thermocouple2SCK;
    int thermocouple2MISO;
    int thermocouple2MOSI;

    int rpmPin;

    int sampleRate;
    int udpPort;

    int servoPin;

    int servoMinUs;
    int servoMaxUs;
};

DAQConfig config;

// ============================================================
// RUNTIME DATA
// ============================================================

float rpm = 0.0f;
float throttle = 0.0f;
float fuel = 0.0f;

float cht = 0.0f;
float egt = 0.0f;

float vibrationX = 0.0f;
float vibrationY = 0.0f;
float vibrationZ = 0.0f;

float thrust = 0.0f;

float battery = 0.0f;

uint32_t packetCounter = 0;

uint32_t lastTelemetryMicros = 0;
uint32_t lastGuiPacketMillis = 0;

IPAddress guiIP;
uint16_t guiPort = 0;

bool guiConnected = false;
bool configurationValid = false;

// ============================================================
// STATE NAME
// ============================================================

const char* stateName(SystemState state)
{
    switch (state)
    {
        case STATE_BOOT:
            return "BOOT";

        case STATE_WIFI_CONNECTING:
            return "WIFI_CONNECTING";

        case STATE_READY:
            return "READY";

        case STATE_CONFIG_RECEIVED:
            return "CONFIG_RECEIVED";

        case STATE_DAQ_READY:
            return "DAQ_READY";

        case STATE_RUNNING:
            return "RUNNING";

        case STATE_STOPPED:
            return "STOPPED";

        case STATE_ERROR:
            return "ERROR";

        default:
            return "UNKNOWN";
    }
}

// ============================================================
// SET STATE
// ============================================================

void setState(SystemState newState)
{
    systemState = newState;

    Serial.print("STATE -> ");
    Serial.println(stateName(systemState));

    sendStatePacket();
}

// ============================================================
// SERVO
// ============================================================

void setThrottle(float percent)
{
    percent = constrain(percent, 0.0f, 100.0f);

    throttle = percent;

    int pulseWidth = map(
        (int)(percent * 10.0f),
        0,
        1000,
        config.servoMinUs,
        config.servoMaxUs
    );

    pulseWidth = constrain(
        pulseWidth,
        config.servoMinUs,
        config.servoMaxUs
    );

    throttleServo.writeMicroseconds(pulseWidth);
}

// ============================================================
// SEND UDP JSON
// ============================================================

void sendJson(JsonDocument& doc)
{
    if (!guiConnected)
        return;

    udp.beginPacket(guiIP, guiPort);

    serializeJson(doc, udp);

    udp.endPacket();
}

// ============================================================
// STATE PACKET
// ============================================================

void sendStatePacket()
{
    if (!guiConnected)
        return;

    StaticJsonDocument<512> doc;

    doc["type"] = "state";
    doc["device"] = "ESP32_DAQ";
    doc["state"] = stateName(systemState);
    doc["mdns"] = MDNS_HOSTNAME;
    doc["ip"] = WiFi.localIP().toString();
    doc["port"] = UDP_PORT;
    doc["timestamp"] = millis();

    sendJson(doc);
}

// ============================================================
// HELLO / DEVICE INFORMATION
// ============================================================

void sendHello()
{
    if (!guiConnected)
        return;

    StaticJsonDocument<512> doc;

    doc["type"] = "hello";
    doc["device"] = "ESP32_DAQ";
    doc["version"] = "1.0.0";

    doc["state"] = stateName(systemState);

    doc["ip"] = WiFi.localIP().toString();
    doc["port"] = UDP_PORT;

    doc["telemetry_hz"] = 60;

    doc["servo_pin"] = THROTTLE_SERVO_PIN;

    sendJson(doc);
}

// ============================================================
// TELEMETRY
// ============================================================

void sendTelemetry()
{
    if (!guiConnected)
        return;

    StaticJsonDocument<1024> doc;

    doc["type"] = "telemetry";

    doc["timestamp_us"] = micros();

    doc["packet"] = packetCounter++;

    doc["state"] = stateName(systemState);

    JsonObject data = doc.createNestedObject("data");

    data["rpm"] = rpm;

    // Throttle is the commanded throttle position.
    data["throttle"] = throttle;

    // In your GUI this gauge can be labelled THRUST.
    data["thrust"] = thrust;

    data["fuel"] = fuel;

    data["cht"] = cht;

    data["egt"] = egt;

    JsonObject vibration = data.createNestedObject("vibration");

    vibration["x"] = vibrationX;
    vibration["y"] = vibrationY;
    vibration["z"] = vibrationZ;

    data["battery"] = battery;

    sendJson(doc);
}

// ============================================================
// ACK
// ============================================================

void sendAck(const char* command, bool success)
{
    StaticJsonDocument<512> doc;

    doc["type"] = "ack";
    doc["command"] = command;
    doc["success"] = success;
    doc["state"] = stateName(systemState);
    doc["timestamp"] = millis();

    sendJson(doc);
}

// ============================================================
// ERROR
// ============================================================

void sendError(const char* message)
{
    StaticJsonDocument<512> doc;

    doc["type"] = "error";
    doc["message"] = message;
    doc["state"] = stateName(systemState);

    sendJson(doc);
}

// ============================================================
// CONFIGURATION PARSER
// ============================================================

bool parseConfiguration(JsonDocument& doc)
{
    JsonObject cfg = doc["config"];

    if (cfg.isNull())
    {
        sendError("Missing config object");
        return false;
    }

    // -------------------------
    // LOAD CELL 1
    // -------------------------

    config.loadCell1Data =
        cfg["load_cell_1"]["data_pin"] | 34;

    config.loadCell1Clock =
        cfg["load_cell_1"]["clock_pin"] | 35;

    // -------------------------
    // LOAD CELL 2
    // -------------------------

    config.loadCell2Data =
        cfg["load_cell_2"]["data_pin"] | 32;

    config.loadCell2Clock =
        cfg["load_cell_2"]["clock_pin"] | 33;

    // -------------------------
    // THERMOCOUPLE 1
    // -------------------------

    config.thermocouple1CS =
        cfg["thermocouple_1"]["cs_pin"] | 5;

    config.thermocouple1SCK =
        cfg["thermocouple_1"]["sck_pin"] | 18;

    config.thermocouple1MISO =
        cfg["thermocouple_1"]["miso_pin"] | 19;

    config.thermocouple1MOSI =
        cfg["thermocouple_1"]["mosi_pin"] | 23;

    // -------------------------
    // THERMOCOUPLE 2
    // -------------------------

    config.thermocouple2CS =
        cfg["thermocouple_2"]["cs_pin"] | 17;

    config.thermocouple2SCK =
        cfg["thermocouple_2"]["sck_pin"] | 18;

    config.thermocouple2MISO =
        cfg["thermocouple_2"]["miso_pin"] | 19;

    config.thermocouple2MOSI =
        cfg["thermocouple_2"]["mosi_pin"] | 23;

    // -------------------------
    // RPM
    // -------------------------

    config.rpmPin =
        cfg["rpm"]["pin"] | 27;

    // -------------------------
    // DAQ
    // -------------------------

    config.sampleRate =
        cfg["daq"]["sample_rate"] | 1000;

    config.udpPort =
        cfg["daq"]["udp_port"] | UDP_PORT;

    // -------------------------
    // SERVO
    // -------------------------

    config.servoPin =
        cfg["servo"]["pin"] | THROTTLE_SERVO_PIN;

    config.servoMinUs =
        cfg["servo"]["min_us"] | SERVO_MIN_US;

    config.servoMaxUs =
        cfg["servo"]["max_us"] | SERVO_MAX_US;

    // Force requested GPIO 9.
    config.servoPin = THROTTLE_SERVO_PIN;

    return true;
}

// ============================================================
// HANDLE CONFIG
// ============================================================

void handleConfig(JsonDocument& doc)
{
    setState(STATE_CONFIG_RECEIVED);

    if (!parseConfiguration(doc))
    {
        setState(STATE_ERROR);
        return;
    }

    configurationValid = true;

    /*
     * Initialize actual hardware here.
     *
     * Example:
     *
     * hx711_1.begin(
     *     config.loadCell1Data,
     *     config.loadCell1Clock
     * );
     *
     * MAX31855 initialization goes here.
     *
     * RPM interrupt setup goes here.
     */

    sendAck("UPLOAD_CONFIG", true);

    setState(STATE_DAQ_READY);
}

// ============================================================
// HANDLE UDP COMMAND
// ============================================================

void handleCommand(JsonDocument& doc)
{
    const char* command = doc["command"] | "";

      if (command == nullptr || command[0] == '\0')
      {
          command = doc["type"] | "";
      }

      Serial.print("COMMAND: ");
      Serial.println(command);

      // --------------------------------
      // HELLO
      // --------------------------------

      if (
          strcmp(command, "HELLO") == 0 ||
          strcmp(command, "hello") == 0
      )
      {
          sendHello();
          sendStatePacket();

          return;
      }

      // --------------------------------
      // UPLOAD CONFIG
      // --------------------------------

      if (
          strcmp(command, "UPLOAD_CONFIG") == 0 ||
          strcmp(command, "setup_upload") == 0 ||
          strcmp(command, "upload_config") == 0
      )
      {
          handleConfig(doc);
          return;
      }

      // --------------------------------
      // READY
      // --------------------------------

      if (
          strcmp(command, "READY") == 0 ||
          strcmp(command, "ready") == 0
      )
      {
          if (configurationValid)
          {
              setState(STATE_DAQ_READY);
              sendAck("READY", true);
          }
          else
          {
              sendAck("READY", false);
              sendError("Configuration not uploaded");
          }

          return;
      }

      // --------------------------------
      // RUN
      // --------------------------------

      if (
          strcmp(command, "RUN") == 0 ||
          strcmp(command, "run") == 0
      )
      {
          if (!configurationValid)
          {
              sendAck("RUN", false);
              sendError("Configuration not uploaded");
              return;
          }

          setState(STATE_RUNNING);

          sendAck("RUN", true);

          return;
      }

      // --------------------------------
      // STOP
      // --------------------------------

      if (
          strcmp(command, "STOP") == 0 ||
          strcmp(command, "stop") == 0
      )
      {
          setThrottle(0);

          setState(STATE_STOPPED);

          sendAck("STOP", true);

          return;
      }

      // --------------------------------
      // THROTTLE
      // --------------------------------

      if (
          strcmp(command, "SET_THROTTLE") == 0 ||
          strcmp(command, "set_throttle") == 0 ||
          strcmp(command, "throttle") == 0
      )
      {
          float value = doc["value"] | 0.0f;

          setThrottle(value);

          sendAck("SET_THROTTLE", true);

          return;
      }

      // --------------------------------
      // PING
      // --------------------------------

      if (
          strcmp(command, "PING") == 0 ||
          strcmp(command, "ping") == 0
      )

    // --------------------------------
    // UNKNOWN
    // --------------------------------

    sendError("Unknown command");
}

// ============================================================
// RECEIVE UDP
// ============================================================

void receiveUDP()
{
    int packetSize = udp.parsePacket();

    if (packetSize <= 0)
        return;

    guiIP = udp.remoteIP();
    guiPort = udp.remotePort();

      // The dashboard can tell us which UDP port to reply to.
      // This avoids losing replies when the sender socket uses an OS-assigned
      // ephemeral source port instead of a fixed local port.
      if (!doc["reply_port"].isNull())
      {
          guiPort = doc["reply_port"].as<uint16_t>();
      }


    char buffer[2048];

    int len = udp.read(
        buffer,
        sizeof(buffer) - 1
    );

    if (len <= 0)
        return;

    buffer[len] = '\0';

    Serial.print("RX: ");
    Serial.println(buffer);

    StaticJsonDocument<4096> doc;

    DeserializationError error =
        deserializeJson(doc, buffer);

    if (error)
    {
        sendError("Invalid JSON");
        return;
    }

    handleCommand(doc);
}

// ============================================================
// WIFI
// ============================================================

void connectWiFi()
{
    setState(STATE_WIFI_CONNECTING);

    WiFi.mode(WIFI_STA);

    WiFi.begin(
        WIFI_SSID,
        WIFI_PASSWORD
    );

    Serial.print("Connecting to WiFi");

    uint32_t start = millis();

    while (
        WiFi.status() != WL_CONNECTED &&
        millis() - start < 20000
    )
    {
        delay(250);
        Serial.print(".");
    }

    Serial.println();

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WiFi FAILED");

        setState(STATE_ERROR);

        return;
    }

    Serial.println("WiFi connected");

    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
}

// ============================================================
// MDNS
// ============================================================

void startMDNS()
{
    if (!MDNS.begin(MDNS_HOSTNAME))
    {
        Serial.println("mDNS failed");

        return;
    }

    /*
     * The GUI searches for:

         _engine-daq._udp.local
    */

    MDNS.addService(
        "engine-daq",
        "udp",
        UDP_PORT
    );

    Serial.print("mDNS: ");
    Serial.print(MDNS_HOSTNAME);
    Serial.println(".local");
}

// ============================================================
// SENSOR PLACEHOLDER
// ============================================================

void updateSensors()
{
    /*
     * Replace these values with actual sensors.
     *
     * IMPORTANT:
     *
     * Telemetry transmission is 60 Hz.
     *
     * Your high-rate vibration acquisition can remain
     * at 1000 Hz or higher independently.
     */

    // -----------------------------
    // DEMO DATA
    // -----------------------------

    static float demoTime = 0;

    demoTime += 0.01f;

    /*
     * Replace everything below with real sensor data.
     */

    rpm = 6000.0f;

    thrust = throttle;

    fuel = 70.0f;

    cht = 120.0f;

    egt = 650.0f;

    vibrationX = 0.10f;
    vibrationY = 0.08f;
    vibrationZ = 0.12f;

    battery = 12.0f;
}

// ============================================================
// SAFETY
// ============================================================

void checkConnectionTimeout()
{
    if (!guiConnected)
        return;

    if (
        millis() - lastGuiPacketMillis >
        GUI_TIMEOUT_MS
    )
    {
        guiConnected = false;

        /*
         * Safety:
         * GUI disappeared.
         * Return throttle to zero.
         */

        setThrottle(0);

        if (systemState == STATE_RUNNING)
        {
            setState(STATE_STOPPED);
        }
    }
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(500);

    Serial.println();
    Serial.println(
        "========================================"
    );
    Serial.println(
        " ENGINE TEST BENCH DAQ - ESP32"
    );
    Serial.println(
        "========================================"
    );

    // -----------------------------
    // Servo
    // -----------------------------

    throttleServo.setPeriodHertz(50);

    throttleServo.attach(
        THROTTLE_SERVO_PIN,
        SERVO_MIN_US,
        SERVO_MAX_US
    );

    setThrottle(0);

    // -----------------------------
    // WiFi
    // -----------------------------

    connectWiFi();

    if (WiFi.status() != WL_CONNECTED)
        return;

    // -----------------------------
    // mDNS
    // -----------------------------

    startMDNS();

    // -----------------------------
    // UDP
    // -----------------------------

    udp.begin(UDP_PORT);

    Serial.print("UDP listening on ");
    Serial.println(UDP_PORT);

    // -----------------------------
    // Initial state
    // -----------------------------

    configurationValid = false;

    setState(STATE_READY);

    Serial.println("ESP32 READY");
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    // -----------------------------
    // Receive commands
    // -----------------------------

    receiveUDP();

    // -----------------------------
    // Sensor acquisition
    // -----------------------------

    updateSensors();

    // -----------------------------
    // 60 Hz telemetry
    // -----------------------------

    uint32_t now = micros();

    if (
        now - lastTelemetryMicros >=
        TELEMETRY_INTERVAL_US
    )
    {
        lastTelemetryMicros = now;

        sendTelemetry();
    }

    // -----------------------------
    // Safety
    // -----------------------------

    checkConnectionTimeout();

    delay(1);
}