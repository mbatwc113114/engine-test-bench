/*
  Teensy 4.1 <-> Nextion transparent bridge
  ------------------------------------------------------------
  UART   : Serial6  (Teensy 4.1: RX6 = pin 25, TX6 = pin 24)
  Baud   : 921600, 8N1
  Wiring : Nextion TX -> Teensy pin 25 (RX6)
           Nextion RX -> Teensy pin 24 (TX6)
           Nextion GND -> Teensy GND
           Nextion +5V -> external 5V (NOT the Teensy 3.3V pin)

  USB Serial -> Nextion:
    "rpm.val=1000\n"           -> sends "rpm.val=1000" + FF FF FF
    "rpm.val=1000" FF FF FF    -> sent as-is (no duplicate terminator)

  Nextion -> USB Serial:
    default = HEX mode (readable in Serial Monitor)
    "#raw"  = transparent binary passthrough (use for Python)
    "#hex"  = readable hex/decoded packets

  Local commands (not forwarded):
    #raw   #hex   #test   #ping
*/

#include <Arduino.h>

// ---------------- Configuration ----------------
#define NEXTION_SERIAL        Serial6
#define NEXTION_BAUD          921600
#define NEXTION_RX_PIN        25      // Teensy RX6 (Nextion TX goes here)
#define NEXTION_TX_PIN        24      // Teensy TX6 (Nextion RX goes here)
#define NEXTION_DEBUG         1       // 1 = print debug (only when not in raw mode)
#define NEXTION_STARTUP_TEST  1       // send rpm.val=1000 once at startup
#define USB_BAUD              115200  // ignored by Teensy USB, kept for reference

// ---------------- Buffers ----------------
static uint8_t nxRxExtra[2048];
static uint8_t nxTxExtra[1024];

static uint8_t  cmdBuf[256];
static size_t   cmdLen  = 0;
static uint8_t  ffCount = 0;

static uint8_t  pktBuf[128];
static size_t   pktLen = 0;
static uint8_t  pktFF  = 0;

static bool rawMode = false;   // false = readable hex, true = transparent binary

// ---------------- Helpers ----------------
static inline void nxTerminator() {
  NEXTION_SERIAL.write((uint8_t)0xFF);
  NEXTION_SERIAL.write((uint8_t)0xFF);
  NEXTION_SERIAL.write((uint8_t)0xFF);
}

void nextionSendCommand(const char *cmd) {
  NEXTION_SERIAL.write((const uint8_t *)cmd, strlen(cmd));
  nxTerminator();
#if NEXTION_DEBUG
  if (!rawMode) {
    Serial.print("[NEXTION TX] ");
    Serial.println(cmd);
  }
#endif
}

static void nextionSendBinaryAsIs(const uint8_t *buf, size_t len) {
  NEXTION_SERIAL.write(buf, len);
#if NEXTION_DEBUG
  if (!rawMode) {
    Serial.print("[NEXTION TX] (already terminated) ");
    Serial.write(buf, len >= 3 ? len - 3 : len);
    Serial.println();
  }
#endif
}

static void printHexByte(uint8_t b) {
  if (b < 16) Serial.print('0');
  Serial.print(b, HEX);
  Serial.print(' ');
}

static void printPacket() {
  Serial.print("[NEXTION RX] ");
  for (size_t i = 0; i < pktLen; i++) printHexByte(pktBuf[i]);

  if (pktLen >= 1) {
    uint8_t id = pktBuf[0];
    if (id == 0x71 && pktLen >= 8) {
      uint32_t v = (uint32_t)pktBuf[1] | ((uint32_t)pktBuf[2] << 8) |
                   ((uint32_t)pktBuf[3] << 16) | ((uint32_t)pktBuf[4] << 24);
      Serial.print(" | number = ");
      Serial.print((int32_t)v);
    } else if (id == 0x70) {
      Serial.print(" | string = ");
      for (size_t i = 1; i + 3 < pktLen; i++) Serial.write(pktBuf[i]);
    } else if (id == 0x66 && pktLen >= 5) {
      Serial.print(" | current page = ");
      Serial.print(pktBuf[1]);
    } else if (id == 0x88) {
      Serial.print(" | Nextion ready");
    } else if (id == 0x00) {
      Serial.print(" | invalid instruction / startup");
    }
  }
  Serial.println();
}

static void handleLocalCommand(const char *c) {
  if (strcmp(c, "#raw") == 0) {
    rawMode = true;   // no more text output
  } else if (strcmp(c, "#hex") == 0) {
    rawMode = false;
    Serial.println("[NEXTION] HEX mode");
  } else if (strcmp(c, "#ping") == 0) {
    nextionSendCommand("sendme");   // expect 66 xx FF FF FF
  } else if (strcmp(c, "#test") == 0) {
    nextionSendCommand("rpm.val=1000");
    nextionSendCommand("rpm.val=1500");
    nextionSendCommand("rpm.val=2000");
    nextionSendCommand("rpm.val=2500");
  } else {
    if (!rawMode) Serial.println("[NEXTION] unknown local command");
  }
}

// ---------------- USB -> Nextion ----------------
static void usbToNextion() {
  while (Serial.available()) {
    uint8_t b = (uint8_t)Serial.read();

    if (b == '\r' || b == '\n') {
      if (cmdLen > 0 && ffCount == 0) {
        cmdBuf[cmdLen] = 0;
        if (cmdBuf[0] == '#') {
          handleLocalCommand((const char *)cmdBuf);
        } else {
          nextionSendCommand((const char *)cmdBuf);  // appends FF FF FF
        }
      }
      cmdLen = 0;
      ffCount = 0;
      continue;
    }

    if (cmdLen >= sizeof(cmdBuf) - 1) {   // overflow protection
      cmdLen = 0;
      ffCount = 0;
    }
    cmdBuf[cmdLen++] = b;

    if (b == 0xFF) {
      ffCount++;
      if (ffCount >= 3) {
        // Already terminated by sender -> forward as-is, no duplicate
        nextionSendBinaryAsIs(cmdBuf, cmdLen);
        cmdLen = 0;
        ffCount = 0;
      }
    } else {
      ffCount = 0;
    }
  }
}

// ---------------- Nextion -> USB ----------------
static void nextionToUsb() {
  uint8_t tmp[64];
  int avail;
  while ((avail = NEXTION_SERIAL.available()) > 0) {
    int n = avail > (int)sizeof(tmp) ? (int)sizeof(tmp) : avail;
    for (int i = 0; i < n; i++) tmp[i] = (uint8_t)NEXTION_SERIAL.read();

    if (rawMode) {
      Serial.write(tmp, n);           // binary-safe passthrough
    } else {
      for (int i = 0; i < n; i++) {
        if (pktLen < sizeof(pktBuf)) pktBuf[pktLen++] = tmp[i];
        if (tmp[i] == 0xFF) {
          if (++pktFF >= 3) {
            printPacket();
            pktLen = 0;
            pktFF = 0;
          }
        } else {
          pktFF = 0;
        }
        if (pktLen >= sizeof(pktBuf)) {   // safety
          printPacket();
          pktLen = 0;
          pktFF = 0;
        }
      }
    }
  }
}

// ---------------- Public init/poll (for merging into DAQ firmware) ----------------
void nextionInit() {
  NEXTION_SERIAL.addMemoryForRead(nxRxExtra, sizeof(nxRxExtra));
  NEXTION_SERIAL.addMemoryForWrite(nxTxExtra, sizeof(nxTxExtra));
  NEXTION_SERIAL.begin(NEXTION_BAUD, SERIAL_8N1);   // Serial6 = RX 25 / TX 24 on Teensy 4.1
#if NEXTION_DEBUG
  Serial.println("[NEXTION] UART initialized (Serial6)");
  Serial.print("[NEXTION] Baud = ");
  Serial.println(NEXTION_BAUD);
  Serial.print("[NEXTION] RX = ");
  Serial.println(NEXTION_RX_PIN);
  Serial.print("[NEXTION] TX = ");
  Serial.println(NEXTION_TX_PIN);
#endif
}

void nextionPoll() {
  usbToNextion();
  nextionToUsb();
}

// ---------------- Arduino ----------------
void setup() {
  Serial.begin(USB_BAUD);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) { }

  nextionInit();

#if NEXTION_STARTUP_TEST
  delay(300);                        // let Nextion finish its own startup
  nextionSendCommand("rpm.val=1000");
#endif
  Serial.println("[NEXTION] Bridge ready. Type e.g. rpm.val=2500 (line ending: Newline)");
}

void loop() {
  nextionPoll();
}
