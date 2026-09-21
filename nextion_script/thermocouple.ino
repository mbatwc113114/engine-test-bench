#include <SPI.h>
#include <Adafruit_MAX31856.h>

// ======================================================
// MAX31856 CHIP SELECT PINS
// ======================================================

#define MAX31856_1_CS 10
#define MAX31856_2_CS 9

// ======================================================
// MAX31856 OBJECTS
//
// Hardware SPI:
// SCK  = Teensy 4.1 pin 13
// MISO = Teensy 4.1 pin 12
// MOSI = Teensy 4.1 pin 11
// ======================================================

Adafruit_MAX31856 thermocouple1(MAX31856_1_CS);
Adafruit_MAX31856 thermocouple2(MAX31856_2_CS);


// ======================================================
// SETUP
// ======================================================

void setup() {

  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println("======================================");
  Serial.println("Teensy 4.1 - 2x MAX31856");
  Serial.println("K-Type Thermocouple Test");
  Serial.println("======================================");

  // ----------------------------------------------------
  // Initialize MAX31856 #1
  // ----------------------------------------------------

  Serial.println();
  Serial.println("Initializing MAX31856 #1...");

  if (!thermocouple1.begin()) {

    Serial.println("ERROR: MAX31856 #1 NOT FOUND!");

  } else {

    Serial.println("MAX31856 #1 OK");

    // Set K-type
    thermocouple1.setThermocoupleType(MAX31856_TCTYPE_K);

    Serial.println("MAX31856 #1 = K-Type");
  }


  // ----------------------------------------------------
  // Initialize MAX31856 #2
  // ----------------------------------------------------

  Serial.println();
  Serial.println("Initializing MAX31856 #2...");

  if (!thermocouple2.begin()) {

    Serial.println("ERROR: MAX31856 #2 NOT FOUND!");

  } else {

    Serial.println("MAX31856 #2 OK");

    // Set K-type
    thermocouple2.setThermocoupleType(MAX31856_TCTYPE_K);

    Serial.println("MAX31856 #2 = K-Type");
  }


  Serial.println();
  Serial.println("Starting measurements...");
  Serial.println();
}


// ======================================================
// PRINT FAULT FOR SENSOR 1
// ======================================================

void printFault1(uint8_t fault) {

  if (fault == 0) {
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
// PRINT FAULT FOR SENSOR 2
// ======================================================

void printFault2(uint8_t fault) {

  if (fault == 0) {
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
// LOOP
// ======================================================

void loop() {

  // ====================================================
  // SENSOR 1
  // ====================================================

  float temp1 = thermocouple1.readThermocoupleTemperature();

  float cj1 = thermocouple1.readCJTemperature();

  uint8_t fault1 = thermocouple1.readFault();


  // ====================================================
  // SENSOR 2
  // ====================================================

  float temp2 = thermocouple2.readThermocoupleTemperature();

  float cj2 = thermocouple2.readCJTemperature();

  uint8_t fault2 = thermocouple2.readFault();


  // ====================================================
  // PRINT SENSOR 1
  // ====================================================

  Serial.println("--------------------------------------");

  Serial.println("MAX31856 #1");

  Serial.print("Thermocouple: ");
  Serial.print(temp1, 2);
  Serial.println(" °C");

  Serial.print("Cold Junction: ");
  Serial.print(cj1, 2);
  Serial.println(" °C");

  printFault1(fault1);


  // ====================================================
  // PRINT SENSOR 2
  // ====================================================

  Serial.println();

  Serial.println("MAX31856 #2");

  Serial.print("Thermocouple: ");
  Serial.print(temp2, 2);
  Serial.println(" °C");

  Serial.print("Cold Junction: ");
  Serial.print(cj2, 2);
  Serial.println(" °C");

  printFault2(fault2);


  // ====================================================
  // WAIT
  // ====================================================

  Serial.println();

  delay(500);
}