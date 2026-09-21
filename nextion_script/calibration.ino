#include "HX711.h"

// ======================================================
// 1 KG LOAD CELL
// ======================================================

#define HX_DOUT 4
#define HX_SCK  5

HX711 scale;


// ======================================================
// SETTINGS
// ======================================================

const int TARE_SAMPLES = 30;
const int CAL_SAMPLES  = 30;


// ======================================================
// SETUP
// ======================================================

void setup()
{
  Serial.begin(115200);

  delay(1500);

  Serial.println();
  Serial.println("==============================================");
  Serial.println("        HX711 CALIBRATION - 1 KG");
  Serial.println("==============================================");

  Serial.println("DATA = Teensy Pin 4");
  Serial.println("CLK  = Teensy Pin 5");

  scale.begin(HX_DOUT, HX_SCK);

  delay(500);

  if (!scale.is_ready())
  {
    Serial.println();
    Serial.println("ERROR: HX711 NOT READY!");
    Serial.println("Check VCC, VDD, GND, DATA and CLK.");

    while (1);
  }

  Serial.println();
  Serial.println("HX711 READY!");

  // ----------------------------------------------------
  // TARE
  // ----------------------------------------------------

  Serial.println();
  Serial.println("REMOVE ALL WEIGHT.");
  Serial.println("Do not touch the load cell.");
  Serial.println("Taring in 5 seconds...");

  delay(5000);

  scale.set_scale(1.0);

  scale.tare(TARE_SAMPLES);

  Serial.println();
  Serial.println("TARE COMPLETE!");

  Serial.println();
  Serial.println("Now place a KNOWN weight on the");
  Serial.println("1 kg load cell.");

  Serial.println();
  Serial.println("Recommended:");
  Serial.println("100 g");
  Serial.println("200 g");
  Serial.println("500 g");
  Serial.println("1000 g = 1 kg");

  Serial.println();
  Serial.println("Enter the weight in GRAMS");
  Serial.println("Example: 187");
  Serial.println();
}


// ======================================================
// LOOP
// ======================================================

void loop()
{
  if (Serial.available())
  {
    float knownWeight = Serial.parseFloat();

    while (Serial.available())
      Serial.read();

    if (knownWeight <= 0)
    {
      Serial.println("Invalid weight!");
      return;
    }

    Serial.println();
    Serial.println("Known weight:");
    Serial.print(knownWeight, 2);
    Serial.println(" g");

    Serial.println();
    Serial.println("Allowing load cell to settle...");

    delay(2000);

    // --------------------------------------------------
    // READ RAW VALUE
    // --------------------------------------------------

    Serial.println("Reading ADC...");

    long rawValue = scale.get_value(CAL_SAMPLES);

    Serial.print("Raw ADC value = ");
    Serial.println(rawValue);

    // --------------------------------------------------
    // CALCULATE CALIBRATION FACTOR
    // --------------------------------------------------

    float calibrationFactor =
      (float)rawValue / knownWeight;

    Serial.println();
    Serial.println("==============================================");
    Serial.println("             CALIBRATION RESULT");
    Serial.println("==============================================");

    Serial.print("Known weight       = ");
    Serial.print(knownWeight, 2);
    Serial.println(" g");

    Serial.print("Raw ADC            = ");
    Serial.println(rawValue);

    Serial.print("Calibration factor = ");
    Serial.println(calibrationFactor, 6);

    Serial.println();
    Serial.println("Use this in your main DAQ code:");

    Serial.print("calibrationFactor1kg = ");
    Serial.print(calibrationFactor, 6);
    Serial.println(";");

    // --------------------------------------------------
    // VERIFY
    // --------------------------------------------------

    scale.set_scale(calibrationFactor);

    Serial.println();
    Serial.println("==============================================");
    Serial.println("                 VERIFICATION");
    Serial.println("==============================================");

    Serial.println("Keep the same known weight on the load cell.");
    Serial.println();

    for (int i = 0; i < 15; i++)
    {
      float weight_g = scale.get_units(10);

      Serial.print("Weight = ");
      Serial.print(weight_g, 2);
      Serial.println(" g");

      delay(500);
    }

    Serial.println();
    Serial.println("==============================================");
    Serial.println("CALIBRATION FINISHED");
    Serial.println("==============================================");

    while (1);
  }
}