#include <Arduino.h>

// Define analog input pins for H2 (MQ8) and O3 (MQ131) sensors
#define H2_SENSOR_PIN 36   // GPIO36 for H2 sensor (MQ8)
#define O3_SENSOR_PIN 39   // GPIO39 for O3 sensor (MQ131)

#define H2_SENSOR_VOLTAGE_REF 3.27 // Reference voltage for H2 sensor
#define O3_SENSOR_VOLTAGE_REF 3.3  // Reference voltage for O3 sensor
#define H2_SENSOR_VOLTAGE_RESOLUTION 4095.0 // 12-bit ADC resolution (0–4095)
#define O3_SENSOR_VOLTAGE_RESOLUTION 4095.0 // 12-bit ADC resolution (0–4095)

#define SERIAL_NUMBER_RESOLUTION // Number of decimal places for voltage output
void setup() {
  Serial.begin(9600);         
  analogReadResolution(12);   // 12-bit ADC resolution (0–4095)
}

void loop() {
  // Read raw ADC values
  int h2Raw = analogRead(H2_SENSOR_PIN);
  int o3Raw = analogRead(O3_SENSOR_PIN);

  // Convert to voltage (3.0V reference)
  float h2Voltage = h2Raw * (H2_SENSOR_VOLTAGE_REF / H2_SENSOR_VOLTAGE_RESOLUTION);
  float o3Voltage = o3Raw * (O3_SENSOR_VOLTAGE_REF / O3_SENSOR_VOLTAGE_RESOLUTION);

  // Send formatted data over serial
  //Serial.print("H2_RAW: ");
  Serial.print(h2Raw);
  Serial.print(",");
  //Serial.print("  H2_V: ");
  Serial.print(h2Voltage, SERIAL_NUMBER_RESOLUTION);
  Serial.print(",");
  //Serial.print("  |  O3_RAW: ");
  Serial.print(o3Raw);
  Serial.print(",");
  //Serial.print("  O3_V: ");
  Serial.println(o3Voltage, SERIAL_NUMBER_RESOLUTION);


  delay(1000);  // 1 second delay