#include <Wire.h>
#include <ZMPT101B.h>
#include <LiquidCrystal.h>

#define SENSITIVITY 500.0f
#define CALIBRATION_FACTOR 1.52

#define OVER_VOLTAGE 240.0
#define UNDER_VOLTAGE 210.0

#define RELAY_PIN 4

LiquidCrystal lcd(19, 23, 18, 17, 16, 15);
ZMPT101B voltageSensor(27, 50.0);

void setup() {
  lcd.begin(16, 2);
  lcd.print("Display Working...");
  Serial.begin(115200);
  voltageSensor.setSensitivity(SENSITIVITY);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // Start OFF (relay OFF, device OFF)
}

void sensor() {
  float voltage = voltageSensor.getRmsVoltage() * CALIBRATION_FACTOR;
  if (voltage < 50) voltage = 0;

  Serial.print("Voltage: ");
  Serial.println(voltage);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Voltage: ");
  lcd.print(voltage, 1);

  lcd.setCursor(0, 1);
  if (voltage == 0) {
    lcd.print("No AC Voltage   ");
    digitalWrite(RELAY_PIN, HIGH); // Relay OFF (trip)
  } else if (voltage < UNDER_VOLTAGE) {
    lcd.print("Under Voltage   ");
    digitalWrite(RELAY_PIN, HIGH); // Relay OFF (trip)
  } else if (voltage > OVER_VOLTAGE) {
    lcd.print("Over Voltage    ");
    digitalWrite(RELAY_PIN, HIGH); // Relay OFF (trip)
  } else {
    lcd.print("Normal Voltage  ");
    digitalWrite(RELAY_PIN, LOW);  // Relay ON (device ON)
  }
}

void loop() {
  sensor();
  delay(1000);
}
