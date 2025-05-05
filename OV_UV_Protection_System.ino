// #include <Wire.h>
// #include <ZMPT101B.h>
// #include <LiquidCrystal.h>

// #define SENSITIVITY 500.0f
// #define CALIBRATION_FACTOR 1.52

// #define OVER_VOLTAGE 240.0
// #define UNDER_VOLTAGE 210.0

// #define RELAY_PIN 4

// LiquidCrystal lcd(19, 23, 18, 17, 16, 15);
// ZMPT101B voltageSensor(27, 50.0);

// void setup() {
//   lcd.begin(16, 2);
//   lcd.print("Display Working...");
//   Serial.begin(115200);
//   voltageSensor.setSensitivity(SENSITIVITY);

//   pinMode(RELAY_PIN, OUTPUT);
//   digitalWrite(RELAY_PIN, HIGH); // Start OFF (relay OFF, device OFF)
// }

// void sensor() {
//   float voltage = voltageSensor.getRmsVoltage() * CALIBRATION_FACTOR;
//   if (voltage < 50) voltage = 0;

//   Serial.print("Voltage: ");
//   Serial.println(voltage);

//   lcd.clear();
//   lcd.setCursor(0, 0);
//   lcd.print("Voltage: ");
//   lcd.print(voltage, 1);

//   lcd.setCursor(0, 1);
//   if (voltage == 0) {
//     lcd.print("No AC Voltage   ");
//     digitalWrite(RELAY_PIN, HIGH); // Relay OFF (trip)
//   } else if (voltage < UNDER_VOLTAGE) {
//     lcd.print("Under Voltage   ");
//     digitalWrite(RELAY_PIN, HIGH); // Relay OFF (trip)
//   } else if (voltage > OVER_VOLTAGE) {
//     lcd.print("Over Voltage    ");
//     digitalWrite(RELAY_PIN, HIGH); // Relay OFF (trip)
//   } else {
//     lcd.print("Normal Voltage  ");
//     digitalWrite(RELAY_PIN, LOW);  // Relay ON (device ON)
//   }
// }

// void loop() {
//   sensor();
//   delay(1000);
// }


#include <WiFi.h>
#include "RMaker.h"
#include "WiFiProv.h"
#include <ZMPT101B.h>
#include <LiquidCrystal.h>

#define SENSITIVITY 500.0f
#define CALIBRATION_FACTOR 1.52
#define OVER_VOLTAGE 240.0
#define UNDER_VOLTAGE 210.0
#define RELAY_PIN 4

const char *service_name = "VoltageMonitor";
const char *pop = "12345678"; // Proof of possession

LiquidCrystal lcd(19, 23, 18, 17, 16, 15);
ZMPT101B voltageSensor(27, 50.0);

// Create Switch device
Switch my_switch("Relay", NULL, false);
// Add a custom voltage parameter (read-only)
Param voltage_param("Voltage", "esp.param.voltage", value((float)0.0), PROP_FLAG_READ);

void sysProvEvent(arduino_event_t *sys_event) {
    switch(sys_event->event_id) {
        case ARDUINO_EVENT_PROV_START:
            Serial.println("Provisioning started");
            printQR(service_name, pop, "ble");
            break;
        default:
            break;
    }
}

// Relay control callback
void relayWriteCallback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx) {
    bool relayState = val.val.b;
    digitalWrite(RELAY_PIN, relayState ? LOW : HIGH); // Active LOW relay
    param->updateAndReport(val);
}

void setup() {
    Serial.begin(115200);
    lcd.begin(16, 2);
    lcd.print("Display Working...");
    voltageSensor.setSensitivity(SENSITIVITY);

    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, HIGH); // OFF (active LOW)

    my_switch.addCb(relayWriteCallback);
    my_switch.addParam(voltage_param);

    // Correct: create a node and add device to node, then initialize RMaker with that node
    static Node my_node; // static to avoid stack issues
    my_node = RMaker.initNode("Voltage Monitor Node");
    my_node.addDevice(my_switch);

    RMaker.enableOTA(OTA_USING_PARAMS);
    RMaker.start();

    WiFi.onEvent(sysProvEvent);
    WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SECURITY_1, service_name, pop);
}

void loop() {
    float voltage = voltageSensor.getRmsVoltage() * CALIBRATION_FACTOR;
    if (voltage < 50) voltage = 0;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Voltage: ");
    lcd.print(voltage, 1);

    bool relayState;
    lcd.setCursor(0, 1);
    if (voltage == 0) {
        lcd.print("No AC Voltage   ");
        digitalWrite(RELAY_PIN, HIGH); relayState = false;
    } else if (voltage < UNDER_VOLTAGE) {
        lcd.print("Under Voltage   ");
        digitalWrite(RELAY_PIN, HIGH); relayState = false;
    } else if (voltage > OVER_VOLTAGE) {
        lcd.print("Over Voltage    ");
        digitalWrite(RELAY_PIN, HIGH); relayState = false;
    } else {
        lcd.print("Normal Voltage  ");
        digitalWrite(RELAY_PIN, LOW);  relayState = true;
    }

    // Report voltage and relay state to ESP RainMaker
    voltage_param.updateAndReport(value((float)voltage));
    my_switch.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, relayState);

    delay(1000);
}





