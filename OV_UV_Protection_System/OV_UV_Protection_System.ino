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


// #include <WiFi.h>
// #include "RMaker.h"
// #include "WiFiProv.h"
// #include <ZMPT101B.h>
// #include <LiquidCrystal.h>

// #define SENSITIVITY 500.0f
// #define CALIBRATION_FACTOR 1.52
// #define OVER_VOLTAGE 240.0
// #define UNDER_VOLTAGE 210.0
// #define RELAY_PIN 4

// const char *service_name = "VoltageMonitor";
// const char *pop = "12345678"; // Proof of possession

// LiquidCrystal lcd(19, 23, 18, 17, 16, 15);
// ZMPT101B voltageSensor(27, 50.0);

// // Create Switch device
// Switch my_switch("Relay", NULL, false);
// // Add a custom voltage parameter (read-only)
// Param voltage_param("Voltage", "esp.param.voltage", value((float)0.0), PROP_FLAG_READ);

// void sysProvEvent(arduino_event_t *sys_event) {
//     switch(sys_event->event_id) {
//         case ARDUINO_EVENT_PROV_START:
//             Serial.println("Provisioning started");
//             printQR(service_name, pop, "ble");
//             break;
//         default:
//             break;
//     }
// }

// // Relay control callback
// void relayWriteCallback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx) {
//     bool relayState = val.val.b;
//     digitalWrite(RELAY_PIN, relayState ? LOW : HIGH); // Active LOW relay
//     param->updateAndReport(val);
// }

// void setup() {
//     Serial.begin(115200);
//     lcd.begin(16, 2);
//     lcd.print("Display Working...");
//     voltageSensor.setSensitivity(SENSITIVITY);

//     pinMode(RELAY_PIN, OUTPUT);
//     digitalWrite(RELAY_PIN, HIGH); // OFF (active LOW)

//     my_switch.addCb(relayWriteCallback);
//     my_switch.addParam(voltage_param);

//     // Correct: create a node and add device to node, then initialize RMaker with that node
//     static Node my_node; // static to avoid stack issues
//     my_node = RMaker.initNode("Voltage Monitor Node");
//     my_node.addDevice(my_switch);

//     RMaker.enableOTA(OTA_USING_PARAMS);
//     RMaker.start();

//     WiFi.onEvent(sysProvEvent);
//     WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SECURITY_1, service_name, pop);
// }

// void loop() {
//     float voltage = voltageSensor.getRmsVoltage() * CALIBRATION_FACTOR;
//     if (voltage < 50) voltage = 0;

//     lcd.clear();
//     lcd.setCursor(0, 0);
//     lcd.print("Voltage: ");
//     lcd.print(voltage, 1);

//     bool relayState;
//     lcd.setCursor(0, 1);
//     if (voltage == 0) {
//         lcd.print("No AC Voltage   ");
//         digitalWrite(RELAY_PIN, HIGH); relayState = false;
//     } else if (voltage < UNDER_VOLTAGE) {
//         lcd.print("Under Voltage   ");
//         digitalWrite(RELAY_PIN, HIGH); relayState = false;
//     } else if (voltage > OVER_VOLTAGE) {
//         lcd.print("Over Voltage    ");
//         digitalWrite(RELAY_PIN, HIGH); relayState = false;
//     } else {
//         lcd.print("Normal Voltage  ");
//         digitalWrite(RELAY_PIN, LOW);  relayState = true;
//     }

//     // Report voltage and relay state to ESP RainMaker
//     voltage_param.updateAndReport(value((float)voltage));
//     my_switch.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, relayState);

//     delay(1000);
// }



#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <Wire.h>
#include <ZMPT101B.h>
#include <LiquidCrystal.h>

#define SENSITIVITY 500.0f
#define CALIBRATION_FACTOR 1.52

#define OVER_VOLTAGE 240.0
#define UNDER_VOLTAGE 210.0

#define RELAY_PIN 4
#define AC_SENSOR_PIN 27
#define RESET_BUTTON_PIN 0
#define WIFI_LED_PIN 2

LiquidCrystal lcd(19, 23, 18, 17, 16, 15);
ZMPT101B voltageSensor(AC_SENSOR_PIN, 50.0);

char deviceName[] = "MainRelay";
static uint8_t relayPin = RELAY_PIN;
static Switch main_relay(deviceName, &relayPin);

// ---------- NEW: Custom RainMaker Params ----------
static Param voltage_param("Voltage", "esp.param.voltage", value(0.0f), PROP_FLAG_READ | PROP_FLAG_PERSIST);
static Param status_param("Voltage Status", "esp.param.status", value((char*)"Unknown"), PROP_FLAG_READ | PROP_FLAG_PERSIST);
// --------------------------------------------------

bool toggleState = LOW;
bool manualOverride = false;
bool g_provisioned = false;

unsigned long previousReconnectAttempt = 0;
const unsigned long reconnectInterval = 10000;
const char *service_name = "PROV_VOLT";
const char *pop = "1234567";

float lastVoltage = 0.0;
String lastStatus = "Init";

void sysProvEvent(arduino_event_t *sys_event) {
    // ... (unchanged)
}

void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx) {
    // ... (unchanged)
}

void manual_control() {
    // ... (unchanged)
}

void check_voltage_and_control() {
    float voltage = voltageSensor.getRmsVoltage() * CALIBRATION_FACTOR;
    if (voltage < 50) voltage = 0;

    String status;
    bool relayShouldBe = LOW;
    if (voltage == 0) {
        status = "No AC Voltage";
        relayShouldBe = HIGH;
    } else if (voltage < UNDER_VOLTAGE) {
        status = "Under Voltage";
        relayShouldBe = HIGH;
    } else if (voltage > OVER_VOLTAGE) {
        status = "Over Voltage";
        relayShouldBe = HIGH;
    } else {
        status = "Normal Voltage";
        relayShouldBe = LOW;
    }
    if (!manualOverride) {
        if (toggleState != relayShouldBe) {
            toggleState = relayShouldBe;
            digitalWrite(relayPin, toggleState);
            main_relay.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState == LOW);
        }
    }
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Voltage: ");
    lcd.print(voltage, 1);
    lcd.setCursor(0, 1);
    lcd.print(status);

    // ----------- UPDATE RAINMAKER PARAMETERS -----------
    voltage_param.updateAndReport(value(voltage));
    status_param.updateAndReport(value(status.c_str()));
    // ---------------------------------------------------

    Serial.print("Voltage: ");
    Serial.print(voltage, 1);
    Serial.print("V  | Status: ");
    Serial.println(status);

    lastVoltage = voltage;
    lastStatus = status;
}

void setup() {
    Serial.begin(115200);
    WiFi.disconnect(true, true);
    delay(100);
    pinMode(relayPin, OUTPUT);
    pinMode(WIFI_LED_PIN, OUTPUT);
    pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
    digitalWrite(relayPin, HIGH);
    digitalWrite(WIFI_LED_PIN, LOW);

    lcd.begin(16, 2);
    lcd.print("Display Working...");
    voltageSensor.setSensitivity(SENSITIVITY);

    Node my_node = RMaker.initNode("ESP32_Voltage_Protect");
    main_relay.addCb(write_callback);
    my_node.addDevice(main_relay);

    // ------------- ADD CUSTOM PARAMETERS TO NODE -------------
    main_relay.addParam(voltage_param);
    main_relay.addParam(status_param);
    // ---------------------------------------------------------

    RMaker.enableOTA(OTA_USING_PARAMS);
    RMaker.enableTZService();
    RMaker.enableSchedule();

    Serial.printf("\nChip ID:  %u Service Name: %s\n", ESP.getChipRevision(), service_name);
    Serial.printf("\nStarting ESP-RainMaker\n");
    RMaker.start();

    WiFi.onEvent(sysProvEvent);

    if (!g_provisioned) {
    #if CONFIG_IDF_TARGET_ESP32
        WiFiProv.beginProvision(
            WIFI_PROV_SCHEME_BLE,
            WIFI_PROV_SCHEME_HANDLER_FREE_BTDM,
            WIFI_PROV_SECURITY_1,
            pop,
            service_name
        );
    #else
        WiFiProv.beginProvision(
            WIFI_PROV_SCHEME_SOFTAP,
            WIFI_PROV_SCHEME_HANDLER_NONE,
            WIFI_PROV_SECURITY_1,
            pop,
            service_name
        );
    #endif
    }
    main_relay.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
}

void loop()
{
    manual_control();
    check_voltage_and_control();

    // Reset/provisioning logic (matches template)
    if (digitalRead(RESET_BUTTON_PIN) == LOW) {
        Serial.printf("Reset Button Pressed!\n");
        delay(100);
        int startTime = millis();
        while (digitalRead(RESET_BUTTON_PIN) == LOW) delay(50);
        int endTime = millis();

        if ((endTime - startTime) > 10000) {
            Serial.printf("Reset to factory.\n");
            RMakerFactoryReset(2);
        } else if ((endTime - startTime) > 3000) {
            Serial.printf("Reset Wi-Fi.\n");
            RMakerWiFiReset(2);
        }
    }

    delay(1000);

    static unsigned long lastPrint = 0;
    unsigned long currentMillis = millis();

    if (g_provisioned) {
        if (WiFi.status() != WL_CONNECTED) {
            digitalWrite(WIFI_LED_PIN, LOW);
            if (currentMillis - previousReconnectAttempt >= reconnectInterval) {
                Serial.println("WiFi disconnected. Trying to reconnect...");
                WiFi.begin();
                previousReconnectAttempt = currentMillis;
            }
        } else {
            digitalWrite(WIFI_LED_PIN, HIGH);
            if (currentMillis - lastPrint >= 30000) {
                Serial.println("WiFi still connected.");
                lastPrint = currentMillis;
            }
        }
    } else {
        Serial.println("Device not provisioned. Waiting for provisioning...");
        digitalWrite(WIFI_LED_PIN, LOW);
    }
}


