# Over Voltage Under Voltage Protection System using ESP32

This project implements an Over Voltage Under Voltage Protection System using an ESP32, a ZMPT101B voltage sensor, and a 16x2 LCD display. The system continuously monitors the AC supply voltage, detects over-voltage and under-voltage conditions, and trips (disconnects) the device in unsafe conditions. The voltage and system status are displayed in real-time on the LCD.

---

## ⭐ Features

- **Real-Time Voltage Monitoring:** Continuously reads AC voltage using the ZMPT101B sensor.
- **Automatic Protection:** Trips (turns off) the relay when voltage is unsafe (over or under thresholds).
- **User Feedback:** Displays measured voltage and status (Normal, Over Voltage, Under Voltage, No AC) on a 16x2 LCD.
- **Serial Output:** Prints voltage readings and status to the Serial Monitor for debugging or logging.
- **Simple Circuit:** Built with easily available components and minimal wiring.

---

## 🛠️ Devices Used

- **ESP32 Development Board**
- **ZMPT101B AC Voltage Sensor**
- **16x2 LCD Display(Without I2C) (parallel interface)**
- **Relay Module (for tripping the device)**
- **AC Load (e.g., lamp, fan, etc.)**
- **Wires and Breadboard/PCB**

---

## 🧰 Technologies and Libraries Used

- [Arduino](https://www.arduino.cc/) (ESP32 Board Support)
- [ZMPT101B Library](https://github.com/Abdurraziq/ZMPT101B-arduino.git) for voltage sensing
- [LiquidCrystal Library](https://github.com/arduino-libraries/LiquidCrystal.git) for LCD display

---

## 🚀 Getting Started

1. **Hardware Setup:** Connect the ZMPT101B sensor, LCD, and relay to your ESP32 as per the pin definitions in the code.
2. **Install Required Libraries:**  
   - `ZMPT101B` (install via Library Manager or download from GitHub)
   - `LiquidCrystal` (install via Library Manager)

3. **Upload the Code:**  
   Use the Arduino IDE to upload [`main.ino`](OV_UV_Protection_System) to your ESP32.

---

## 📹 Demo Video

*Check out the demo video below to see the system in action!*


[Demo Video](demo/IMG_1395.MOV)


---

## 📄 Source Code

See [`main.ino`](OV_UV_Protection_System.ino) for the complete Arduino code.

---

## 🛠️ Troubleshooting

### 1. LCD Not Displaying Text

- **Check Wiring:** Ensure all LCD pins are correctly connected to the ESP32 as per the code.
- **Contrast:** Adjust the potentiometer (if used) for contrast.
- **Initialization:** Confirm `lcd.begin(16, 2);` is in `setup()`.
- **Blog:** Here is a blog which helped me solving this issue. See[`blog`](https://diyprojectslabs.com/interfacing-zmpt101b-voltage-sensor-with-esp32/)

### 2. No Voltage Reading / Always 0V

- **Sensor Wiring:** Make sure the ZMPT101B sensor's output is connected to the correct analog pin (27 by default).
- **Sensitivity:** Try adjusting the `SENSITIVITY` value in code for your sensor.
- **Calibration:** Double-check the `CALIBRATION_FACTOR` or adjust based on your AC supply. In India the Frequency for AC Input is 50 Hz and I have set the Callibration according to 220V AC.

### 3. Relay Not Switching

- **Pin Definition:** Ensure `RELAY_PIN` in code matches your actual relay pin.
- **Relay Wiring:** Confirm relay VCC and GND connections; some relay modules require external power.
- **Relay Type:** If using a 5V relay, ensure it is compatible with ESP32's output logic level.

### 4. Serial Monitor Shows Garbage Values

- **Baud Rate:** Set Serial Monitor to 115200 baud.
- **Wiring:** Avoid cross-talk between LCD and Serial pins.

### 5. Device Does Not Trip on Over/Under Voltage

- **Thresholds:** Verify `OVER_VOLTAGE` and `UNDER_VOLTAGE` constants in code match your requirements.
- **Sensor Output:** Use Serial Monitor to confirm voltage readings are accurate.

### 6. Interfacing ZMPT101B Voltage Sensor with ESP32
- **Blog:** Here is a blog which helped me solve how to interface ZMPT101B sensor with ESP32. After this i got the perfect reading.See[`blog`](https://diyprojectslabs.com/interfacing-zmpt101b-voltage-sensor-with-esp32/)

There will be a garbage voltage reading when the power is off, it is due to the Induced/Ghost Voltage.  
---


## 👥 Contributors

- **Walter Hartwell White** ([@Walter-Hartwell-White](https://github.com/Walter-Hartwell-White)) - Project Design & Implementation
- **[Your Friend's Name]** - Contributor & Support

---

## 📜 License

This project is open-source and available under the [MIT License](LICENSE).
