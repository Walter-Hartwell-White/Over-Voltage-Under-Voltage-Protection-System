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
- **16x2 LCD Display (parallel interface)**
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
   Use the Arduino IDE to upload [main.ino](main.ino) to your ESP32.

---

## 📹 Demo Video

*Check out the demo video below to see the system in action!*


[![Demo Video](demo/demo_thumbnail.png)](demo/demo_video.mp4)


---

## 📄 Source Code

See [`main.ino`](main.ino) for the complete Arduino code.

---

## 👥 Contributors

- **Walter Hartwell White** ([@Walter-Hartwell-White](https://github.com/Walter-Hartwell-White)) - Project Design & Implementation
- **[Your Friend's Name]** - Contributor & Support

---

## 📜 License

This project is open-source and available under the [MIT License](LICENSE).
