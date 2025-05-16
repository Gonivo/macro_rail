📸 Macro Rail Controller for ESP32

**Macro Rail Controller** is an ESP32-based project designed to automate macro focus stacking photography. It allows precise control of camera or object movement using a stepper motor and provides a convenient web interface for configuration and control.

This firmware enables:
- 🔄 Automatic homing with endstop detection
- 📏 Precise positioning in hundredths of a millimeter
- 📷 Focus stacking mode with customizable step size and delays
- ⚙️ Web-based UI for easy control and configuration
- 🌐 Wi-Fi connectivity for wireless operation

---

![Animation showing the web interface in action](docs/macro_rail_ui.gif)

## 🧩 Hardware Requirements

| Component              | Description |
|------------------------|-------------|
| ESP32 Dev Module       | Main controller |
| Stepper Motor          | For linear movement |
| DRV8825/A4988 Driver   | Controls the stepper motor |
| Endstop Switch         | Detects home position |
| 12V Power Supply       | Powers the motor and logic |
| 12V -> 5V Buck module  | Powers the ESP32 |
| Transistors/Relays     | Control camera focus and shutter |

> Make sure your mechanical system (e.g., leadscrew) is compatible with the configuration in the code.

---

## 🛠️ Installation

### Prerequisites:
- [PlatformIO](https://platformio.org/ ) (integrated with VSCode)
- ESP32 board support installed
- Arduino framework
- Required libraries:
  - `ArduinoJson`
  - `AccelStepper`
  - `WiFi`
  - `WebServer`

### Steps:
1. Clone the repository:
   ```bash
   git clone https://github.com/yourname/macro_rail.git 
2. Open the project in VSCode + PlatformIO
3. Configure Wi-Fi credentials in main.cpp:
WifiCredentials wifiNetworks[] = {
    {"SSID1", "PASSWORD1"},
    {nullptr, nullptr}
};
4. Upload the firmware to your ESP32
5. Use the Serial Monitor to check the assigned IP address, or locate it manually through your access point's DHCP client list.
6. Navigate to http://<ESP32_IP> in your browser to access the web interface


🖥️ Features & Usage

Web Interface Controls:
🔧 Home — Find the zero/home position using the endstop
⚠️ Stop — Stop any ongoing motion
🔁 Reset Error — Clear error state after hitting the endstop
📍 Move To — Move to a specific position in mm
➕➖ Step Buttons — Fine manual movements: ±0.01 / ±0.1 / ±1 mm
📷 Start Shooting — Begin automatic photo sequence with custom settings

Shooting Settings:
Number of photos
Step size in mm
Speed in mm/s
Delay before focusing
Focus hold time
Shutter release time
Option to return to start position after shooting

📦 Example Workflow
Connect power and ensure the rail is free to move.
Open the web interface and click Home to set the zero point.
Set up your camera and configure shooting parameters:
Photo count: 10
Step size: 0.5 mm
Speed: 0.7 mm/s
Focus time: 500 ms
Release time: 200 ms
Click Start to begin the automated photo series.
The system will move, focus, and trigger the shutter at each step.
Optionally, enable "Return to Start" to go back to the original position.

📝 License
MIT License — feel free to use and modify, but please give appropriate credit.

📬 Contact
If you have questions, suggestions, or found bugs — feel free to open an issue on GitHub or contact me directly.

🌟 Credits
Arduino
ESP-IDF & PlatformIO
AccelStepper Library
ArduinoJson