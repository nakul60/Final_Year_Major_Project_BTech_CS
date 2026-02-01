import os, json, hashlib, tempfile, shutil
from datetime import datetime
from typing import Dict, List, Optional, Tuple, Any, Annotated
from pathlib import Path
import subprocess, requests, asyncio, PyPDF2
import re
from dotenv import load_dotenv
from langchain_core.messages import BaseMessage, HumanMessage, AIMessage, SystemMessage
from langchain_core.tools import BaseTool, tool
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from langchain_core.output_parsers import JsonOutputParser
from langchain_google_genai import ChatGoogleGenerativeAI
from langchain_community.vectorstores import Chroma
from langchain_huggingface import HuggingFaceEmbeddings
from langchain_community.document_loaders import PyPDFLoader, TextLoader
from langchain_text_splitters import RecursiveCharacterTextSplitter
from langchain_core.documents import Document
from langgraph.graph import StateGraph, END
from langgraph.prebuilt import ToolNode
from langgraph.graph.message import add_messages
from typing_extensions import TypedDict
from sentence_transformers import SentenceTransformer
import numpy as np
from duckduckgo_search import DDGS

# Load environment variables from .env file
load_dotenv()

# Check for API key
API_KEY = os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY")

if not API_KEY:
    print("❌ Error: No API key found in environment variables.")
    print("Please set GEMINI_API_KEY in your .env file.")
    print("Example: GEMINI_API_KEY=your_api_key_here")

WEB_SEARCH_AVAILABLE = True

"""# Class of State"""

class ProjectState(TypedDict):
    """State management for embedded projects using TypedDict for LangGraph compatibility"""
    messages: Annotated[list, add_messages]
    platform: str
    requirements: str
    generated_code: str
    validation_result: Optional[Dict]
    documentation: str
    project_name: str
    current_step: str
    context_chunks: Optional[List[str]]
    error_message: str
    search_results: Optional[List[Dict]]
    iteration_count: int  # Track iterations to prevent infinite loops

class EmbeddedSystemsTools:
    """Collection of tools for embedded systems development"""

    def __init__(self, knowledge_base_path: str = "./knowledge_base"):
        self.knowledge_base_path = Path(knowledge_base_path)
        self.knowledge_base_path.mkdir(exist_ok=True)

        # Initialize embeddings and vector store
        try:
            self.embeddings = HuggingFaceEmbeddings(model_name="all-MiniLM-L6-v2")
            self.vectorstore = Chroma(
                persist_directory=str(self.knowledge_base_path / "chroma_db"),
                embedding_function=self.embeddings
            )
        except Exception as e:
            print(f"⚠️ Vector store initialization failed: {e}")
            self.vectorstore = None

        # Text splitter for documents
        self.text_splitter = RecursiveCharacterTextSplitter(
            chunk_size=1000,
            chunk_overlap=200
        )

        # Platform configurations
        self.platforms = {
            "arduino": {
                "language": "C++",
                "extensions": [".ino", ".cpp", ".h"],
                "libraries": ["Arduino.h", "SoftwareSerial.h", "Wire.h", "SPI.h"],
                "common_sensors": ["DHT22", "DS18B20", "BME280", "MPU6050"]
            },
            "esp32": {
                "language": "C++",
                "extensions": [".ino", ".cpp", ".h"],
                "libraries": ["WiFi.h", "WebServer.h", "BluetoothSerial.h", "SPIFFS.h"],
                "common_sensors": ["DHT22", "DS18B20", "BME280", "MPU6050", "HC-SR04"]
            },
            "raspberry_pi": {
                "language": "Python",
                "extensions": [".py"],
                "libraries": ["RPi.GPIO", "gpiozero", "picamera", "spidev", "smbus"],
                "common_sensors": ["DHT22", "DS18B20", "BME280", "MPU6050", "HC-SR04"]
            }
        }

# Global tools instance for tool functions
tools_instance = None

"""# Tools"""

# Define tools using @tool decorator
@tool
def web_search_tool(query: str, max_results: int = 5) -> str:
    """Search the web for embedded systems information, tutorials, and documentation"""
    if not WEB_SEARCH_AVAILABLE:
        return "Web search not available. Please install duckduckgo-search package."

    try:
        ddgs = DDGS()
        results = []

        # Add embedded systems context to query
        enhanced_query = f"{query} embedded systems arduino esp32 raspberry pi"

        search_results = ddgs.text(enhanced_query, max_results=max_results)

        for result in search_results:
            results.append({
                "title": result.get("title", ""),
                "url": result.get("href", ""),
                "snippet": result.get("body", "")
            })

        # Format results as string
        formatted_results = "🔍 Web Search Results:\n\n"
        for i, result in enumerate(results, 1):
            formatted_results += f"{i}. **{result['title']}**\n"
            formatted_results += f"   URL: {result['url']}\n"
            formatted_results += f"   {result['snippet'][:200]}...\n\n"

        return formatted_results

    except Exception as e:
        return f"Web search failed: {str(e)}"

@tool
def component_lookup_tool(component_name: str) -> str:
    """Look up information about electronic components, sensors, and modules"""
    component_db = {
        "dht22": {
            "type": "Temperature & Humidity Sensor",
            "description": "Digital temperature and humidity sensor with high accuracy",
            "voltage": "3.3-5V",
            "pins": ["VCC (Red)", "Data (Yellow)", "NC (Not Connected)", "GND (Black)"],
            "libraries": ["DHT", "Adafruit_DHT"],
            "arduino_code": """
#include <DHT.h>
#define DHTPIN 2
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  dht.begin();
}

void loop() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print("%, Temperature: ");
  Serial.println(t);
  delay(2000);
}""",
            "raspberry_pi_code": """
import Adafruit_DHT
sensor = Adafruit_DHT.DHT22
pin = 4

humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)
if humidity is not None and temperature is not None:
    print(f'Temp: {temperature:.1f}°C  Humidity: {humidity:.1f}%')
"""
        },
        "hc-sr04": {
            "type": "Ultrasonic Distance Sensor",
            "description": "Measures distance using ultrasonic waves (2cm-400cm range)",
            "voltage": "5V",
            "pins": ["VCC", "Trig", "Echo", "GND"],
            "libraries": ["NewPing (Arduino)"],
            "arduino_code": """
#define trigPin 9
#define echoPin 8

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;

  Serial.print(distance);
  Serial.println(" cm");
  delay(1000);
}""",
            "raspberry_pi_code": """
import RPi.GPIO as GPIO
import time

TRIG = 23
ECHO = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def get_distance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    return round(distance, 2)
"""
        },
        "led": {
            "type": "Light Emitting Diode",
            "description": "Basic LED for visual indication",
            "voltage": "1.8-3.3V (with current limiting resistor)",
            "pins": ["Anode (+)", "Cathode (-)"],
            "arduino_code": """
int ledPin = 13;

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  digitalWrite(ledPin, HIGH);
  delay(1000);
  digitalWrite(ledPin, LOW);
  delay(1000);
}""",
            "raspberry_pi_code": """
import RPi.GPIO as GPIO
import time

LED_PIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)

try:
    while True:
        GPIO.output(LED_PIN, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(LED_PIN, GPIO.LOW)
        time.sleep(1)
except KeyboardInterrupt:
    GPIO.cleanup()
"""
        }
    }

    component_key = component_name.lower().replace("-", "").replace(" ", "")

    # Find matching component
    for key, info in component_db.items():
        if component_key in key or component_name.lower() in info["type"].lower():
            result = f"🔌 **{info['type']}**\n\n"
            result += f"**Description:** {info['description']}\n"
            result += f"**Voltage:** {info['voltage']}\n"
            result += f"**Pins:** {', '.join(info['pins'])}\n"

            if 'libraries' in info:
                result += f"**Libraries:** {', '.join(info['libraries'])}\n"

            if 'arduino_code' in info:
                result += f"\n**Arduino Code:**\n```cpp\n{info['arduino_code']}\n```\n"

            if 'raspberry_pi_code' in info:
                result += f"\n**Raspberry Pi Code:**\n```python\n{info['raspberry_pi_code']}\n```\n"

            return result

    return f"❌ Component '{component_name}' not found in database. Try: DHT22, HC-SR04, LED"

@tool
def pinout_lookup_tool(platform: str) -> str:
    """Get detailed pinout information for microcontrollers and development boards"""
    pinouts = {
        "arduino_uno": {
            "description": "Arduino Uno R3 Pinout",
            "digital_pins": "0-13 (pins 0,1 are RX,TX for serial communication)",
            "analog_pins": "A0-A5 (can also be used as digital pins 14-19)",
            "power_pins": "3.3V, 5V, GND, VIN (7-12V input)",
            "pwm_pins": "3, 5, 6, 9, 10, 11 (marked with ~ symbol)",
            "special_pins": {
                "I2C SDA": "A4 (or pin 18)",
                "I2C SCL": "A5 (or pin 19)",
                "SPI SS": "10",
                "SPI MOSI": "11",
                "SPI MISO": "12",
                "SPI SCK": "13",
                "LED_BUILTIN": "13"
            },
            "notes": "- Pins 0,1 used for USB communication\n- Pin 13 has built-in LED\n- Maximum current per pin: 20mA"
        },
        "esp32": {
            "description": "ESP32 Development Board Pinout",
            "digital_pins": "0-39 (some are input-only: 34, 35, 36, 39)",
            "analog_pins": "32-39, 25-27, 12-15, 2, 0, 4 (12-bit ADC)",
            "touch_pins": "0, 2, 4, 12, 13, 14, 15, 27, 32, 33",
            "pwm_pins": "All digital pins support PWM",
            "special_pins": {
                "I2C SDA": "21 (default)",
                "I2C SCL": "22 (default)",
                "UART RX": "3",
                "UART TX": "1",
                "SPI SS": "5",
                "SPI MOSI": "23",
                "SPI MISO": "19",
                "SPI SCK": "18",
                "Built-in LED": "2"
            },
            "notes": "- WiFi and Bluetooth built-in\n- 3.3V logic level\n- Some pins are strapping pins (0, 2, 5, 12, 15)\n- Pins 6-11 connected to flash memory"
        },
        "raspberry_pi": {
            "description": "Raspberry Pi 4 GPIO Pinout (40-pin header)",
            "gpio_pins": "GPIO 2-27 (40-pin header)",
            "power_pins": "3.3V (pins 1,17), 5V (pins 2,4), GND (pins 6,9,14,20,25,30,34,39)",
            "special_pins": {
                "I2C SDA": "GPIO 2 (pin 3)",
                "I2C SCL": "GPIO 3 (pin 5)",
                "UART RX": "GPIO 15 (pin 10)",
                "UART TX": "GPIO 14 (pin 8)",
                "SPI MOSI": "GPIO 10 (pin 19)",
                "SPI MISO": "GPIO 9 (pin 21)",
                "SPI SCK": "GPIO 11 (pin 23)",
                "SPI CE0": "GPIO 8 (pin 24)",
                "SPI CE1": "GPIO 7 (pin 26)",
                "PWM0": "GPIO 12 (pin 32)",
                "PWM1": "GPIO 13 (pin 33)"
            },
            "notes": "- 3.3V logic level (NOT 5V tolerant!)\n- Maximum current per pin: 16mA\n- Total current from 3.3V supply: 50mA\n- BCM numbering vs Physical pin numbering"
        }
    }

    platform_key = platform.lower().replace("-", "_").replace(" ", "_")

    if platform_key in pinouts:
        info = pinouts[platform_key]
        result = f"📌 **{info['description']}**\n\n"

        if 'digital_pins' in info:
            result += f"**Digital Pins:** {info['digital_pins']}\n"
        if 'analog_pins' in info:
            result += f"**Analog Pins:** {info['analog_pins']}\n"
        if 'power_pins' in info:
            result += f"**Power Pins:** {info['power_pins']}\n"
        if 'pwm_pins' in info:
            result += f"**PWM Pins:** {info['pwm_pins']}\n"
        if 'touch_pins' in info:
            result += f"**Touch Pins:** {info['touch_pins']}\n"

        result += f"\n**Special Functions:**\n"
        for func, pin in info['special_pins'].items():
            result += f"- {func}: {pin}\n"

        result += f"\n**Important Notes:**\n{info['notes']}"

        return result
    else:
        available = list(pinouts.keys())
        return f"❌ Pinout for '{platform}' not found. Available: {', '.join(available)}"

@tool
def code_template_tool(platform: str, template_type: str) -> str:
    """Get code templates for different platforms and use cases"""
    templates = {
        "arduino": {
            "basic": """
// Arduino Basic Template
#include <Arduino.h>

void setup() {
    Serial.begin(9600);
    Serial.println("Arduino Started!");
    // Initialize your components here
}

void loop() {
    // Main code here
    Serial.println("Hello World!");
    delay(1000);
}
""",
            "sensor": """
// Arduino Sensor Reading Template
#include <Arduino.h>

// Pin definitions
const int sensorPin = A0;
const int ledPin = 13;

void setup() {
    Serial.begin(9600);
    pinMode(sensorPin, INPUT);
    pinMode(ledPin, OUTPUT);
    Serial.println("Sensor Monitor Started");
}

void loop() {
    int sensorValue = analogRead(sensorPin);
    float voltage = sensorValue * (5.0 / 1023.0);

    Serial.print("Sensor Value: ");
    Serial.print(sensorValue);
    Serial.print(", Voltage: ");
    Serial.print(voltage, 2);
    Serial.println("V");

    // Blink LED based on sensor value
    if (sensorValue > 512) {
        digitalWrite(ledPin, HIGH);
    } else {
        digitalWrite(ledPin, LOW);
    }

    delay(500);
}
""",
            "servo": """
// Arduino Servo Control Template
#include <Servo.h>

Servo myServo;
const int servoPin = 9;
const int potPin = A0;

void setup() {
    Serial.begin(9600);
    myServo.attach(servoPin);
    Serial.println("Servo Control Ready");
}

void loop() {
    int potValue = analogRead(potPin);
    int angle = map(potValue, 0, 1023, 0, 180);

    myServo.write(angle);

    Serial.print("Potentiometer: ");
    Serial.print(potValue);
    Serial.print(", Servo Angle: ");
    Serial.println(angle);

    delay(50);
}
"""
        },
        "esp32": {
            "basic": """
// ESP32 Basic Template
#include <WiFi.h>
#include <Arduino.h>

const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("ESP32 Starting...");

    // Connect to WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");

    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

void loop() {
    Serial.println("ESP32 is running...");
    delay(5000);
}
""",
            "webserver": """
// ESP32 Web Server Template
#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

WebServer server(80);
int ledPin = 2;
bool ledState = false;

void handleRoot() {
    String html = "<html><body>";
    html += "<h1>ESP32 Web Server</h1>";
    html += "<p>LED Status: " + String(ledState ? "ON" : "OFF") + "</p>";
    html += "<p><a href='/led/on'>Turn LED ON</a></p>";
    html += "<p><a href='/led/off'>Turn LED OFF</a></p>";
    html += "</body></html>";

    server.send(200, "text/html", html);
}

void handleLEDOn() {
    ledState = true;
    digitalWrite(ledPin, HIGH);
    server.send(200, "text/plain", "LED is ON");
}

void handleLEDOff() {
    ledState = false;
    digitalWrite(ledPin, LOW);
    server.send(200, "text/plain", "LED is OFF");
}

void setup() {
    Serial.begin(115200);
    pinMode(ledPin, OUTPUT);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }

    Serial.println("WiFi connected!");
    Serial.println("IP address: " + WiFi.localIP().toString());

    server.on("/", handleRoot);
    server.on("/led/on", handleLEDOn);
    server.on("/led/off", handleLEDOff);

    server.begin();
    Serial.println("HTTP server started");
}

void loop() {
    server.handleClient();
}
""",
            "bluetooth": """
// ESP32 Bluetooth Template
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
String deviceName = "ESP32-Device";

void setup() {
    Serial.begin(115200);
    SerialBT.begin(deviceName);
    Serial.println("Device started, now you can pair it with bluetooth!");
    Serial.println("Device name: " + deviceName);
}

void loop() {
    // Forward data between Serial and Bluetooth
    if (Serial.available()) {
        String message = Serial.readString();
        SerialBT.print(message);
        Serial.print("Sent: " + message);
    }

    if (SerialBT.available()) {
        String message = SerialBT.readString();
        Serial.print("Received: " + message);
        // Echo back
        SerialBT.print("Echo: " + message);
    }

    delay(20);
}
"""
        },
        "raspberry_pi": {
            "basic": """
#!/usr/bin/env python3
# Raspberry Pi Basic Template

import time
import sys

def setup():
    \"""Initialize your components here\"""
    print("Raspberry Pi application starting...")
    print("Setup complete!")

def main_loop():
    \"""Main program loop\"""
    print("Entering main loop...")

    try:
        counter = 0
        while True:
            print(f"Loop iteration: {counter}")
            counter += 1
            time.sleep(2)

    except KeyboardInterrupt:
        cleanup()

def cleanup():
    \"""Cleanup resources before exit\"""
    print("\\nCleaning up and exiting...")
    sys.exit(0)

if __name__ == "__main__":
    setup()
    main_loop()
""",
            "gpio": """
#!/usr/bin/env python3
# Raspberry Pi GPIO Template

import RPi.GPIO as GPIO
import time
import signal
import sys

# Pin definitions
LED_PIN = 18
BUTTON_PIN = 2

def setup_gpio():
    \"""Setup GPIO pins\"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(LED_PIN, GPIO.OUT)
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    print("GPIO setup complete")

def signal_handler(sig, frame):
    \"""Handle Ctrl+C gracefully\"""
    cleanup()

def cleanup():
    \"""Cleanup GPIO and exit\"""
    print("\\nCleaning up GPIO...")
    GPIO.cleanup()
    sys.exit(0)

def main_loop():
    \"""Main application loop\"""
    print("Starting GPIO control loop...")
    print("Press Ctrl+C to exit")

    led_state = False

    try:
        while True:
            # Read button state
            button_pressed = GPIO.input(BUTTON_PIN) == GPIO.LOW

            if button_pressed:
                led_state = not led_state
                GPIO.output(LED_PIN, GPIO.HIGH if led_state else GPIO.LOW)
                print(f"Button pressed! LED {'ON' if led_state else 'OFF'}")

                # Debounce delay
                time.sleep(0.3)

            time.sleep(0.1)

    except KeyboardInterrupt:
        cleanup()

if __name__ == "__main__":
    # Setup signal handler for clean exit
    signal.signal(signal.SIGINT, signal_handler)

    setup_gpio()
    main_loop()
""",
            "camera": """
#!/usr/bin/env python3
# Raspberry Pi Camera Template

import time
from datetime import datetime
import os

# Try to import picamera, fall back to alternative methods
try:
    from picamera2 import Picamera2
    CAMERA_LIB = "picamera2"
except ImportError:
    try:
        from picamera import PiCamera
        CAMERA_LIB = "picamera"
    except ImportError:
        CAMERA_LIB = None
        print("No camera library found. Install with: sudo apt install python3-picamera2")

def setup_camera():
    \"""Initialize camera\"""
    if CAMERA_LIB == "picamera2":
        camera = Picamera2()
        camera.configure(camera.create_still_configuration())
        camera.start()
        return camera
    elif CAMERA_LIB == "picamera":
        camera = PiCamera()
        camera.resolution = (1920, 1080)
        camera.start_preview()
        time.sleep(2)  # Camera warm-up time
        return camera
    else:
        return None

def capture_image(camera, filename=None):
    \"""Capture a single image\"""
    if not camera:
        print("Camera not available")
        return False

    if not filename:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"image_{timestamp}.jpg"

    try:
        if CAMERA_LIB == "picamera2":
            camera.capture_file(filename)
        elif CAMERA_LIB == "picamera":
            camera.capture(filename)

        print(f"Image saved: {filename}")
        return True

    except Exception as e:
        print(f"Error capturing image: {e}")
        return False

def main_loop():
    \"""Main camera application\"""
    camera = setup_camera()

    if not camera:
        print("Failed to initialize camera")
        return

    print("Camera ready. Press Ctrl+C to exit")
    print("Capturing images every 10 seconds...")

    try:
        while True:
            capture_image(camera)
            time.sleep(10)

    except KeyboardInterrupt:
        print("\\nExiting...")
    finally:
        if CAMERA_LIB == "picamera2":
            camera.stop()
        elif CAMERA_LIB == "picamera":
            camera.stop_preview()
            camera.close()
        print("Camera closed")

if __name__ == "__main__":
    main_loop()
"""
        }
    }

    platform_key = platform.lower()
    template_key = template_type.lower()

    if platform_key in templates:
        if template_key in templates[platform_key]:
            template_code = templates[platform_key][template_key]
            return f"📝 **{platform.title()} {template_type.title()} Template**\n\n```{templates[platform_key].get('language', 'text')}\n{template_code.strip()}\n```"
        else:
            available = list(templates[platform_key].keys())
            return f"❌ Template '{template_type}' not found for {platform}. Available: {', '.join(available)}"
    else:
        available = list(templates.keys())
        return f"❌ Platform '{platform}' not supported. Available: {', '.join(available)}"

@tool
def code_validator_tool(code: str, platform: str) -> Dict:
    """Validate code syntax and structure for embedded platforms"""
    try:
        validation_result = {
            "success": True,
            "warnings": [],
            "errors": [],
            "suggestions": []
        }

        if platform.lower() in ["arduino", "esp32"]:
            # Arduino/ESP32 C++ validation
            if "setup()" not in code:
                validation_result["errors"].append("Missing required setup() function")

            if "loop()" not in code and platform.lower() != "library":
                validation_result["errors"].append("Missing required loop() function")

            # Check for common issues
            if "Serial.begin" in code and "Serial.h" not in code and "#include <Arduino.h>" not in code:
                validation_result["warnings"].append("Using Serial without including Arduino.h")

            # Check for WiFi usage without include
            if "WiFi." in code and "#include <WiFi.h>" not in code:
                validation_result["errors"].append("Using WiFi functions without including WiFi.h")

        elif platform.lower() == "raspberry_pi":
            # Python validation
            try:
                compile(code, '<string>', 'exec')
            except SyntaxError as e:
                validation_result["errors"].append(f"Python syntax error: {str(e)}")

            # Check for GPIO usage
            if "GPIO." in code and "import RPi.GPIO" not in code:
                validation_result["warnings"].append("Using GPIO without importing RPi.GPIO")

            if "GPIO.cleanup()" not in code and "GPIO." in code:
                validation_result["suggestions"].append("Consider adding GPIO.cleanup() for proper resource cleanup")

        # General checks
        lines = code.split('\n')
        for i, line in enumerate(lines, 1):
            if len(line.strip()) > 120:
                validation_result["warnings"].append(f"Line {i} is very long (>120 chars)")

        if validation_result["errors"]:
            validation_result["success"] = False

        return validation_result

    except Exception as e:
        return {"success": False, "error": f"Validation failed: {str(e)}"}

@tool
def library_lookup_tool(library_name: str, platform: str) -> Dict:
    """Look up library information and installation instructions"""
    libraries = {
        "arduino": {
            "dht": {
                "name": "DHT sensor library",
                "description": "Arduino library for DHT11, DHT22, etc Temp & Humidity Sensors",
                "installation": "Arduino Library Manager: Search 'DHT sensor library'",
                "github": "https://github.com/adafruit/DHT-sensor-library",
                "example": "#include <DHT.h>\nDHT dht(2, DHT22);"
            },
            "servo": {
                "name": "Servo",
                "description": "Control servo motors",
                "installation": "Built-in Arduino library",
                "example": "#include <Servo.h>\nServo myservo;"
            },
            "wifi": {
                "name": "WiFi",
                "description": "WiFi functionality for ESP32/ESP8266",
                "installation": "Built-in for ESP32",
                "example": "#include <WiFi.h>\nWiFi.begin(ssid, password);"
            }
        },
        "raspberry_pi": {
            "rpi.gpio": {
                "name": "RPi.GPIO",
                "description": "Raspberry Pi GPIO control library",
                "installation": "pip install RPi.GPIO",
                "example": "import RPi.GPIO as GPIO\nGPIO.setmode(GPIO.BCM)"
            },
            "gpiozero": {
                "name": "GPIO Zero",
                "description": "Simple interface to GPIO devices",
                "installation": "pip install gpiozero",
                "example": "from gpiozero import LED\nled = LED(18)"
            },
            "picamera": {
                "name": "PiCamera",
                "description": "Raspberry Pi camera module interface",
                "installation": "pip install picamera",
                "example": "from picamera import PiCamera\ncamera = PiCamera()"
            }
        }
    }

    platform_libs = libraries.get(platform.lower(), {})
    lib_key = library_name.lower().replace("-", "").replace("_", "").replace(".", "")

    for key, info in platform_libs.items():
        if lib_key in key or library_name.lower() in info["name"].lower():
            return {"success": True, "library": info}

    return {"error": f"Library '{library_name}' not found for platform '{platform}'"}

@tool
def knowledge_search_tool(query: str, tools_instance=None) -> List[str]:
    """Search the knowledge base for relevant information"""
    # This needs access to the tools instance
    if tools_instance and hasattr(tools_instance, 'vectorstore'):
        try:
            docs = tools_instance.vectorstore.similarity_search(query, k=3)
            return [doc.page_content for doc in docs]
        except Exception as e:
            return [f"Knowledge search error: {str(e)}"]
    return ["Knowledge base not available"]

@tool
def file_operations_tool(operation: str, file_path: str, content: str = "") -> Dict:
    """Perform file operations like read, write, create directories"""
    try:
        path = Path(file_path)

        if operation == "read":
            if path.exists():
                with open(path, 'r', encoding='utf-8') as f:
                    return {"success": True, "content": f.read()}
            else:
                return {"error": f"File {file_path} not found"}

        elif operation == "write":
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, 'w', encoding='utf-8') as f:
                f.write(content)
            return {"success": True, "message": f"File written to {file_path}"}

        elif operation == "create_dir":
            path.mkdir(parents=True, exist_ok=True)
            return {"success": True, "message": f"Directory created: {file_path}"}

        elif operation == "list":
            if path.exists() and path.is_dir():
                files = [str(f) for f in path.iterdir()]
                return {"success": True, "files": files}
            else:
                return {"error": f"Directory {file_path} not found"}

        else:
            return {"error": f"Unknown operation: {operation}"}

    except Exception as e:
        return {"error": f"File operation failed: {str(e)}"}

def power_profile_estimator(code: str, platform: str) -> Dict:
    """Estimate power consumption and suggest optimizations for embedded code."""
    # Typical mA values (approximate)
    mcu_profiles = {
        "esp32": {"active": 80, "wifi": 160, "bt": 90, "deep_sleep": 0.01},
        "arduino_uno": {"active": 45, "deep_sleep": 0.05},
        "raspberry_pi_4": {"active": 600, "wifi": 200, "bt": 100, "deep_sleep": 10},
    }
    sensors = {"dht22": 1.5, "bme280": 0.7, "mpu6050": 3.9, "hc-sr04": 15, "led": 10}
    profile = mcu_profiles.get(platform.lower(), {"active": 100, "deep_sleep": 1})
    total_active = profile["active"]
    total_deep = profile.get("deep_sleep", 1)
    peripherals = []
    suggestions = []
    # Detect peripherals
    if "WiFi" in code or "wifi" in code:
        total_active += profile.get("wifi", 0)
        peripherals.append("WiFi")
    if "Bluetooth" in code or "SerialBT" in code:
        total_active += profile.get("bt", 0)
        peripherals.append("Bluetooth")
    for s in sensors:
        if s in code.lower():
            total_active += sensors[s]
            peripherals.append(s)
    # Detect delay() usage
    if re.search(r'delay\s*\(', code):
        suggestions.append("Code uses delay(). Suggest using deep sleep or interrupts for better battery life.")
        if platform.lower() == "esp32":
            suggestions.append("Consider replacing delay() with ESP.deepSleep() for power savings.")
    # Build report
    report = {
        "active_mode_ma": round(total_active, 2),
        "deep_sleep_ma": round(total_deep, 2),
        "peripherals": peripherals,
        "suggestions": suggestions,
        "note": "Estimates are approximate. Actual current depends on hardware and code."
    }
    return report

# Keep the raw function available for direct calls
power_profile_estimator_fn = power_profile_estimator

# Create a tool version for LangGraph
power_profile_estimator_tool = tool(power_profile_estimator)

"""# Workflow"""

class EmbeddedSystemsAgent:
    """Main agent class using LangGraph for embedded systems development"""
    def __init__(self, api_key: str = None, knowledge_base_path: str = "./knowledge_base"):
        """
        Initialize the agent with Gemini API.
        
        Args:
            api_key: API key for Gemini
            knowledge_base_path: Path to store knowledge base files
        """
        self.llm = ChatGoogleGenerativeAI(
            google_api_key=api_key,
            model="gemini-2.5-flash",
            temperature=0.1,
            convert_system_message_to_human=True
        )
        print("✅ Agent initialized successfully")

        self.tools_instance = EmbeddedSystemsTools(knowledge_base_path)

        # Initialize tools
        self.tools = [
            web_search_tool,
            component_lookup_tool,
            pinout_lookup_tool,
            code_template_tool,
            code_validator_tool,
            library_lookup_tool,
            file_operations_tool
        ]

        # ✅ Instead of ToolExecutor, use ToolNode
        self.tool_node = ToolNode(tools=self.tools)

        # Create the graph
        self.graph = self._create_graph()
        self._max_iterations = 3  # Limit agent iterations to prevent API exhaustion

    def _get_content_as_string(self, content) -> str:
        """Safely convert content to string, handling lists and other types."""
        if content is None:
            return ""
        if isinstance(content, str):
            return content
        if isinstance(content, list):
            text_parts = []
            for item in content:
                if isinstance(item, str):
                    text_parts.append(item)
                elif hasattr(item, 'text'):
                    text_parts.append(item.text)
                elif isinstance(item, dict) and 'text' in item:
                    text_parts.append(item['text'])
            return '\n'.join(text_parts)
        return str(content)

    def _create_graph(self) -> StateGraph:
        """Create the LangGraph workflow - simplified without excessive loops"""

        def call_agent(state: ProjectState):
            # Increment iteration counter
            current_count = state.get("iteration_count", 0)
            state["iteration_count"] = current_count + 1
            
            if state["iteration_count"] > self._max_iterations:
                # Stop if we've exceeded max iterations
                return state
                
            messages = state["messages"]
            response = self.llm.bind_tools(self.tools).invoke(messages)
            state["messages"].append(response)
            
            # Extract code from response if present
            if hasattr(response, 'content') and response.content:
                content_str = self._get_content_as_string(response.content)
                if content_str:
                    code = self._extract_code_from_response(content_str)
                    if code:
                        state["generated_code"] = code
            
            return state

        def should_use_tools(state: ProjectState):
            """Check if the last message has tool calls"""
            # Stop if max iterations reached
            if state.get("iteration_count", 0) >= self._max_iterations:
                return "end"
                
            last_message = state["messages"][-1]
            # Check if there are tool calls
            if hasattr(last_message, 'tool_calls') and last_message.tool_calls:
                return "tools"
            return "end"

        def call_tools(state: ProjectState):
            result = self.tool_node.invoke({"messages": state["messages"]})
            if isinstance(result, dict) and "messages" in result:
                state["messages"] = result["messages"]
            return state

        def should_continue_after_tools(state: ProjectState):
            """After tools, go back to agent for final response (only once)"""
            # Limit to prevent infinite loops
            if state.get("iteration_count", 0) >= self._max_iterations:
                return "end"
            return "agent"

        # Build simplified graph
        workflow = StateGraph(ProjectState)
        workflow.add_node("agent", call_agent)
        workflow.add_node("tools", call_tools)

        workflow.set_entry_point("agent")
        
        # After agent, check if tools are needed
        workflow.add_conditional_edges("agent", should_use_tools, {
            "tools": "tools",
            "end": END
        })
        
        # After tools, go back to agent for final response
        workflow.add_conditional_edges("tools", should_continue_after_tools, {
            "agent": "agent",
            "end": END
        })

        return workflow.compile()


    async def add_knowledge(self, file_path: str) -> bool:
        """Add documents to the knowledge base"""
        try:
            file_path = Path(file_path)

            if file_path.suffix.lower() == '.pdf':
                loader = PyPDFLoader(str(file_path))
            elif file_path.suffix.lower() == '.txt':
                loader = TextLoader(str(file_path))
            else:
                print(f"Unsupported file type: {file_path.suffix}")
                return False

            documents = loader.load()
            texts = self.tools_instance.text_splitter.split_documents(documents)

            # Add to vector store
            self.tools_instance.vectorstore.add_documents(texts)

            print(f"Added {len(texts)} chunks from {file_path.name}")
            return True

        except Exception as e:
            print(f"Error adding knowledge: {e}")
            return False

    async def process_request(self, user_input: str, platform: str = "", max_retries: int = 3) -> Dict:
        """Process user request through the LangGraph workflow with retry logic for rate limits"""
        import time
        
        for attempt in range(max_retries):
            try:
                # Create initial state with iteration_count initialized to 0
                initial_state = ProjectState(
                    messages=[
                        SystemMessage(content=f"""You are an expert embedded systems developer specializing in Arduino, ESP32, and Raspberry Pi.

Your capabilities include:
- Code generation with best practices (you MAY refer to templates if helpful, but do not strictly follow them)
- Hardware component recommendations and pinout information
- Code validation and debugging assistance
- Documentation generation
- Web search for latest information
- Library and component lookup

Always provide practical, working solutions with clear explanations.
When generating code, consider:
- Platform-specific requirements
- Hardware limitations
- Power consumption
- Error handling
- Code organization

📌 Important:
- Treat templates as **reference only**, not mandatory output format.
- If the request is outside the template (e.g., "traffic light with 3 LEDs"), generate fresh working code.
- If the request mentions templates explicitly, you may use them.
- Respond directly without using tools unless absolutely necessary.

🔌 **WIRING DIAGRAM REQUIREMENT:**
When generating code that involves hardware connections (sensors, LEDs, motors, displays, etc.), you MUST include a Mermaid.js wiring diagram at the end of your response.

**IMPORTANT: Use this exact simple format for valid Mermaid syntax:**
```mermaid
graph TD
    MCU[{platform.upper() if platform else 'MCU'}]
    SENSOR[Component Name]
    
    MCU -->|Pin X - Data| SENSOR
    MCU -->|5V - Power| SENSOR
    MCU -->|GND - Ground| SENSOR
```

**Example for DHT22 sensor on Arduino:**
```mermaid
graph TD
    A[Arduino Uno]
    D[DHT22 Sensor]
    
    A -->|Pin 2 - Data| D
    A -->|5V - Power| D
    A -->|GND - Ground| D
```

**Rules for valid Mermaid:**
1. Use simple node IDs with letters and numbers only (A, B, MCU, LED1, R1, etc.)
2. Put descriptive text inside square brackets [Like This]
3. Use --> for connections with labels like -->|Label|
4. Do NOT use colons, parentheses, or special symbols like Ω or µ in any text
5. Write "Ohm" instead of "Ω", write "10k Ohm" instead of "10kΩ"
6. Keep labels short and simple - avoid special characters
7. One line per connection only
"""),
                        HumanMessage(content=f"Platform: {platform}\nRequest: {user_input}")
                    ],
                    platform=platform,
                    requirements=user_input,
                    current_step="processing",
                    iteration_count=0  # Initialize iteration counter
                )

                # Run the graph
                result = await asyncio.get_event_loop().run_in_executor(
                    None, self.graph.invoke, initial_state
                )

                # Extract the final response
                final_message = result["messages"][-1]
                
                # Get content as string (handle list content from Gemini)
                if hasattr(final_message, 'content'):
                    response_content = self._get_content_as_string(final_message.content)
                else:
                    response_content = str(final_message)

                return {
                    "success": True,
                    "response": response_content,
                    "platform": platform,
                    "timestamp": datetime.now().isoformat()
                }

            except Exception as e:
                error_str = str(e)
                
                # Check if it's a rate limit error (429)
                if "429" in error_str or "RESOURCE_EXHAUSTED" in error_str or "quota" in error_str.lower():
                    # Extract retry delay if available
                    retry_delay = 60  # Default to 60 seconds
                    if "retry" in error_str.lower():
                        import re
                        match = re.search(r'(\d+\.?\d*)\s*s', error_str)
                        if match:
                            retry_delay = float(match.group(1)) + 5  # Add 5 second buffer
                    
                    if attempt < max_retries - 1:
                        print(f"⏳ Rate limit hit. Waiting {retry_delay:.0f}s before retry {attempt + 2}/{max_retries}...")
                        time.sleep(retry_delay)
                        continue
                    else:
                        return {
                            "success": False,
                            "error": f"API rate limit exceeded. Please wait ~{retry_delay:.0f} seconds and try again. You may have exceeded the free tier quota (20 requests/day for gemini-2.5-flash)."
                        }
                
                # For other errors, return immediately
                return {
                    "success": False,
                    "error": f"Request processing failed: {error_str}"
                }

    async def generate_project(self, platform: str, requirements: str,
                             project_name: str) -> Dict:
        """Generate complete project with code, documentation, and validation"""

        try:
            # Step 1: Generate code
            code_request = f"""
Generate complete {platform} code for: {requirements}

Please:
1. Use appropriate templates and libraries
2. Include proper error handling
3. Add comprehensive comments
4. Follow best practices for {platform}
5. Validate the code structure
"""

            code_result = await self.process_request(code_request, platform)

            if not code_result["success"]:
                return code_result

            # Step 2: Generate documentation
            doc_request = f"""
Create comprehensive documentation for this {platform} project:
Requirements: {requirements}
Project Name: {project_name}

Include:
- Project overview
- Hardware requirements and connections
- Software dependencies
- Setup instructions
- Usage guide
- Troubleshooting tips

Format as markdown.
"""

            doc_result = await self.process_request(doc_request, platform)

            # Step 3: Save project
            project_dir = self.tools_instance.knowledge_base_path / "projects" / project_name
            project_dir.mkdir(parents=True, exist_ok=True)

            # Save files
            project_data = {
                "name": project_name,
                "platform": platform,
                "requirements": requirements,
                "generated_code": code_result.get("response", ""),
                "documentation": doc_result.get("response", "") if doc_result.get("success") else "",
                "timestamp": datetime.now().isoformat()
            }

            # Save project metadata
            with open(project_dir / "project.json", 'w') as f:
                json.dump(project_data, f, indent=2)

            # Save documentation
            with open(project_dir / "README.md", 'w') as f:
                f.write(project_data["documentation"])

            # Extract and save code if present
            code_content = self._extract_code_from_response(code_result["response"])
            if code_content:
                if platform.lower() in ["arduino", "esp32"]:
                    code_file = project_dir / f"{project_name}.ino"
                else:
                    code_file = project_dir / f"{project_name}.py"

                with open(code_file, 'w') as f:
                    f.write(code_content)

            return {
                "success": True,
                "project_name": project_name,
                "project_path": str(project_dir),
                "code_generated": bool(code_content),
                "documentation_generated": bool(doc_result.get("success")),
                "files_created": [
                    "project.json",
                    "README.md",
                    f"{project_name}.{'ino' if platform.lower() in ['arduino', 'esp32'] else 'py'}"
                ]
            }

        except Exception as e:
            return {
                "success": False,
                "error": f"Project generation failed: {str(e)}"
            }

    def _extract_code_from_response(self, response) -> str:
        """Extract code blocks from AI response"""
        
        # Handle different response types
        if response is None:
            return ""
        
        # If response is a list, join it or get the first text element
        if isinstance(response, list):
            text_parts = []
            for item in response:
                if isinstance(item, str):
                    text_parts.append(item)
                elif hasattr(item, 'text'):
                    text_parts.append(item.text)
                elif isinstance(item, dict) and 'text' in item:
                    text_parts.append(item['text'])
            response = '\n'.join(text_parts)
        
        # If still not a string, convert it
        if not isinstance(response, str):
            response = str(response)

        # Look for code blocks
        code_patterns = [
            r'```(?:cpp|c\+\+|arduino|ino)\n(.*?)```',
            r'```python\n(.*?)```',
            r'```\n(.*?)```'
        ]

        for pattern in code_patterns:
            matches = re.findall(pattern, response, re.DOTALL)
            if matches:
                return matches[0].strip()

        # If no code blocks found, look for code-like content
        lines = response.split('\n')
        code_lines = []
        in_code = False

        for line in lines:
            # Detect code by common patterns
            if any(keyword in line for keyword in ['#include', 'void setup', 'void loop', 'import ', 'def ', 'if __name__']):
                in_code = True

            if in_code:
                code_lines.append(line)

            # Stop at explanation or other sections
            if line.lower().startswith(('explanation:', 'components:', 'setup:')):
                break

        return '\n'.join(code_lines).strip()

    def _extract_mermaid_from_response(self, response: str) -> str:
        """Extract Mermaid.js code block from AI response."""
        import re
        match = re.search(r'```mermaid\s*([\s\S]+?)```', response)
        if match:
            return match.group(1).strip()
        return ""

    def _generate_mermaid_diagram(self, code: str, platform: str) -> str:
        """Generate a simple Mermaid.js wiring diagram from code and platform."""
        # This is a basic heuristic; for research, you can expand this
        connections = []
        if platform == "arduino_uno":
            if "DHT" in code:
                connections.append("Arduino_Pin2 --> DHT22_Data")
                connections.append("Arduino_5V --> DHT22_VCC")
                connections.append("Arduino_GND --> DHT22_GND")
            if "HC-SR04" in code or "hc-sr04" in code:
                connections.append("Arduino_Pin9 --> HCSR04_Trig")
                connections.append("Arduino_Pin8 --> HCSR04_Echo")
        elif platform == "esp32":
            if "DHT" in code:
                connections.append("ESP32_GPIO21 --> DHT22_Data")
                connections.append("ESP32_3V3 --> DHT22_VCC")
                connections.append("ESP32_GND --> DHT22_GND")
        elif platform == "raspberry_pi_4":
            if "DHT" in code:
                connections.append("RPI_Pin4 --> DHT22_Data")
                connections.append("RPI_3V3 --> DHT22_VCC")
                connections.append("RPI_GND --> DHT22_GND")
        if not connections:
            connections.append("MCU --> Sensor")
        mermaid = "graph TD\n" + "\n".join(connections)
        return mermaid

    # Update system prompt for code generation to always include Mermaid.js diagram
    SYSTEM_PROMPT = (
        "You are an embedded systems copilot. When generating code, always include a Mermaid.js wiring diagram "
        "in a code block after the code, showing how the MCU pins connect to the sensors and actuators. "
        "Format: ```mermaid\ngraph TD\nMCU_GPIO21 --> DHT22_Data\n...```\n"
        "IMPORTANT: Do not use colons, special symbols like Ω, or parentheses in node IDs or labels.\n"
        "If the user asks for code, always output both code and diagram."
    )

"""# CLI"""

class EmbeddedSystemsCLI:
    """Enhanced CLI interface with LangGraph integration"""

    def __init__(self, api_key: str):
        self.agent = EmbeddedSystemsAgent(api_key)
        self.current_platform = ""
        self.session_history = []

    async def run_interactive_session(self):
        """Run enhanced interactive session"""
        print("🤖 Embedded Systems AI Agent with LangGraph")
        print("=" * 50)
        print("Available commands:")
        print("- chat: General conversation and questions")
        print("- generate: Generate code for specific platform")
        print("- project: Create complete project")
        print("- search: Search web for information")
        print("- knowledge: Add knowledge files")
        print("- tools: List available tools")
        print("- platform: Set current platform")
        print("- history: Show session history")
        print("- quit: Exit")
        print("=" * 50)

        while True:
            try:
                command = input(f"\n[{self.current_platform or 'general'}] > ").strip().lower()

                if command == "quit":
                    print("👋 Goodbye!")
                    break
                elif command == "chat":
                    await self._handle_chat()
                elif command == "generate":
                    await self._handle_generate()
                elif command == "project":
                    await self._handle_project()
                elif command == "search":
                    await self._handle_search()
                elif command == "knowledge":
                    await self._handle_knowledge()
                elif command == "tools":
                    await self._handle_tools()
                elif command == "platform":
                    await self._handle_platform()
                elif command == "history":
                    await self._handle_history()
                elif command == "help":
                    await self._show_help()
                else:
                    print("❓ Unknown command. Type 'help' for available commands.")

            except KeyboardInterrupt:
                print("\n👋 Exiting...")
                break
            except Exception as e:
                print(f"❌ Error: {e}")

    async def _handle_chat(self):
        """Handle general chat/questions"""
        question = input("Ask me anything about embedded systems: ").strip()
        if not question:
            return

        print("🤔 Thinking...")
        result = await self.agent.process_request(question, self.current_platform)

        if result["success"]:
            print(f"\n💡 {result['response']}")
            self.session_history.append({
                "type": "chat",
                "question": question,
                "response": result['response'][:100] + "..." if len(result['response']) > 100 else result['response'],
                "timestamp": result['timestamp']
            })
        else:
            print(f"❌ Error: {result['error']}")

    async def _handle_generate(self):
        """Handle code generation"""
        if not self.current_platform:
            platform = input("Platform (arduino/esp32/raspberry_pi): ").strip().lower()
        else:
            platform = self.current_platform
            print(f"Using current platform: {platform}")

        requirements = input("Describe what you want to build: ").strip()
        if not requirements:
            return

        print("⚡ Generating code...")

        request = f"Generate {platform} code for: {requirements}"
        result = await self.agent.process_request(request, platform)

        if result["success"]:
            print(f"\n✅ Code generated for {platform}:")
            print("=" * 50)
            print(result['response'])
            print("=" * 50)

            self.session_history.append({
                "type": "code_generation",
                "platform": platform,
                "requirements": requirements,
                "timestamp": result['timestamp']
            })

            # Ask if user wants to save
            save = input("\n💾 Save this code? (y/n): ").strip().lower()
            if save == 'y':
                filename = input("Filename (without extension): ").strip()
                if filename:
                    await self._save_code(result['response'], platform, filename)
        else:
            print(f"❌ Error: {result['error']}")

    async def _handle_project(self):
        """Handle complete project generation"""
        if not self.current_platform:
            platform = input("Platform (arduino/esp32/raspberry_pi): ").strip().lower()
        else:
            platform = self.current_platform
            print(f"Using current platform: {platform}")

        project_name = input("Project name: ").strip()
        requirements = input("Project requirements: ").strip()

        if not project_name or not requirements:
            print("⚠️ Project name and requirements are required")
            return

        print("🏗️ Creating complete project...")

        result = await self.agent.generate_project(platform, requirements, project_name)

        if result["success"]:
            print(f"\n✅ Project '{project_name}' created successfully!")
            print(f"📁 Location: {result['project_path']}")
            print(f"📄 Files created: {', '.join(result['files_created'])}")

            self.session_history.append({
                "type": "project_creation",
                "project_name": project_name,
                "platform": platform,
                "timestamp": datetime.now().isoformat()
            })
        else:
            print(f"❌ Error: {result['error']}")

    async def _handle_search(self):
        """Handle web search"""
        query = input("Search query: ").strip()
        if not query:
            return

        print("🔍 Searching...")

        # Use the web search through the agent
        search_request = f"Search for: {query}"
        result = await self.agent.process_request(search_request)

        if result["success"]:
            print(f"\n🔍 Search results:")
            print(result['response'])
        else:
            print(f"❌ Search error: {result['error']}")

    async def _handle_knowledge(self):
        """Handle knowledge base operations"""
        print("Knowledge base operations:")
        print("1. Add PDF document")
        print("2. Add text file")
        print("3. List knowledge files")

        choice = input("Choose option (1-3): ").strip()

        if choice == "1":
            file_path = input("PDF file path: ").strip()
            if file_path:
                success = await self.agent.add_knowledge(file_path)
                if success:
                    print("✅ PDF added to knowledge base")
                else:
                    print("❌ Failed to add PDF")

        elif choice == "2":
            file_path = input("Text file path: ").strip()
            if file_path:
                success = await self.agent.add_knowledge(file_path)
                if success:
                    print("✅ Text file added to knowledge base")
                else:
                    print("❌ Failed to add text file")

        elif choice == "3":
            kb_path = self.agent.tools_instance.knowledge_base_path
            if kb_path.exists():
                files = list(kb_path.rglob("*"))
                print(f"\n📚 Knowledge base contents ({len(files)} files):")
                for file in files[:10]:  # Show first 10
                    print(f"  - {file.name}")
                if len(files) > 10:
                    print(f"  ... and {len(files) - 10} more files")
            else:
                print("📚 Knowledge base is empty")

    async def _handle_tools(self):
        """Show available tools"""
        print("\n🔧 Available Tools:")
        print("=" * 30)
        tools_info = [
            ("🌐 web_search_tool", "Search web for embedded systems information"),
            ("🔌 component_lookup_tool", "Look up electronic components and sensors"),
            ("📌 pinout_lookup_tool", "Get pinout information for microcontrollers"),
            ("📝 code_template_tool", "Get code templates for different platforms"),
            ("✅ code_validator_tool", "Validate code syntax and structure"),
            ("📚 library_lookup_tool", "Look up library information"),
            ("📁 file_operations_tool", "File and directory operations"),
        ]

        for tool_name, description in tools_info:
            print(f"{tool_name}: {description}")

        print("=" * 30)

        # Tool usage example
        tool_demo = input("\nWant to try a tool? (component/pinout/template/n): ").strip().lower()

        if tool_demo == "component":
            component = input("Component name (e.g., DHT22, HC-SR04): ").strip()
            if component:
                request = f"Look up component information for {component}"
                result = await self.agent.process_request(request)
                print(f"\n{result.get('response', 'No information found')}")

        elif tool_demo == "pinout":
            platform = input("Platform (arduino_uno/esp32/raspberry_pi): ").strip()
            if platform:
                request = f"Show pinout information for {platform}"
                result = await self.agent.process_request(request)
                print(f"\n{result.get('response', 'No pinout found')}")

        elif tool_demo == "template":
            platform = input("Platform: ").strip()
            template_type = input("Template type (basic/sensor_reading/web_server): ").strip()
            if platform and template_type:
                request = f"Get {template_type} template for {platform}"
                result = await self.agent.process_request(request)
                print(f"\n{result.get('response', 'No template found')}")

    async def _handle_platform(self):
        """Set current platform"""
        platforms = ["arduino", "esp32", "raspberry_pi"]
        print(f"\nAvailable platforms: {', '.join(platforms)}")
        platform = input("Set current platform (or 'clear' to reset): ").strip().lower()

        if platform == "clear":
            self.current_platform = ""
            print("✅ Platform cleared")
        elif platform in platforms:
            self.current_platform = platform
            print(f"✅ Current platform set to: {platform}")
        else:
            print("❌ Invalid platform")

    async def _handle_history(self):
        """Show session history"""
        if not self.session_history:
            print("📝 No history yet")
            return

        print(f"\n📝 Session History ({len(self.session_history)} items):")
        print("=" * 40)

        for i, item in enumerate(self.session_history[-5:], 1):  # Show last 5
            print(f"{i}. [{item['type']}] {item.get('timestamp', 'Unknown time')}")
            if item['type'] == 'chat':
                print(f"   Q: {item['question'][:50]}...")
                print(f"   A: {item['response']}")
            elif item['type'] == 'code_generation':
                print(f"   Platform: {item['platform']}")
                print(f"   Requirements: {item['requirements'][:50]}...")
            elif item['type'] == 'project_creation':
                print(f"   Project: {item['project_name']} ({item['platform']})")
            print()

    async def _show_help(self):
        """Show detailed help"""
        help_text = """
🤖 Embedded Systems AI Agent Help
================================

COMMANDS:
- chat: Ask questions about embedded systems, get advice, troubleshoot issues
- generate: Generate code for Arduino, ESP32, or Raspberry Pi
- project: Create complete projects with code, documentation, and file structure
- search: Search the web for tutorials, datasheets, and documentation
- knowledge: Add PDF manuals and documentation to the knowledge base
- tools: Explore available tools (component lookup, pinouts, templates)
- platform: Set default platform to avoid retyping
- history: View your session activity
- help: Show this help message
- quit: Exit the application

PLATFORMS SUPPORTED:
- Arduino (Uno, Nano, Mega, etc.)
- ESP32 (WiFi, Bluetooth, sensors)
- Raspberry Pi (GPIO, camera, I2C, SPI)

EXAMPLE WORKFLOWS:
1. Set platform → Generate code → Validate → Save
2. Add knowledge PDFs → Ask specific questions → Get context-aware answers
3. Search for components → Get pinouts → Generate integration code
4. Create complete project → Get documentation → Save everything

TIPS:
- Use specific requirements for better code generation
- Add datasheets to knowledge base for better context
- Validate code before uploading to hardware
- Save projects for future reference
"""
        print(help_text)

    async def _save_code(self, response: str, platform: str, filename: str):
        """Save generated code to file"""
        try:
            # Extract code from response
            code = self.agent._extract_code_from_response(response)
            if not code:
                print("⚠️ No code found in response")
                return

            # Determine file extension
            if platform in ["arduino", "esp32"]:
                extension = ".ino"
            else:
                extension = ".py"

            filepath = f"{filename}{extension}"

            with open(filepath, 'w') as f:
                f.write(code)

            print(f"✅ Code saved to {filepath}")

        except Exception as e:
            print(f"❌ Error saving code: {e}")

"""# Main"""

async def main():
    """Main execution function"""
    print("🚀 Starting Embedded Systems AI Agent...")

    # Get API key from environment
    api_key = os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY")
    
    if not api_key or api_key == "your_api_key_here":
        print("❌ GEMINI_API_KEY is not set or is still the placeholder value.")
        print("Please update the .env file with your actual API key.")
        print("Get your API key from: https://aistudio.google.com/app/apikey")
        return

    try:
        # Initialize CLI
        cli = EmbeddedSystemsCLI(api_key)

        # Check if web search is available
        if WEB_SEARCH_AVAILABLE:
            print("✅ Web search enabled")
        else:
            print("⚠️ Web search disabled (install duckduckgo-search for web search)")

        # Run interactive session
        await cli.run_interactive_session()

    except Exception as e:
        print(f"❌ Failed to start agent: {e}")
        print("Make sure you have all required dependencies installed:")
        print("pip install langchain langchain-google-genai langgraph chromadb sentence-transformers PyPDF2 duckduckgo-search")

if __name__ == "__main__":
    asyncio.run(main())

# --- Hardware Digital Twin: MCU Hardware Registry ---
MCU_HARDWARE_REGISTRY = {
    "esp32": {
        "input_only_pins": [34, 35, 36, 39],
        "analog_input_pins": [0, 2, 4, 12, 13, 14, 15, 25, 26, 27, 32, 33, 34, 35, 36, 39],
        "pwm_capable_pins": list(range(0, 34)),  # Most GPIOs 0-33 support PWM
        "i2c_default_pins": {"sda": 21, "scl": 22},
        "max_io_voltage": 3.3
    },
    "arduino_uno": {
        "input_only_pins": [],
        "analog_input_pins": ["A0", "A1", "A2", "A3", "A4", "A5"],
        "pwm_capable_pins": [3, 5, 6, 9, 10, 11],
        "i2c_default_pins": {"sda": "A4", "scl": "A5"},
        "max_io_voltage": 5.0
    },
    "raspberry_pi_4": {
        "input_only_pins": [],
        "analog_input_pins": [],  # No native analog inputs
        "pwm_capable_pins": [12, 13, 18, 19],
        "i2c_default_pins": {"sda": 2, "scl": 3},
        "max_io_voltage": 3.3
    }
}

import re

def hardware_constraint_verifier(code: str, platform: str) -> dict:
    """Check code for hardware pin conflicts and capability mismatches using MCU_HARDWARE_REGISTRY."""
    report = {"errors": [], "warnings": [], "info": [], "pins_used": {}}
    registry = MCU_HARDWARE_REGISTRY.get(platform.lower())
    if not registry:
        report["errors"].append(f"Unknown platform: {platform}")
        return report

    # Regex patterns for pin usage (C++/Arduino & Python)
    pin_patterns = [
        r'pinMode\((\w+),\s*(OUTPUT|INPUT|INPUT_PULLUP)\)',
        r'digitalWrite\((\w+),',
        r'digitalRead\((\w+),',
        r'analogRead\((\w+),',
        r'analogWrite\((\w+),',
        r'GPIO\.setup\((\w+),',
        r'GPIO\.output\((\w+),',
        r'GPIO\.input\((\w+),',
        r'PWM\(.*?(\w+),',
        r'ledcWrite\((\d+),',
        r'ledcRead\((\d+),'
    ]

    # Check for pin conflicts and capabilities
    for pattern in pin_patterns:
        matches = re.findall(pattern, code)
        for match in matches:
            pin = match[0]
            if pin.isdigit():
                pin_num = int(pin)
            else:
                pin_num = pin  # Keep as string for GPIO names like 'D1', 'A0', etc.

            # Check if pin is valid for the platform
            if pin_num not in registry["analog_input_pins"] and pin_num not in registry["pwm_capable_pins"]:
                report["errors"].append(f"Pin {pin} is not valid for {platform}")

            # Check for input-only pins used as outputs
            if pin_num in registry["input_only_pins"]:
                if "digitalWrite" in pattern or "pinMode" in pattern and "OUTPUT" in match:
                    report["errors"].append(f"Pin {pin} is input-only and cannot be used as output")

            # Check for PWM on non-PWM pins
            if "analogWrite" in pattern or "ledcWrite" in pattern:
                if pin_num not in registry["pwm_capable_pins"]:
                    report["errors"].append(f"Pin {pin} does not support PWM")

            # Check for I2C default pins
            if pin == "21" or pin == "22":  # Common I2C SDA/SCL pins for ESP32
                if platform.lower() == "esp32":
                    report["info"].append(f"Pin {pin} is configured as default I2C {'SDA' if pin == '21' else 'SCL'}")
                else:
                    report["warnings"].append(f"Pin {pin} is typically used for I2C SDA/SCL on ESP32")

    return report