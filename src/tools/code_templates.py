"""
Code templates for different platforms
"""

from langchain_core.tools import tool


TEMPLATES = {
    "arduino": {
        "basic": '''// Arduino Basic Template
void setup() {
    Serial.begin(9600);
    Serial.println("Arduino Started!");
}

void loop() {
    Serial.println("Hello World!");
    delay(1000);
}''',
        "sensor": '''// Arduino Sensor Template
const int sensorPin = A0;
const int ledPin = 13;

void setup() {
    Serial.begin(9600);
    pinMode(ledPin, OUTPUT);
}

void loop() {
    int value = analogRead(sensorPin);
    float voltage = value * (5.0 / 1023.0);
    Serial.print("Value: "); Serial.println(value);
    digitalWrite(ledPin, value > 512 ? HIGH : LOW);
    delay(500);
}'''
    },
    "esp32": {
        "basic": '''// ESP32 Basic Template
#include <WiFi.h>

const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\\nConnected! IP: " + WiFi.localIP().toString());
}

void loop() {
    Serial.println("ESP32 running...");
    delay(5000);
}''',
        "webserver": '''// ESP32 Web Server
#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
WebServer server(80);

void handleRoot() {
    server.send(200, "text/html", "<h1>ESP32 Server</h1>");
}

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) delay(1000);
    server.on("/", handleRoot);
    server.begin();
}

void loop() { server.handleClient(); }'''
    },
    "raspberry_pi": {
        "basic": '''#!/usr/bin/env python3
import time

def main():
    print("Raspberry Pi Started!")
    while True:
        print("Running...")
        time.sleep(2)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\\nExiting...")''',
        "gpio": '''#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

LED_PIN = 18
BUTTON_PIN = 2

GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

try:
    while True:
        if GPIO.input(BUTTON_PIN) == GPIO.LOW:
            GPIO.output(LED_PIN, not GPIO.input(LED_PIN))
            time.sleep(0.3)
except KeyboardInterrupt:
    GPIO.cleanup()'''
    }
}


@tool
def code_template_tool(platform: str, template_type: str) -> str:
    """Get code templates for different platforms and use cases"""
    platform_key = platform.lower()
    template_key = template_type.lower()

    if platform_key in TEMPLATES:
        if template_key in TEMPLATES[platform_key]:
            code = TEMPLATES[platform_key][template_key]
            return f"📝 **{platform.title()} {template_type.title()} Template**\n\n```\n{code}\n```"
        return f"❌ Template '{template_type}' not found. Available: {', '.join(TEMPLATES[platform_key].keys())}"
    return f"❌ Platform '{platform}' not found. Available: {', '.join(TEMPLATES.keys())}"
