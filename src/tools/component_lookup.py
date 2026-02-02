"""
Component lookup tool for electronic components
"""

from langchain_core.tools import tool


COMPONENT_DB = {
    "dht22": {
        "type": "Temperature & Humidity Sensor",
        "description": "Digital temperature and humidity sensor with high accuracy",
        "voltage": "3.3-5V",
        "pins": ["VCC (Red)", "Data (Yellow)", "NC (Not Connected)", "GND (Black)"],
        "libraries": ["DHT", "Adafruit_DHT"],
        "arduino_code": '''#include <DHT.h>
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
  Serial.print("Humidity: "); Serial.print(h);
  Serial.print("%, Temperature: "); Serial.println(t);
  delay(2000);
}''',
        "raspberry_pi_code": '''import Adafruit_DHT
sensor = Adafruit_DHT.DHT22
pin = 4

humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)
if humidity is not None and temperature is not None:
    print(f'Temp: {temperature:.1f}C  Humidity: {humidity:.1f}%')'''
    },
    "hc-sr04": {
        "type": "Ultrasonic Distance Sensor",
        "description": "Measures distance using ultrasonic waves (2cm-400cm range)",
        "voltage": "5V",
        "pins": ["VCC", "Trig", "Echo", "GND"],
        "libraries": ["NewPing (Arduino)"],
        "arduino_code": '''#define trigPin 9
#define echoPin 8

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  long duration, distance;
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  Serial.print(distance); Serial.println(" cm");
  delay(1000);
}''',
        "raspberry_pi_code": '''import RPi.GPIO as GPIO
import time

TRIG, ECHO = 23, 24
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def get_distance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    while GPIO.input(ECHO) == 0: pulse_start = time.time()
    while GPIO.input(ECHO) == 1: pulse_end = time.time()
    return round((pulse_end - pulse_start) * 17150, 2)'''
    },
    "led": {
        "type": "Light Emitting Diode",
        "description": "Basic LED for visual indication",
        "voltage": "1.8-3.3V (with current limiting resistor)",
        "pins": ["Anode (+)", "Cathode (-)"],
        "arduino_code": '''int ledPin = 13;
void setup() { pinMode(ledPin, OUTPUT); }
void loop() {
  digitalWrite(ledPin, HIGH); delay(1000);
  digitalWrite(ledPin, LOW); delay(1000);
}''',
        "raspberry_pi_code": '''import RPi.GPIO as GPIO
import time
LED_PIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
try:
    while True:
        GPIO.output(LED_PIN, GPIO.HIGH); time.sleep(1)
        GPIO.output(LED_PIN, GPIO.LOW); time.sleep(1)
except KeyboardInterrupt:
    GPIO.cleanup()'''
    }
}


@tool
def component_lookup_tool(component_name: str) -> str:
    """Look up information about electronic components, sensors, and modules"""
    component_key = component_name.lower().replace("-", "").replace(" ", "")

    for key, info in COMPONENT_DB.items():
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

    return f"❌ Component '{component_name}' not found. Try: DHT22, HC-SR04, LED"
