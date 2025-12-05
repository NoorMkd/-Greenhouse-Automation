#include <Arduino.h>
#include <DHT.h>
#include <Wire.h>
#include <BH1750.h>
#include <ESP32Servo.h>

// ------------------- PINS -------------------
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

BH1750 lightMeter;

// Actuators
#define LED_R 18
#define LED_G 25
#define LED_B 14
#define BUZZER 33
#define RELAY_POMPE 12
#define RELAY_VENTILO 15
#define SERVO_PIN 27

// Sensors
#define FLAME_PIN 13      // digital (D0 pin of flame sensor)
#define SOIL_PIN 34       // analog
#define GAS_PIN 35        // analog
#define RAIN_DO 26        // digital (active LOW)
#define ANEMO_PIN 32      // analog wind sensor

// Thresholds
#define LUX_SEUIL 50      // Below this value, turn on RGB
#define SOIL_SEUIL 2000   // If soil is dry (higher value = drier)
#define TEMP_VENTILO 21  // Temperature threshold for fan
#define HUM_VENTILO 32   // Humidity threshold for fan
#define GAS_SEUIL 700    // Gas detection threshold
#define WIND_THRESHOLD 10 // km/h

// Servo
Servo myservo;

// Function to read wind speed from analog anemometer
float readWindSpeed() {
  int raw = analogRead(ANEMO_PIN);
  float voltage = raw * (3.3f / 4095.0f);
  float windKMH = (voltage / 5.0) * 252;
  
  // Ensure non-negative value
  if (windKMH < 0) windKMH = 0;
  
  return windKMH;
}

void setup() {
  Serial.begin(115200);
  Serial.println("=== SMART GREENHOUSE SYSTEM ===");

  // Initialize I2C
  Wire.begin(21, 22);
  
  // Initialize BH1750
  delay(100);
  lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
  Serial.println("BH1750: OK");
  
  // Initialize DHT
  delay(100);
  dht.begin();
  Serial.println("DHT11: OK");

  // Outputs
  pinMode(RELAY_POMPE, OUTPUT);
  pinMode(RELAY_VENTILO, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  // Inputs - FLAME_PIN is digital input
  pinMode(FLAME_PIN, INPUT);
  pinMode(RAIN_DO, INPUT_PULLUP);
  
  // Initialize servo
  myservo.attach(SERVO_PIN);
  myservo.write(0);

  // Initial state: all OFF
  digitalWrite(RELAY_POMPE, LOW);
  digitalWrite(RELAY_VENTILO, LOW);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);
  digitalWrite(BUZZER, LOW);

  delay(2000);
  Serial.println("=== SYSTEM READY ===\n");
}

void loop() {
  // ---------- READ SENSORS ----------
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();
  float lux = lightMeter.readLightLevel();
  int soil = analogRead(SOIL_PIN);
  int flame = digitalRead(FLAME_PIN);  // Digital read for flame sensor
  int gas = analogRead(GAS_PIN);
  int rain = digitalRead(RAIN_DO);
  float wind = readWindSpeed();

  // Validate DHT readings
  if (isnan(temp) || isnan(hum)) {
    Serial.println("ERROR: DHT sensor reading failed!");
    temp = 25.0;
    hum = 50.0;
  }

  // Display sensor readings
  Serial.println("------ SENSOR READINGS ------");
  Serial.printf("Temperature: %.1f C\n", temp);
  Serial.printf("Humidity: %.1f %%\n", hum);
  Serial.printf("Soil moisture: %d\n", soil);
  Serial.printf("Light level: %.1f lx\n", lux);
  Serial.printf("Gas level: %d\n", gas);
  Serial.printf("Flame detected: %s\n", flame == LOW ? "YES" : "NO");  // Flame sensor is active LOW
  Serial.printf("Rain detected: %s\n", rain == LOW ? "YES" : "NO");
  Serial.printf("Wind speed: %.1f km/h\n", wind);
  Serial.println("----------------------------");

  // ---------- RULE 1: Automatic watering ----------
  // If soil is dry -> pump ON
  if (soil > SOIL_SEUIL) {
    digitalWrite(RELAY_POMPE, HIGH);
    Serial.println("PUMP: ON (Soil is dry)");
  } else {
    digitalWrite(RELAY_POMPE, LOW);
    Serial.println("PUMP: OFF (Soil is wet enough)");
  }

  // ---------- RULE 2: Fan control ----------
  // If temperature > 27 AND humidity > 50 -> fan ON
  if (temp > TEMP_VENTILO && hum > HUM_VENTILO) {
    digitalWrite(RELAY_VENTILO, HIGH);
    Serial.println("FAN: ON (High temperature and humidity)");
  } else {
    digitalWrite(RELAY_VENTILO, LOW);
    Serial.println("FAN: OFF");
  }

  // ---------- RULE 3: Light control ----------
  // If light is low -> RGB ON
  if (lux < LUX_SEUIL) {
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, HIGH);
    Serial.println("RGB LIGHTS: ON (Low light)");
  } else {
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);
    Serial.println("RGB LIGHTS: OFF");
  }

  // ---------- RULE 4: Rain/Wind detection ----------
  // Rain detected OR wind > 15 km/h -> servo to 90°
  if (rain == LOW || wind > WIND_THRESHOLD) {
    myservo.write(90);
    if (rain == LOW) Serial.println("SERVO: 90 degrees (Rain detected)");
    if (wind > WIND_THRESHOLD) Serial.printf("SERVO: 90 degrees (Strong wind: %.1f km/h)\n", wind);
  } else {
    myservo.write(0);
    Serial.println("SERVO: 0 degrees (Normal)");
  }

  // ---------- RULE 5: Gas/Flame detection ----------
  // Gas detected OR flame detected -> buzzer ON
  if (gas > GAS_SEUIL || flame == LOW) {  // Flame sensor is active LOW
    digitalWrite(BUZZER, HIGH);
    if (gas > GAS_SEUIL) Serial.printf("BUZZER ALARM: Gas detected! (%d > %d)\n", gas, GAS_SEUIL);
    if (flame == LOW) Serial.println("BUZZER ALARM: Flame detected!");
  } else {
    digitalWrite(BUZZER, LOW);
    Serial.println("BUZZER: OFF (Normal)");
  }

  Serial.println();  // Empty line for separation
  delay(2000);
}