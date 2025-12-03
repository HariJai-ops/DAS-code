#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>
#include <BluetoothSerial.h>
#include <TinyGPS++.h>

#define LORA_SS         5
#define LORA_RST        14
#define LORA_DIO0       2
#define DHTPIN          4
#define DHTTYPE         DHT11
#define VIBRATION_PIN   35
#define GAS_SENSOR_PIN  34
#define GPS_RX          16
#define GPS_TX          17
#define ALERT_INPUT     32  // GPIO input from camera board ALERT_PIN
#define BUZZER_PIN      12

DHT dht(DHTPIN, DHTTYPE);
BluetoothSerial SerialBT;
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

unsigned long previousMillis = 0;
const long interval = 8000;
volatile bool motionAlertFlag = false;
bool motionDetectedStatus = false;

void IRAM_ATTR onMotionAlert() {
  motionAlertFlag = true;
  motionDetectedStatus = true;
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Rescue_Node");
  pinMode(ALERT_INPUT, INPUT_PULLDOWN);
  attachInterrupt(ALERT_INPUT, onMotionAlert, RISING);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  dht.begin();
  SPI.begin(18, 19, 23, 5);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(915E6)) {
    Serial.println("LoRa init failed!");
    while (true) delay(1000);
  }
  Serial.println("ESP32 main board ready");
}

void loop() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (motionAlertFlag) {
    Serial.println("Motion alert received! Sounding buzzer.");
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);
    motionAlertFlag = false;
  }

  // Reset motion status after interval
  if (motionDetectedStatus && millis() - previousMillis > interval) {
    motionDetectedStatus = false;
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    int vibration = digitalRead(VIBRATION_PIN);
    int gasLevel = analogRead(GAS_SENSOR_PIN);

    String latitude = "0", longitude = "0";
    if (gps.location.isValid()) {
      latitude = String(gps.location.lat(), 6);
      longitude = String(gps.location.lng(), 6);
    }
    String motionStatus = motionDetectedStatus ? "A:1" : "A:0";

    String message = "T:" + String(temperature, 1) +
                     " H:" + String(humidity, 1) +
                     " V:" + String(vibration) +
                     " G:" + String(gasLevel) +
                     " Lat:" + latitude +
                     " Lng:" + longitude +
                     " " + motionStatus;

    LoRa.beginPacket();
    LoRa.print(message);
    LoRa.endPacket();

    SerialBT.println(message);
    Serial.println("Data sent: " + message);
  }

  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('
');
    Serial.println("Bluetooth command: " + cmd);
    // Add command handling if needed
  }
}
