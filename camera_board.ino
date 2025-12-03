#include <WiFi.h>
#include <HardwareSerial.h>
#include <FS.h>
#include <SPIFFS.h>
#include <WebServer.h>

// WiFi credentials - UPDATE THESE BEFORE FLASHING
const char* ssid = "YOUR_WIFI_NAME";
const char* password = "YOUR_WIFI_PASSWORD";

// Camera serial (GPIO16=RX2, GPIO17=TX2)
HardwareSerial cameraSerial(2);

#define MOTION_PIN 4
#define ALERT_PIN 14

WebServer server(80);

// ZH PCV06 commands
const byte CMD_RESET[]   = {0x56, 0x00, 0x26, 0x00};
const byte CMD_CAPTURE[] = {0x56, 0x00, 0x36, 0x01, 0x00};
const byte CMD_GET_PICTURE[] = {0x56, 0x00, 0x32, 0x00, 0x00};
// To be filled before each chunk:
byte CMD_READ_IMAGE[16] = {0x56,0x00,0x32,0x0C,0x00,0x0A,
                           0x00,0x00,0x00,0x00,0x00,0x00,
                           0x00,0x00,0x00,0x00};

void setup() {
  Serial.begin(115200);
  cameraSerial.begin(38400, SERIAL_8N1, 16, 17);
  pinMode(MOTION_PIN, INPUT);
  pinMode(ALERT_PIN, OUTPUT);
  digitalWrite(ALERT_PIN, LOW);

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS failed!");
    while(1);
  }

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println();
  Serial.print("ESP32 IP Address: "); Serial.println(WiFi.localIP());

  resetCamera();

  // HTTP /photo.jpg endpoint
  server.on("/photo.jpg", HTTP_GET, handlePhotoRequest);
  server.begin();
  Serial.println("Camera board ready");
}

void loop() {
  server.handleClient();

  static bool lastMotion = false;
  static unsigned long lastAlert = 0;
  const unsigned long alertCooldown = 5000;

  bool motionDetected = digitalRead(MOTION_PIN);

  if (motionDetected && !lastMotion) {
    if (millis() - lastAlert > alertCooldown) {
      Serial.println("Motion detected, capturing image...");
      if (captureAndSaveImage()) {
        sendAlert();
        lastAlert = millis();
      }
    }
  }
  lastMotion = motionDetected;
  delay(100);
}

void resetCamera() {
  cameraSerial.write(CMD_RESET, sizeof(CMD_RESET));
  delay(3000);
  Serial.println("Camera reset done");
}

// Capture and save JPEG to SPIFFS
bool captureAndSaveImage() {
  flushCameraSerial();
  cameraSerial.write(CMD_CAPTURE, sizeof(CMD_CAPTURE));
  delay(300); // Wait for capture

  // Get picture
  flushCameraSerial();
  cameraSerial.write(CMD_GET_PICTURE, sizeof(CMD_GET_PICTURE));
  delay(100);

  int imgLen = getJPEGLength();
  if (imgLen <= 0) {
    Serial.println("Invalid JPEG length");
    return false;
  }
  Serial.print("JPEG Size: "); Serial.println(imgLen);

  if (readAndSaveJPEG(imgLen)) {
    Serial.println("Image saved to /photo.jpg");
    return true;
  } else {
    Serial.println("Failed to read/save JPEG");
    return false;
  }
}

void handlePhotoRequest() {
  File photo = SPIFFS.open("/photo.jpg", "r");
  if (!photo) {
    server.send(404, "text/plain", "No photo");
    return;
  }
  server.streamFile(photo, "image/jpeg");
  photo.close();
}

// --- Camera helpers ---

int getJPEGLength() {
  unsigned long timeout = millis() + 1500;
  int len = -1;
  while (millis() < timeout) {
    if (cameraSerial.available() && cameraSerial.read() == 0x76) {
      // skip to length field
      for (int i = 0; i < 7; ++i) cameraSerial.read();
      int b0 = cameraSerial.read();
      int b1 = cameraSerial.read();
      int b2 = cameraSerial.read();
      int b3 = cameraSerial.read();
      len = (b0 << 24) | (b1 << 16) | (b2 << 8) | b3;
      break;
    }
  }
  return len;
}

bool readAndSaveJPEG(int imgLen) {
  File file = SPIFFS.open("/photo.jpg", "w");
  if (!file) return false;

  int offset = 0;
  int blockSize = 512; // Try to keep this modest (RAM/speed balance)
  while (offset < imgLen) {
    int need = min(blockSize, imgLen - offset);

    CMD_READ_IMAGE[8] = (offset >> 8) & 0xFF;
    CMD_READ_IMAGE[9] = offset & 0xFF;
    CMD_READ_IMAGE[10] = 0x00;
    CMD_READ_IMAGE[11] = 0x00;
    CMD_READ_IMAGE[12] = (need >> 8) & 0xFF;
    CMD_READ_IMAGE[13] = need & 0xFF;

    cameraSerial.write(CMD_READ_IMAGE, sizeof(CMD_READ_IMAGE));
    delay(50);

    skipUntilStartOfJPEG();
    for (int i = 0; i < need; i++) {
      int c = readSerialWithTimeout(100);
      if (c < 0) { file.close(); return false; }
      file.write((uint8_t)c);
    }
    offset += need;
  }
  file.close();
  return true;
}

void flushCameraSerial() {
  while (cameraSerial.available()) cameraSerial.read();
}

void skipUntilStartOfJPEG() {
  while (cameraSerial.available()) {
    if (cameraSerial.peek() == 0xFF) break;
    cameraSerial.read();
  }
}

int readSerialWithTimeout(unsigned short timeoutMs) {
  unsigned long deadline = millis() + timeoutMs;
  while (millis() < deadline) {
    if (cameraSerial.available())
      return cameraSerial.read();
  }
  return -1;
}

void sendAlert() {
  digitalWrite(ALERT_PIN, HIGH);
  delay(300);
  digitalWrite(ALERT_PIN, LOW);
  Serial.println("Alert sent to main board");
}
