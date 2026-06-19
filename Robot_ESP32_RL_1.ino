#include <esp_now.h>
#include <WiFi.h>

// --- KİMLİK BİLGİSİ ---
const int MY_ROBOT_ID = 1; // Robot 1 için ID

// --- PİN TANIMLAMALARI (Senin çalışan kusursuz dizilimin) ---
// SAĞ MOTOR -> KANAL A
const int motorRightA = 17;
const int motorRightB = 16;
const int enableRight = 18;

// SOL MOTOR -> KANAL B
const int motorLeftA = 21;
const int motorLeftB = 22;
const int enableLeft = 19;

const int stbyPin = 23;

// --- VERİ PAKETİ ŞABLONU (Komutan ile birebir aynı olmalı) ---
typedef struct struct_message {
  int id;
  int leftSpeed;
  int rightSpeed;
} struct_message;

struct_message myData;

// --- FAILSAFE (DEAD-MAN'S SWITCH) DEĞİŞKENLERİ ---
unsigned long lastCommandTime = 0;
bool isMoving = false;

// --- MOTOR SÜRÜŞ FONKSİYONU ---
void driveMotors(int leftSpeed, int rightSpeed) {
  // Python'dan (RL_Swarm_GCS) gelen -30 ile 30 arası hız komutları (veya -90 ile 90 klavye)
  // ESP32'nin anladığı 0-255 PWM gücüne çevir
  int mappedLeft = map(leftSpeed, -90, 90, -255, 255);
  int mappedRight = map(rightSpeed, -90, 90, -255, 255);

  // Sol Motor (Kanal B)
  if (mappedLeft > 0) {
    digitalWrite(motorLeftA, HIGH);
    digitalWrite(motorLeftB, LOW);
  } else if (mappedLeft < 0) {
    digitalWrite(motorLeftA, LOW);
    digitalWrite(motorLeftB, HIGH);
  } else {
    digitalWrite(motorLeftA, LOW);
    digitalWrite(motorLeftB, LOW);
  }
  analogWrite(enableLeft, abs(mappedLeft));

  // Sağ Motor (Kanal A)
  if (mappedRight > 0) {
    digitalWrite(motorRightA, HIGH);
    digitalWrite(motorRightB, LOW);
  } else if (mappedRight < 0) {
    digitalWrite(motorRightA, LOW);
    digitalWrite(motorRightB, HIGH);
  } else {
    digitalWrite(motorRightA, LOW);
    digitalWrite(motorRightB, LOW);
  }
  analogWrite(enableRight, abs(mappedRight));
}

// --- ESP-NOW: YENİ SÜRÜME (CORE 3.X) UYGUN VERİ ALMA FONKSİYONU ---
void OnDataRecv(const esp_now_recv_info * info, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  
  if (myData.id == MY_ROBOT_ID) {
    driveMotors(myData.leftSpeed, myData.rightSpeed);
    lastCommandTime = millis();
    isMoving = (myData.leftSpeed != 0 || myData.rightSpeed != 0);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(motorRightA, OUTPUT);
  pinMode(motorRightB, OUTPUT);
  pinMode(enableRight, OUTPUT);
  pinMode(motorLeftA, OUTPUT);
  pinMode(motorLeftB, OUTPUT);
  pinMode(enableLeft, OUTPUT);
  pinMode(stbyPin, OUTPUT);

  digitalWrite(stbyPin, HIGH);
  driveMotors(0, 0);

  WiFi.mode(WIFI_MODE_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Baslatilamadi!");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  Serial.println(String("Robot Asker (ID ") + String(MY_ROBOT_ID) + String(") Hazir. RL Beyninden Emir Bekleniyor!"));
}

void loop() {
  if (isMoving && (millis() - lastCommandTime > 500)) {
    driveMotors(0, 0);
    isMoving = false;
    Serial.println("[FAILSAFE] Baglanti Koptu! Motorlar Kilitlendi.");
  }
  delay(1); 
}
