#include <esp_now.h>
#include <WiFi.h>

// =====================================================================
//  ROBOTA ÖZEL DONANIM VE KALİBRASYON AYARLARI
// =====================================================================
const int MY_ROBOT_ID = 2; // Bu robotun kimliği (Takipçi 2)

// Hız Denkleştirme: Motorlar çok hızlıysa düşür, yavaşsa artır.
const float HIZ_CARPANI = 0.5; // R2: standart

// Motor Terslik Düzeltmeleri: Bağlantılardan (L298N) kaynaklı ters 
// hareketleri düzeltmek için kablo sökmeden buraları 'true' yapabilirsiniz.
const bool SOL_MOTOR_TERS = false; 
const bool SAG_MOTOR_TERS = false;

// =====================================================================
// PİN TANIMLAMALARI
// =====================================================================
// SAĞ MOTOR -> KANAL A
const int motorRightA = 17;
const int motorRightB = 16;
const int enableRight = 18;

// SOL MOTOR -> KANAL B
const int motorLeftA = 21;
const int motorLeftB = 22;
const int enableLeft = 19;

const int stbyPin = 23;

// --- VERİ PAKETİ ŞABLONU ---
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
  // 1. DONANIM KALİBRASYONU (HIZ VE YÖN) — FLOAT matematik!
  // (Eski int*0.5 KESME yapiyordu: PC'nin 24 ve 25 komutlari AYNI duty'ye
  //  dusuyordu -> PWM cozunurlugu yariya iniyordu. Float + yuvarlama ile
  //  her PC adimi motorda gercek bir adim olur.)
  float l = leftSpeed * HIZ_CARPANI;
  float r = rightSpeed * HIZ_CARPANI;

  if (SOL_MOTOR_TERS) l = -l;
  if (SAG_MOTOR_TERS) r = -r;

  // 2. PWM MAPPING: giris -30..30 -> duty -180..180 (x6), float carpim + yuvarla
  int mappedLeft  = (int)lroundf(l * 6.0f);
  int mappedRight = (int)lroundf(r * 6.0f);

  // Sınırlandırma
  mappedLeft = constrain(mappedLeft, -180, 180);
  mappedRight = constrain(mappedRight, -180, 180);

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

// --- ESP-NOW VERİ ALMA FONKSİYONU ---
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
  Serial.println(String("Robot (ID ") + String(MY_ROBOT_ID) + String(") Hazir. Kalibrasyon Aktif!"));
}

void loop() {
  if (isMoving && (millis() - lastCommandTime > 500)) {
    driveMotors(0, 0);
    isMoving = false;
    Serial.println("[FAILSAFE] Baglanti Koptu! Motorlar Kilitlendi.");
  }
  delay(1); 
}
