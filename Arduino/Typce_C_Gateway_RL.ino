/*
 * ============================================
 * GATEWAY ESP32 - OPTIMIZE EDILMIS SURUM (RL Uyumlu)
 * ============================================
 * Python'dan (RL_Swarm_GCS) %100 saf performans icin tasarlandi.
 * - CPU isinma sorunu cozuldu (80MHz + delay(1))
 * - Tumuyle gereksiz debug printleri kaldirildi.
 * - Seri haberlesme hizlandirildi.
 * - RL modeline sensor verisi gonderebilme gucune sahip (Gelecek icin)
 * ============================================
 */

#include <esp_now.h>
#include <WiFi.h>

// --- BROADCAST (Tum robotlara gonderir) ---
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// --- VERİ PAKETİ ŞABLONU (Gateway -> Robot) ---
typedef struct struct_message {
  int id;
  int leftSpeed;
  int rightSpeed;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

// --- ROBOTTAN GELEN VERİYİ DİNLEME (Gateway -> Python GCS) ---
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  // Gelen veriyi direkt olarak Python'un (PPO Modelinin) okumasi icin Seri Porta bas
  char buffer[len + 1];
  memcpy(buffer, incomingData, len);
  buffer[len] = '\0';
  Serial.println(buffer);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10); // COM Gecikmesini engeller
  
  // CPU Frekansi: 80Mhz (Isiyi inanilmaz dusurur)
  setCpuFrequencyMhz(80);
  
  WiFi.mode(WIFI_MODE_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm); // MENZIL ARTIRICI
  
  if (esp_now_init() != ESP_OK) {
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  esp_now_add_peer(&peerInfo);
}

void loop() {
  // --- GATEWAY ANA GOREVI ---
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();
    
    if (data.startsWith("<") && data.endsWith(">")) {
      data = data.substring(1, data.length() - 1);
      
      int firstComma = data.indexOf(',');
      int secondComma = data.indexOf(',', firstComma + 1);
      
      if (firstComma > 0 && secondComma > 0) {
        myData.id = data.substring(0, firstComma).toInt();
        myData.leftSpeed = data.substring(firstComma + 1, secondComma).toInt();
        myData.rightSpeed = data.substring(secondComma + 1).toInt();
        
        esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
      }
    }
  }
  
  // Isinmayi onleme
  if (Serial.available() == 0) {
    delay(1);
  }
}
