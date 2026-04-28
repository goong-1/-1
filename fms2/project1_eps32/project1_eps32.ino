#include <WiFi.h>

// 💡 Wi-Fi 정보
const char* ssid = "asia-edu_2G";
const char* password = "12345678";

// 🌟 PC(Python 서버) 정보 설정
const char* serverIP = "192.168.0.139"; 
const uint16_t serverPort = 10000;

WiFiClient client;

void setup() {
  Serial.begin(115200); 
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // 아두이노 우노와 통신

  // Wi-Fi 연결
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ Wi-Fi 연결 성공!");
}

void loop() {
  // 1. 서버(PC) 연결 확인 및 자동 재접속
  if (!client.connected()) {
    if (client.connect(serverIP, serverPort)) {
      Serial.println("\n✅ PC 관제 센터와 연결되었습니다!");
    } else {
      delay(2000);
      return;
    }
  }

  // 2. 🌟 [PC -> ESP32 -> 아두이노] 모니터링 및 배달
  if (client.available()) {
    // 줄바꿈(\n)이 올 때까지 명령어 한 줄을 통째로 읽습니다.
    String cmdFromPC = client.readStringUntil('\n');
    
    // 1) 시리얼 모니터에 "나 이거 받았어!"라고 명확히 출력
    Serial.print("👉 [FROM PC]: ");
    Serial.println(cmdFromPC); 

    // 2) 아두이노(Uno)에게 줄바꿈을 붙여서 전달
    Serial2.println(cmdFromPC); 
  }

  // 3. [아두이노 -> ESP32 -> PC] 피드백 배달
  if (Serial2.available()) {
    String feedbackFromUno = Serial2.readStringUntil('\n');
    
    // 1) PC(소켓)로 전달
    client.println(feedbackFromUno);

    // 2) 시리얼 모니터에 출력
   // Serial.print("👈 [FROM UNO]: ");
    // Serial.println(feedbackFromUno);
  }
}