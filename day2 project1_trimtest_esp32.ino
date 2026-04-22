#include <WiFi.h>
#include <WebServer.h>

// 💡 본인의 Wi-Fi 정보로 수정하세요!
const char* ssid = "asia-edu_2G";
const char* password = "12345678";

WebServer server(80);

int currentSpeed = 150;
float leftTrim = 1.0; 

// ================= [웹 대시보드 HTML/JS/CSS] =================
// (이전과 동일한 웹 대시보드 UI 코드입니다. 그대로 복사하시면 됩니다.)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="ko">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>로봇 관제 대시보드</title>
  <style>
    body { font-family: 'Segoe UI', sans-serif; background-color: #1e1e1e; color: #fff; text-align: center; margin: 0; padding: 20px; }
    .card { background: #2c2c2c; border-radius: 10px; padding: 20px; max-width: 400px; margin: 10px auto; }
    .btn { background-color: #3a3a3a; color: white; border: none; padding: 15px; font-size: 18px; border-radius: 10px; cursor: pointer; margin: 5px; user-select: none; }
    .btn:active { background-color: #4CAF50; }
    .speed-val { font-size: 32px; color: #4CAF50; font-weight: bold; margin: 10px 0; }
    .dpad { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; max-width: 200px; margin: 0 auto; }
    input[type=range] { width: 80%; margin: 10px 0; }
  </style>
</head>
<body>
  <h2>🚀 로봇 제어 시스템</h2>

  <div class="card">
    <h3>좌측 바퀴 보정 (Left Trim)</h3>
    <input type="range" min="0.5" max="1.0" step="0.01" value="1.0" oninput="changeTrim(this.value)">
    <div id="trimDisp" style="font-size: 20px; color: #ffb300;">1.00 (100%)</div>
  </div>

  <div class="card">
    <h3>현재 출력 (Speed)</h3>
    <div class="speed-val" id="speedDisp">150</div>
    <button class="btn" onclick="changeSpeed(10)" style="width: 80px; background: #45a049;">가속 (+)</button>
    <button class="btn" onclick="changeSpeed(-10)" style="width: 80px; background: #e53935;">감속 (-)</button>
  </div>

  <div class="card">
    <h3>주행 제어 (WASD)</h3>
    <div class="dpad">
      <div></div>
      <button class="btn" id="btnW" onmousedown="sendCmd('w')" onmouseup="sendCmd('x')" ontouchstart="sendCmd('w')" ontouchend="sendCmd('x')">W</button>
      <div></div>
      <button class="btn" id="btnA" onmousedown="sendCmd('a')" onmouseup="sendCmd('x')" ontouchstart="sendCmd('a')" ontouchend="sendCmd('x')">A</button>
      <button class="btn" id="btnS" onmousedown="sendCmd('s')" onmouseup="sendCmd('x')" ontouchstart="sendCmd('s')" ontouchend="sendCmd('x')">S</button>
      <button class="btn" id="btnD" onmousedown="sendCmd('d')" onmouseup="sendCmd('x')" ontouchstart="sendCmd('d')" ontouchend="sendCmd('x')">D</button>
    </div>
    <button class="btn" style="width: 100%; background: #d32f2f; margin-top: 15px;" onclick="sendCmd('x')">긴급 정지 (X)</button>
  </div>

  <script>
    let currentSpeed = 150;
    let keys = {};

    function sendCmd(action) { fetch(`/action?go=${action}`); }

    function changeSpeed(delta) {
      currentSpeed = Math.min(255, Math.max(0, currentSpeed + delta));
      document.getElementById('speedDisp').innerText = currentSpeed;
      fetch(`/setSpeed?val=${currentSpeed}`);
    }

    function changeTrim(val) {
      document.getElementById('trimDisp').innerText = val + " (" + Math.round(val*100) + "%)";
      fetch(`/setTrim?val=${val}`);
    }

    document.addEventListener('keydown', (e) => {
      const key = e.key.toLowerCase();
      if(['w', 'a', 's', 'd', 'x'].includes(key) && !keys[key]) {
        keys[key] = true;
        sendCmd(key);
        document.getElementById('btn' + key.toUpperCase())?.style.setProperty('background-color', '#4CAF50');
      }
    });

    document.addEventListener('keyup', (e) => {
      const key = e.key.toLowerCase();
      if(['w', 'a', 's', 'd'].includes(key)) {
        keys[key] = false;
        sendCmd('x');
        document.getElementById('btn' + key.toUpperCase())?.style.setProperty('background-color', '#3a3a3a');
      }
    });
  </script>
</body>
</html>
)rawliteral";
// =============================================================

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); }
  Serial.println("\n✅ Wi-Fi 연결 성공!");
  Serial.print("웹페이지 주소: "); Serial.println(WiFi.localIP());

  server.on("/", []() { server.send(200, "text/html", index_html); });

  server.on("/setSpeed", []() {
    if (server.hasArg("val")) currentSpeed = server.arg("val").toInt();
    server.send(200, "text/plain", "OK");
  });

  server.on("/setTrim", []() {
    if (server.hasArg("val")) leftTrim = server.arg("val").toFloat();
    server.send(200, "text/plain", "OK");
  });

  // 🌟 [수정됨] 아두이노 우노의 a,d 프로토콜에 완벽히 맞춘 번역기
  server.on("/action", []() {
    if (server.hasArg("go")) {
      String go = server.arg("go");
      
      int L = currentSpeed * leftTrim; 
      int R = currentSpeed;
      
      String cmd = "a0,d0\n"; // 기본 정지

      // 사용자님의 우노 파싱 규칙(a~~,d~~\n)에 맞게 문자열을 조립합니다.
      if(go == "w")      cmd = "a" + String(L) + ",d" + String(R) + "\n";
      else if(go == "s") cmd = "a-" + String(L) + ",d-" + String(R) + "\n";
      else if(go == "a") cmd = "a-" + String(L) + ",d" + String(R) + "\n"; // 제자리 좌회전
      else if(go == "d") cmd = "a" + String(L) + ",d-" + String(R) + "\n"; // 제자리 우회전
      else if(go == "x") cmd = "a0,d0\n"; // 정지

      Serial2.print(cmd); // 우노로 쏘기!
      Serial.print("우노로 전송: "); Serial.print(cmd); // 디버깅용 모니터
    }
    server.send(200, "text/plain", "OK");
  });

  server.begin();
}

void loop() {
  server.handleClient();
}
