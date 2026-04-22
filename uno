// [1. 핀 설정] 
// 🌟 만약 'W'를 눌렀는데 뒤로 간다면, A와 B의 핀 번호를 서로 바꾸세요!
const int leftMotorA = 10;  // 기존 10 -> 11로 수정 (방향 반전 대응)
const int leftMotorB = 11;  // 기존 11 -> 10으로 수정
const int rightMotorA = 5;  // 기존 5 -> 6으로 수정
const int rightMotorB = 6;  // 기존 6 -> 5으로 수정

// 엔코더 핀 (2, 3번은 우노의 인터럽트 전용 핀입니다)
const int leftEncoderPin = 2;
const int rightEncoderPin = 3;

// [2. 변수 설정]
volatile long leftTicks = 0;
volatile long rightTicks = 0;

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 50; 

void setup() {
  // 🌟 ESP32와 통신하는 속도 (ESP32의 Serial2.begin(9600...)과 일치해야 함)
  Serial.begin(9600);
  
  pinMode(leftMotorA, OUTPUT); pinMode(leftMotorB, OUTPUT);
  pinMode(rightMotorA, OUTPUT); pinMode(rightMotorB, OUTPUT);

  pinMode(leftEncoderPin, INPUT_PULLUP);
  pinMode(rightEncoderPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), countLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), countRight, CHANGE);
}

void loop() {
  // --- [수신] ESP32 -> 아두이노 (모터 제어) ---
  if (Serial.available() > 0) {
    // 1. 줄바꿈(\n)이 올 때까지 데이터를 읽음 (예: "a80d80")
    String data = Serial.readStringUntil('\n');
    
    // 2. 식별자 'a'와 'd'의 위치를 찾음 (이것이 동료가 말한 진짜 파싱!)
    int aIdx = data.indexOf('a');
    int dIdx = data.indexOf('d');
    
    // 3. 'a'와 'd'가 모두 문자열 안에 들어있는지 확인
    if (aIdx != -1 && dIdx != -1) {
      // a 바로 다음부터 d 직전까지 숫자를 잘라냄 (좌측 모터)
      int leftSpeed = data.substring(aIdx + 1, dIdx).toInt();
      
      // d 바로 다음부터 끝까지 숫자를 잘라냄 (우측 모터)
      int rightSpeed = data.substring(dIdx + 1).toInt();
      
      // 4. 추출된 숫자로 실제 모터 구동
      driveMotors(leftSpeed, rightSpeed);
      
      // (선택 사항) 제대로 파싱됐는지 시리얼 모니터로 확인
      Serial.print("L_Cmd:"); Serial.print(leftSpeed);
      Serial.print(" R_Cmd:"); Serial.println(rightSpeed);
    }
  }

  // --- [송신] 아두이노 -> ESP32 (엔코더 피드백) ---
  if (millis() - lastSendTime >= sendInterval) {
    lastSendTime = millis();
    
    // "E:좌측틱,우측틱" 형태로 PC/ESP32에 보고
    Serial.print("E:");
    Serial.print(leftTicks);
    Serial.print(",");
    Serial.println(rightTicks);
  }
}

// 모터 구동 핵심 함수 (부호에 따라 방향 결정)
void driveMotors(int left, int right) {
  // 좌측 모터 제어
  if (left >= 0) { // 양수(+)면 전진
    analogWrite(leftMotorA, left); 
    analogWrite(leftMotorB, 0);
  } else { // 음수(-)면 후진
    analogWrite(leftMotorA, 0); 
    analogWrite(leftMotorB, -left); // 음수를 양수로 바꿔서 출력
  }
  
  // 우측 모터 제어
  if (right >= 0) {
    analogWrite(rightMotorA, right); 
    analogWrite(rightMotorB, 0);
  } else {
    analogWrite(rightMotorA, 0); 
    analogWrite(rightMotorB, -right);
  }
}

// 인터럽트 서비스 루틴 (회전 수 카운트)
void countLeft() { leftTicks++; }
void countRight() { rightTicks++; }
