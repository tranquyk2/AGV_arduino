// ============================================
//  AGV MAGNETIC LINE FOLLOWER - 4 MOTOR (PID FIXED)
//  Driver: IBT-2 (BTS7960) x2
//  Arduino Mega 2560
//  ĐÃ SỬA: Bỏ cảm biến ngoài cùng index 0 và 7
// ============================================

// --- CẢM BIẾN TRƯỚC ---
const int SF[8] = {36,34,32,30,28,26,24,22};

// --- CẢM BIẾN SAU ---
const int SR[8] = {51,49,47,45,43,41,39,37};

// --- MOTOR TRÁI (IBT-2) ---
const int L_LPWM = 6;
const int L_RPWM = 5;

// --- MOTOR PHẢI (IBT-2) ---
const int R_LPWM = 2;
const int R_RPWM = 3;

// --- NÚT BẤM ---
const int BTN_FORWARD  = 10;
const int BTN_BACKWARD = 11;

// --- THÔNG SỐ CHÍNH ---
const int BASE_SPEED  = 210;
const int MIN_SPEED   = 0;

// --- CALIBRATION ---
const float LEFT_FACTOR  = 1.00;
const float RIGHT_FACTOR = 0.95;

// --- RAMP ACCELERATION ---
const int RAMP_RATE = 255;  // Tắt ramp - phản ứng tức thì

// --- PID PARAMETERS ---
const float Kp = 120.0;
const float Ki = 0.50;
const float Kd = 6.0;

// --- NGƯỠNG ---
const unsigned long SENSOR_TIMEOUT = 200;
const int MIN_SENSORS = 2;
const int STATION_THRESHOLD = 5;

// --- TRẠNG THÁI ---
enum State { STOP, FORWARD, BACKWARD, AT_STATION };
State currentState = STOP;

// --- BIẾN ---
int  valF[8], valR[8];
int  filteredF[8], filteredR[8];
int  posFront = 0, posRear = 0;
bool onLineFront = false, onLineRear = false;
bool stationFront = false, stationRear = false;

unsigned long lastBtnTime = 0;
const int DEBOUNCE_MS = 80;
bool lastBtnF = HIGH;
bool lastBtnB = HIGH;

unsigned long lastLineSeen_F = 0, lastLineSeen_R = 0;

// Ramp
int currentSpeed_L = 0, currentSpeed_R = 0;
int targetSpeed_L = 0, targetSpeed_R = 0;

// PID
float prev_error_F = 0, prev_error_R = 0;
float integral_F = 0, integral_R = 0;

// Moving average filter
int posF_history[3] = {0, 0, 0};
int posR_history[3] = {0, 0, 0};
int pos_idx = 0;

// ============================================
void setup() {
  for(int i = 0; i < 8; i++) {
    pinMode(SF[i], INPUT);
    pinMode(SR[i], INPUT);
  }
  pinMode(L_LPWM, OUTPUT); pinMode(L_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT); pinMode(R_RPWM, OUTPUT);
  pinMode(BTN_FORWARD,  INPUT_PULLUP);
  pinMode(BTN_BACKWARD, INPUT_PULLUP);

  stopMotors();
  Serial.begin(9600);
  delay(800);
  Serial.println("=== AGV PID - READY ===");

  lastLineSeen_F = millis();
  lastLineSeen_R = millis();
}

// ============================================
void loop() {
  readButtons();
  readSensors();
  detectStation();
  calcPositionFront();
  calcPositionRear();
  runMotors();
  debugPrint();
  delay(8);
}

// ============================================
// ĐỌC NÚT BẤM
// ============================================
void readButtons() {
  if(millis() - lastBtnTime < DEBOUNCE_MS) return;

  bool curF = digitalRead(BTN_FORWARD);
  bool curB = digitalRead(BTN_BACKWARD);

  bool pressedF = (lastBtnF == HIGH && curF == LOW);
  bool pressedB = (lastBtnB == HIGH && curB == LOW);

  if(pressedF) {
    currentState = (currentState == FORWARD) ? STOP : FORWARD;
    integral_F = 0; integral_R = 0;
    prev_error_F = 0; prev_error_R = 0;
    Serial.println(currentState == FORWARD ? ">> TIEN" : ">> DUNG");
  }
  else if(pressedB) {
    currentState = (currentState == BACKWARD) ? STOP : BACKWARD;
    integral_F = 0; integral_R = 0;
    prev_error_F = 0; prev_error_R = 0;
    Serial.println(currentState == BACKWARD ? ">> LUI" : ">> DUNG");
  }

  lastBtnTime = millis();
  lastBtnF = curF;
  lastBtnB = curB;
}

// ============================================
// ĐỌC & XỬ LÝ CẢM BIẾN
// ============================================
void readSensors() {
  for(int i = 0; i < 8; i++) {
    valF[i] = digitalRead(SF[i]);
    valR[i] = digitalRead(SR[i]);
  }
  for(int i = 0; i < 8; i++) {
    filteredF[i] = valF[i];
    filteredR[i] = valR[i];
  }
}

// ============================================
// PHÁT HIỆN TRẠM
// ============================================
void detectStation() {
  int countF = 0, countR = 0;
  // ✅ Chỉ đếm index 1→6 (bỏ 2 đầu ngoài cùng)
  for(int i = 1; i < 7; i++) {
    if(valF[i] == 0) countF++;
    if(valR[i] == 0) countR++;
  }
  stationFront = (countF >= STATION_THRESHOLD);
  stationRear  = (countR >= STATION_THRESHOLD);

  if((currentState == FORWARD && stationFront) ||
     (currentState == BACKWARD && stationRear)) {
    stopMotors();
    currentState = AT_STATION;
    Serial.println(">>> DEN TRAM DUNG <<<");
  }
}

// ============================================
// TÍNH VỊ TRÍ
// ============================================
void calcPositionFront() {
  int sum = 0, count = 0;
  // ✅ Bỏ index 0 và 7 - tránh nhiễu cảm biến ngoài cùng
  for(int i = 1; i < 7; i++) {
    if(filteredF[i] == 0) {
      sum += (i - 4);
      count++;
    }
  }
  if(count < MIN_SENSORS) {
    onLineFront = false;
    return;
  }
  posFront = sum / count;
  posFront = applyMovingAverage(posFront, posF_history);
  onLineFront = true;
  lastLineSeen_F = millis();
}

void calcPositionRear() {
  int sum = 0, count = 0;
  // ✅ Bỏ index 0 và 7 - tránh nhiễu cảm biến ngoài cùng
  for(int i = 1; i < 7; i++) {
    if(filteredR[i] == 0) {
      sum += (i - 4);
      count++;
    }
  }
  if(count < MIN_SENSORS) {
    onLineRear = false;
    return;
  }
  posRear = sum / count;
  posRear = applyMovingAverage(posRear, posR_history);
  onLineRear = true;
  lastLineSeen_R = millis();
}

int applyMovingAverage(int newVal, int* history) {
  history[pos_idx] = newVal;
  pos_idx = (pos_idx + 1) % 3;
  return (history[0] + history[1] + history[2]) / 3;
}

// ============================================
// CHẠY MOTOR
// ============================================
void runMotors() {
  if(currentState == FORWARD) {
    if(!onLineRear) {
      if(millis() - lastLineSeen_R > 150) stopMotors();
      return;
    }
    if(millis() - lastLineSeen_R > SENSOR_TIMEOUT) {
      stopMotors();
      currentState = STOP;
      Serial.println(">> MAT LINE SAU - TIMEOUT");
      return;
    }
    driveForward(posRear);
  }
  else if(currentState == BACKWARD) {
    if(!onLineFront) {
      if(millis() - lastLineSeen_F > 150) stopMotors();
      return;
    }
    if(millis() - lastLineSeen_F > SENSOR_TIMEOUT) {
      stopMotors();
      currentState = STOP;
      Serial.println(">> MAT LINE TRUOC - TIMEOUT");
      return;
    }
    driveBackward(posFront);
  }
}

// ============================================
// DRIVE FORWARD
// ============================================
void driveForward(int pos) {
  float error = pos;

  integral_F += error;
  float derivative = error - prev_error_F;
  float correction = Kp * error + Ki * integral_F + Kd * derivative;

  prev_error_F = error;
  integral_F = constrain(integral_F, -60, 60);

  int leftSpeed  = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;

  leftSpeed  = constrain(leftSpeed,  MIN_SPEED, 255);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, 255);

  targetSpeed_L = leftSpeed;
  targetSpeed_R = rightSpeed;
  applyRampAcceleration();

  float L_factor = (error != 0) ? LEFT_FACTOR : 1.0;
  float R_factor = (error != 0) ? RIGHT_FACTOR : 1.0;

  int L_out = constrain((int)(currentSpeed_L * L_factor), 0, 255);
  int R_out = constrain((int)(currentSpeed_R * R_factor), 0, 255);

  analogWrite(L_LPWM, L_out);   analogWrite(L_RPWM, 0);
  analogWrite(R_LPWM, R_out);   analogWrite(R_RPWM, 0);
}

// ============================================
// DRIVE BACKWARD
// ============================================
void driveBackward(int pos) {
  float error = pos;

  integral_R += error;
  float derivative = error - prev_error_R;
  float correction = Kp * error + Ki * integral_R + Kd * derivative;

  correction = -correction;  // Đảo dấu cho chiều lùi

  prev_error_R = error;
  integral_R = constrain(integral_R, -60, 60);

  int leftSpeed  = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;

  leftSpeed  = constrain(leftSpeed,  MIN_SPEED, 255);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, 255);

  targetSpeed_L = leftSpeed;
  targetSpeed_R = rightSpeed;
  applyRampAcceleration();

  float L_factor = (error != 0) ? LEFT_FACTOR : 1.0;
  float R_factor = (error != 0) ? RIGHT_FACTOR : 1.0;

  int L_out = constrain((int)(currentSpeed_L * L_factor), 0, 255);
  int R_out = constrain((int)(currentSpeed_R * R_factor), 0, 255);

  analogWrite(L_LPWM, 0);       analogWrite(L_RPWM, L_out);
  analogWrite(R_LPWM, 0);       analogWrite(R_RPWM, R_out);
}

// ============================================
// RAMP & STOP
// ============================================
void applyRampAcceleration() {
  if(currentSpeed_L < targetSpeed_L)
    currentSpeed_L = min(currentSpeed_L + RAMP_RATE, targetSpeed_L);
  else if(currentSpeed_L > targetSpeed_L)
    currentSpeed_L = max(currentSpeed_L - RAMP_RATE, targetSpeed_L);

  if(currentSpeed_R < targetSpeed_R)
    currentSpeed_R = min(currentSpeed_R + RAMP_RATE, targetSpeed_R);
  else if(currentSpeed_R > targetSpeed_R)
    currentSpeed_R = max(currentSpeed_R - RAMP_RATE, targetSpeed_R);
}

void stopMotors() {
  targetSpeed_L = 0; targetSpeed_R = 0;
  currentSpeed_L = 0; currentSpeed_R = 0;
  analogWrite(L_LPWM, 0); analogWrite(L_RPWM, 0);
  analogWrite(R_LPWM, 0); analogWrite(R_RPWM, 0);
  integral_F = 0;
  integral_R = 0;
  prev_error_F = 0;
  prev_error_R = 0;
}

// ============================================
// DEBUG
// ============================================
void debugPrint() {
  static unsigned long last = 0;
  if(millis() - last < 100) return;
  last = millis();

  if(currentState == FORWARD) {
    Serial.print(">>> TIEN <<<");
    Serial.print(" | POS:");
    Serial.print(posRear);
    Serial.print(" CORR:");
    Serial.print(Kp * posRear, 1);
    Serial.print(" | L:");
    Serial.print(currentSpeed_L);
    Serial.print(" R:");
    Serial.print(currentSpeed_R);
    Serial.print(" | SENSOR R: [");
    // ✅ Đánh dấu cảm biến bị bỏ qua bằng dấu *
    for(int i = 0; i < 8; i++) {
      if(i == 0 || i == 7) Serial.print("*");
      else Serial.print(valR[i]);
      if(i < 7) Serial.print(" ");
    }
    Serial.println("]");
  }
  else if(currentState == BACKWARD) {
    Serial.print(">>> LUI <<<");
    Serial.print(" | POS:");
    Serial.print(posFront);
    Serial.print(" CORR:");
    Serial.print(Kp * posFront, 1);
    Serial.print(" | L:");
    Serial.print(currentSpeed_L);
    Serial.print(" R:");
    Serial.print(currentSpeed_R);
    Serial.print(" | SENSOR F: [");
    for(int i = 0; i < 8; i++) {
      if(i == 0 || i == 7) Serial.print("*");
      else Serial.print(valF[i]);
      if(i < 7) Serial.print(" ");
    }
    Serial.println("]");
  }
  else {
    Serial.print(">>> DUNG - SENSOR TEST <<<");
    Serial.print(" | F: [");
    for(int i = 0; i < 8; i++) {
      if(i == 0 || i == 7) Serial.print("*");
      else Serial.print(valF[i]);
      if(i < 7) Serial.print(" ");
    }
    Serial.print("] R: [");
    for(int i = 0; i < 8; i++) {
      if(i == 0 || i == 7) Serial.print("*");
      else Serial.print(valR[i]);
      if(i < 7) Serial.print(" ");
    }
    Serial.println("]");
  }
}