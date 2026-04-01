// ============================================
//  AGV MAGNETIC LINE FOLLOWER - 4 MOTOR (PID FIXED)
//  Driver: IBT-2 (BTS7960) x2
//  Arduino Mega 2560
//  ĐÃ SỬA DẤU PID - LỆCH TRÁI: BÁNH TRÁI CHẬM - BÁNH PHẢI NHANH
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
const int BASE_SPEED  = 110;   // Tốc độ nền (giảm để tune PID dễ hơn)
const int MIN_SPEED   = 45;    // Tốc độ tối thiểu của motor

// --- CALIBRATION ---
const float LEFT_FACTOR  = 1.00;
const float RIGHT_FACTOR = 0.95;

// --- RAMP ACCELERATION ---
const int RAMP_RATE = 16;  // Tăng từ 9 → 16 (tăng tốc nhanh hơn)

// --- PID PARAMETERS (tune ở đây) ---
const float Kp = 12.5;   // Tăng từ 8.5 → 12.5 (phản ứng nhanh tức thì)
const float Ki = 0.14;   // Tăng từ 0.12 → 0.14 (tích lũy error nhanh)
const float Kd = 2.5;    // Tăng từ 2.2 → 2.5 (smooth hơn)

// --- NGƯỠNG ---
const unsigned long SENSOR_TIMEOUT = 600;  // ms
const int MIN_SENSORS = 2;                 // Tăng từ 2 → 3 (cần 3/8 cảm biến)
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
const int DEBOUNCE_MS = 300;
bool lastBtnF = HIGH;
bool lastBtnB = HIGH;

unsigned long lastLineSeen_F = 0, lastLineSeen_R = 0;

// Ramp
int currentSpeed_L = 0, currentSpeed_R = 0;
int targetSpeed_L = 0, targetSpeed_R = 0;

// PID
float prev_error_F = 0, prev_error_R = 0;
float integral_F = 0, integral_R = 0;

// Moving average filter cho position
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
  Serial.println("=== AGV PID FIXED - READY ===");
  
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

  if(pressedF && pressedB) {
    currentState = STOP;
    stopMotors();
    Serial.println(">>> DUNG KHAN CAP <<<");
  }
  else if(pressedF) {
    currentState = (currentState == FORWARD) ? STOP : FORWARD;
    Serial.println(currentState == FORWARD ? ">> TIEN" : ">> DUNG");
  }
  else if(pressedB) {
    currentState = (currentState == BACKWARD) ? STOP : BACKWARD;
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

void detectStation() {
  int countF = 0, countR = 0;
  for(int i = 0; i < 8; i++) {
    if(valF[i] == 0) countF++;
    if(valR[i] == 0) countR++;
  }
  stationFront = (countF >= STATION_THRESHOLD);
  stationRear  = (countR >= STATION_THRESHOLD);

  if((currentState == FORWARD && stationFront) || (currentState == BACKWARD && stationRear)) {
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
  for(int i = 0; i < 8; i++) {
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
  posFront = applyMovingAverage(posFront, posF_history);  // ✅ Filter vị trí
  onLineFront = true;
  lastLineSeen_F = millis();
}

void calcPositionRear() {
  int sum = 0, count = 0;
  for(int i = 0; i < 8; i++) {
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
  posRear = applyMovingAverage(posRear, posR_history);  // ✅ Filter vị trí
  onLineRear = true;
  lastLineSeen_R = millis();
}

// ✅ MOVING AVERAGE FILTER - LỌC NHIỄU VỊ TRÍ
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
    // ✅ TIẾN: Dùng cảm biến SAU (R)
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
    // ✅ LÙI: Dùng cảm biến TRƯỚC (F)
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
// DRIVE FORWARD - PID ĐÃ OPTIMIZE
// ============================================
void driveForward(int pos) {
  float error = pos;
  
  // ✅ DEADZONE
  if(abs(error) < 0.6) error = 0;

  integral_F += error;
  float derivative = error - prev_error_F;
  float correction = Kp * error + Ki * integral_F + Kd * derivative;

  prev_error_F = error;
  integral_F = constrain(integral_F, -100, 100);

  int leftSpeed  = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;

  leftSpeed  = constrain(leftSpeed,  MIN_SPEED, 255);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, 255);

  targetSpeed_L = leftSpeed;
  targetSpeed_R = rightSpeed;
  applyRampAcceleration();

  // ✅ CHỈ ÁP DỤNG CALIBRATION KHI CÓ LỆCH (error != 0)
  float L_factor = (error != 0) ? LEFT_FACTOR : 1.0;
  float R_factor = (error != 0) ? RIGHT_FACTOR : 1.0;
  
  int L_out = constrain((int)(currentSpeed_L * L_factor), 0, 255);
  int R_out = constrain((int)(currentSpeed_R * R_factor), 0, 255);

  analogWrite(L_LPWM, L_out);   analogWrite(L_RPWM, 0);
  analogWrite(R_LPWM, R_out);   analogWrite(R_RPWM, 0);
}

// ============================================
// DRIVE BACKWARD - PID ĐÃ OPTIMIZE
// ============================================
void driveBackward(int pos) {
  float error = pos;

  // ✅ DEADZONE
  if(abs(error) < 0.6) error = 0;

  integral_R += error;
  float derivative = error - prev_error_R;
  float correction = Kp * error + Ki * integral_R + Kd * derivative;

  prev_error_R = error;
  integral_R = constrain(integral_R, -100, 100);

  int leftSpeed  = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;

  leftSpeed  = constrain(leftSpeed,  MIN_SPEED, 255);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, 255);

  targetSpeed_L = leftSpeed;
  targetSpeed_R = rightSpeed;
  applyRampAcceleration();

  // ✅ CHỈ ÁP DỤNG CALIBRATION KHI CÓ LỆCH (error != 0)
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
  if(currentSpeed_L < targetSpeed_L) currentSpeed_L += RAMP_RATE;
  else if(currentSpeed_L > targetSpeed_L) currentSpeed_L -= RAMP_RATE;

  if(currentSpeed_R < targetSpeed_R) currentSpeed_R += RAMP_RATE;
  else if(currentSpeed_R > targetSpeed_R) currentSpeed_R -= RAMP_RATE;
}

void stopMotors() {
  targetSpeed_L = 0; targetSpeed_R = 0;
  currentSpeed_L = 0; currentSpeed_R = 0;
  analogWrite(L_LPWM, 0); analogWrite(L_RPWM, 0);
  analogWrite(R_LPWM, 0); analogWrite(R_RPWM, 0);
  integral_F = 0;
  integral_R = 0;
}

// ============================================
// DEBUG (có hiển thị tốc độ thực tế)
// ============================================
void debugPrint() {
  static unsigned long last = 0;
  if(millis() - last < 200) return;
  last = millis();

  Serial.print("F:[");
  for(int i=0; i<8; i++) { Serial.print(valF[i]); if(i<7) Serial.print("|"); }
  Serial.print("] ");
  
  Serial.print("PF:"); Serial.print(posFront);
  Serial.print(" | R:[");
  for(int i=0; i<8; i++) { Serial.print(valR[i]); if(i<7) Serial.print("|"); }
  Serial.print("] ");
  
  Serial.print("PR:"); Serial.print(posRear);
  Serial.print(" | ");
  
  Serial.print("L:"); Serial.print(currentSpeed_L);
  Serial.print(" R:"); Serial.print(currentSpeed_R);
  Serial.print(" | ");
  
  switch(currentState) {
    case FORWARD: Serial.print("TIEN"); break;
    case BACKWARD: Serial.print("LUI"); break;
    case AT_STATION: Serial.print("TRAM"); break;
    default: Serial.print("DUNG"); break;
  }
  Serial.println();
}