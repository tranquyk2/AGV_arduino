
#include <avr/wdt.h>

// --- CẢM BIẾN ---
const int SF[8] = {36,34,32,30,28,26,24,22};
const int SR[8] = {51,49,47,45,43,41,39,37};

// --- MOTOR ---
const int L_LPWM = 6;
const int L_RPWM = 5;
const int R_LPWM = 2;
const int R_RPWM = 3;

// --- NÚT ---
const int BTN_FORWARD  = 10;
const int BTN_BACKWARD = 11;

// --- LED & BUZZER ---
const int LED_GREEN = 44;
const int LED_RED   = 46;
const int BUZZER    = 8;

// --- THÔNG SỐ ---
const int BASE_SPEED = 210;
const int MIN_SPEED  = 0;
const int RAMP_RATE  = 30;

// --- CALIBRATION ---
const float LEFT_FACTOR  = 1.00;
const float RIGHT_FACTOR = 0.95;

// --- PID ---
const float Kp = 70.0;
const float Ki = 0.25;   // Giảm nhẹ so với 0.30 để ổn định hơn
const float Kd = 4.0;

// --- NGƯỠNG ---
const unsigned long SENSOR_TIMEOUT = 280;
const int MIN_SENSORS       = 2;
const int STATION_THRESHOLD = 5;

// --- STATE ---
enum State { STOP, FORWARD, BACKWARD, AT_STATION };
State currentState = STOP;

// --- BIẾN ---
int valF[8], valR[8];
int filteredF[8], filteredR[8];

int posFront = 0, posRear = 0;
bool onLineFront = false, onLineRear = false;

unsigned long lastLineSeen_F = 0, lastLineSeen_R = 0;

// Button
unsigned long lastBtnTime = 0;
const int DEBOUNCE_MS = 80;
bool lastBtnF = HIGH, lastBtnB = HIGH;

// Speed
int currentSpeed_L = 0, currentSpeed_R = 0;
int targetSpeed_L  = 0, targetSpeed_R  = 0;

// PID
float prev_error_F = 0, prev_error_R = 0;
float integral_F   = 0, integral_R   = 0;

// Moving Average Filter
int posF_history[3] = {0,0,0};
int posR_history[3] = {0,0,0};
int pos_idx_F = 0;
int pos_idx_R = 0;

// Station
bool exitingStation = false;

void setup() {
  for(int i=0; i<8; i++) {
    pinMode(SF[i], INPUT);
    pinMode(SR[i], INPUT);
  }

  pinMode(L_LPWM, OUTPUT); pinMode(L_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT); pinMode(R_RPWM, OUTPUT);
  
  pinMode(BTN_FORWARD,  INPUT_PULLUP);
  pinMode(BTN_BACKWARD, INPUT_PULLUP);
  
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED,   OUTPUT);
  pinMode(BUZZER,    OUTPUT);

  stopMotors();
  setLED_Stop();

  Serial.begin(115200);
  delay(800);
  Serial.println("=== AGV MERGED OPTIMIZED - READY ===");

  lastLineSeen_F = millis();
  lastLineSeen_R = millis();

  buzzer_beep(3);
  delay(1000);
  
  wdt_enable(WDTO_4S);
}

// ============================================
void loop() {
  wdt_reset();
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
// LED & BUZZER
void setLED_Run() {
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_RED,   LOW);
}

void setLED_Stop() {
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED,   HIGH);
}

void buzzer_beep(int times) {
  for(int i = 0; i < times; i++) {
    digitalWrite(BUZZER, HIGH);
    delay(150);
    digitalWrite(BUZZER, LOW);
    if(i < times - 1) delay(100);
  }
}

// ============================================
// BUTTON
void readButtons() {
  if(millis() - lastBtnTime < DEBOUNCE_MS) return;

  bool curF = digitalRead(BTN_FORWARD);
  bool curB = digitalRead(BTN_BACKWARD);

  bool pressedF = (lastBtnF == HIGH && curF == LOW);
  bool pressedB = (lastBtnB == HIGH && curB == LOW);

  if(pressedF) {
    buzzer_beep(1);
    if(currentState == FORWARD) {
      currentState = STOP;
      setLED_Stop();
      Serial.println(">> DUNG");
    } else {
      currentState = FORWARD;
      exitingStation = true;
      integral_F = 0; integral_R = 0;
      prev_error_F = 0; prev_error_R = 0;
      setLED_Run();
      Serial.println(">> TIEN");
    }
  }
  else if(pressedB) {
    buzzer_beep(1);
    if(currentState == BACKWARD) {
      currentState = STOP;
      setLED_Stop();
      Serial.println(">> DUNG");
    } else {
      currentState = BACKWARD;
      exitingStation = true;
      integral_F = 0; integral_R = 0;
      prev_error_F = 0; prev_error_R = 0;
      setLED_Run();
      Serial.println(">> LUI");
    }
  }

  lastBtnTime = millis();
  lastBtnF = curF;
  lastBtnB = curB;
}

// ============================================
// SENSOR
void readSensors() {
  static int prevF[8] = {0}, prevR[8] = {0};

  for(int i = 0; i < 8; i++) {
    valF[i] = digitalRead(SF[i]);
    valR[i] = digitalRead(SR[i]);

    filteredF[i] = (valF[i] + prevF[i] == 2) ? 1 : 0;
    filteredR[i] = (valR[i] + prevR[i] == 2) ? 1 : 0;

    prevF[i] = valF[i];
    prevR[i] = valR[i];
  }
}

// ============================================
// STATION DETECTION (Improved)
void detectStation() {
  int countF = 0, countR = 0;
  for(int i = 1; i < 7; i++) {
    if(filteredF[i] == 0) countF++;
    if(filteredR[i] == 0) countR++;
  }

  bool atStation = (countF >= STATION_THRESHOLD) || (countR >= STATION_THRESHOLD);

  if(exitingStation) {
    if(!atStation) {
      exitingStation = false;
      Serial.println("... Da thoat tram thanh cong ...");
    }
    return;
  }

  if((currentState == FORWARD || currentState == BACKWARD) && atStation) {
    stopMotors();
    currentState = AT_STATION;
    setLED_Stop();
    buzzer_beep(2);
    Serial.println(">>> DEN TRAM - DUNG <<<");
  }
}

// ============================================
// POSITION CALCULATION
int applyMovingAverage(int newVal, int* history, int &idx) {
  history[idx] = newVal;
  idx = (idx + 1) % 3;
  return (history[0] + history[1] + history[2]) / 3;
}

void calcPositionFront() {
  int sum = 0, count = 0;
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
  posFront = applyMovingAverage(sum / count, posF_history, pos_idx_F);
  onLineFront = true;
  lastLineSeen_F = millis();
}

void calcPositionRear() {
  int sum = 0, count = 0;
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
  posRear = applyMovingAverage(sum / count, posR_history, pos_idx_R);
  onLineRear = true;
  lastLineSeen_R = millis();
}

// ============================================
// MOTOR CONTROL
void runMotors() {
  if(currentState == STOP || currentState == AT_STATION) {
    stopMotors();
    return;
  }

  // FAILSAFE MẤT LINE TOÀN BỘ
  if(!onLineFront && !onLineRear) {
    stopMotors();
    currentState = STOP;
    setLED_Stop();
    Serial.println(">> MAT LINE HOAN TOAN");
    return;
  }

  // Failsafe
  if((currentState == FORWARD && !onLineRear) || 
     (currentState == BACKWARD && !onLineFront)) {
    if(millis() - (currentState == FORWARD ? lastLineSeen_R : lastLineSeen_F) > SENSOR_TIMEOUT) {
      stopMotors();
      currentState = STOP;
      setLED_Stop();
      Serial.println(">> MAT LINE - TIMEOUT - STOP");
    }
    return;
  }

  if(currentState == FORWARD) {
    driveForward(posRear);
  } else if(currentState == BACKWARD) {
    driveBackward(posFront);
  }
}

void driveForward(int pos) {
  float error = pos;
  integral_F += error;
  float derivative = error - prev_error_F;
  float correction = Kp * error + Ki * integral_F + Kd * derivative;

  correction = constrain(correction, -160, 160);
  prev_error_F = error;
  integral_F = constrain(integral_F, -60, 60);

  int leftSpeed  = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;

  targetSpeed_L = constrain(leftSpeed,  MIN_SPEED, 255);
  targetSpeed_R = constrain(rightSpeed, MIN_SPEED, 255);

  applyRamp();
  applyMotorOutput(true);
}

void driveBackward(int pos) {
  float error = pos;
  integral_R += error;
  float derivative = error - prev_error_R;
  float correction = Kp * error + Ki * integral_R + Kd * derivative;

  correction = -correction;
  correction = constrain(correction, -160, 160);
  prev_error_R = error;
  integral_R = constrain(integral_R, -60, 60);

  int leftSpeed  = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;

  targetSpeed_L = constrain(leftSpeed,  MIN_SPEED, 255);
  targetSpeed_R = constrain(rightSpeed, MIN_SPEED, 255);

  applyRamp();
  applyMotorOutput(false);
}

void applyRamp() {
  currentSpeed_L = (currentSpeed_L < targetSpeed_L) ? 
                   min(currentSpeed_L + RAMP_RATE, targetSpeed_L) :
                   max(currentSpeed_L - RAMP_RATE, targetSpeed_L);

  currentSpeed_R = (currentSpeed_R < targetSpeed_R) ? 
                   min(currentSpeed_R + RAMP_RATE, targetSpeed_R) :
                   max(currentSpeed_R - RAMP_RATE, targetSpeed_R);
}

void applyMotorOutput(bool forward) {
  float L_factor = LEFT_FACTOR;
  float R_factor = RIGHT_FACTOR;

  int L_out = constrain((int)(currentSpeed_L * L_factor), 0, 255);
  int R_out = constrain((int)(currentSpeed_R * R_factor), 0, 255);

  if(forward) {
    analogWrite(L_LPWM, L_out); analogWrite(L_RPWM, 0);
    analogWrite(R_LPWM, R_out); analogWrite(R_RPWM, 0);
  } else {
    analogWrite(L_LPWM, 0);     analogWrite(L_RPWM, L_out);
    analogWrite(R_LPWM, 0);     analogWrite(R_RPWM, R_out);
  }
}

void resetPID() {
  integral_F = integral_R = 0;
  prev_error_F = prev_error_R = 0;
}

void stopMotors() {
  targetSpeed_L = targetSpeed_R = 0;
  currentSpeed_L = currentSpeed_R = 0;
  analogWrite(L_LPWM,0); analogWrite(L_RPWM,0);
  analogWrite(R_LPWM,0); analogWrite(R_RPWM,0);
  resetPID();
}

// ============================================
// DEBUG
void debugPrint() {
  static unsigned long last = 0;
  if(millis() - last < 120) return;
  last = millis();

  Serial.print(currentState == FORWARD ? ">>> TIEN " : 
               currentState == BACKWARD ? ">>> LUI " : 
               currentState == AT_STATION ? ">>> TRAM " : ">>> DUNG ");

  if(exitingStation) Serial.print("[EXITING] ");

  if(currentState == FORWARD) {
    Serial.print("POS_R:"); Serial.print(posRear);
  } else if(currentState == BACKWARD) {
    Serial.print("POS_F:"); Serial.print(posFront);
  }

  Serial.print(" | L:"); Serial.print(currentSpeed_L);
  Serial.print(" R:");   Serial.print(currentSpeed_R);
  
  // BUTTON DEBUG
  Serial.print(" | BTN_F:"); Serial.print(digitalRead(BTN_FORWARD));
  Serial.print(" BTN_B:"); Serial.print(digitalRead(BTN_BACKWARD));
  
  // SENSOR DEBUG - FILTERED
  Serial.print(" | SF[");
  for(int i = 0; i < 8; i++) {
    Serial.print(i); Serial.print(":");Serial.print(filteredF[i]);
    if(i<7) Serial.print(" ");
  }
  Serial.print("] SR[");
  for(int i = 0; i < 8; i++) {
    Serial.print(i); Serial.print(":");Serial.print(filteredR[i]);
    if(i<7) Serial.print(" ");
  }
  Serial.print("]");
  
  // SENSOR DEBUG - RAW
  Serial.print(" | RAW_SF[");
  for(int i = 0; i < 8; i++) {
    Serial.print(i); Serial.print(":");Serial.print(valF[i]);
    if(i<7) Serial.print(" ");
  }
  Serial.print("] RAW_SR[");
  for(int i = 0; i < 8; i++) {
    Serial.print(i); Serial.print(":");Serial.print(valR[i]);
    if(i<7) Serial.print(" ");
  }
  Serial.print("]");
  
  Serial.println();
}

