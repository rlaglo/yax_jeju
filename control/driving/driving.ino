// Henes-style command loop -> MD30C (PWM+DIR) with centered mapping & asym steering

// ===== Pins (MD30C) =====
static const int FRONT_PWM = 5;
static const int FRONT_DIR = 22;
static const int REAR_PWM  = 6;
static const int REAR_DIR  = 24;
static const int STEER_PWM = 9;
static const int STEER_DIR = 26;

// ===== DIR polarity =====
static bool FRONT_DIR_FWD = LOW;
static bool REAR_DIR_FWD  = LOW;
static bool STEER_DIR_RT  = LOW;

// ===== POT (steering sensor) =====
const int POT = A2;
// 좌/센터/우 실측값 (네 값 반영)
const int RES_LEFT   = 1023;   // 가장 왼쪽에서 읽힌 값
const int RES_CENTER = 580;    // 정확한 센터 실측
const int RES_RIGHT  = 8;     // 가장 오른쪽에서 읽힌 값
const int MAX_STEERING_STEP = 7;

// ===== Params =====
const unsigned int COMMAND_INTERVAL = 50; // ms
const int MAX_SPEED_CHANGE_PER_INTERVAL = 20;
// (기존 파라미터들...)

// === 비례 제어(P-Control)를 위한 새 파라미터 ===
const float STEER_KP = 30.0; // 비례 상수 (핵심 튜닝값!)
const int STEER_DEAD_BAND = 1; // 오차가 이 값 이하면 정지 (기존 DEAD_BAND와 역할이 다름)
const int MIN_STEER_SPEED = 40;  // 모터가 움직이기 시작하는 최소 PWM 값 (옵션)
// 좌/우 비대칭 토크 보정
const int STEER_SPEED_R = 150;  // 오른쪽이 덜 가면 좀 더 크게 (0~255)
const int STEER_SPEED_L = 150;  // 왼쪽
const int DEAD_BAND      = 1;   // |오차|<=1 → 0 처리
const int OVERSHOOT_STEP = 1;   // 목표 근처에서 살짝 더 밀어줌(마찰 극복)

// ===== State =====
int angle_cmd = 0;                 // -MAX..+MAX
int target_front_speed = 0;
int target_rear_speed = 0;

// "현재" 모터에 실제 인가되는 속도 (서서히 증가/감소시킬 값)
int current_front_speed = 0;
int current_rear_speed = 0;
unsigned long lastCommandTime = 0;

// ==== MD30C drive ====
void md30c_drive(int pwmPin, int dirPin, int signedSpeed, bool dirPositiveLevel) {
  int duty = abs(signedSpeed);
  if (duty > 255) duty = 255;
  bool pos = (signedSpeed >= 0);
  digitalWrite(dirPin, pos ? dirPositiveLevel : !dirPositiveLevel);
  analogWrite(pwmPin, duty);  // 0 -> brake
}
void brake(int pwmPin) { analogWrite(pwmPin, 0); }

void setFrontMotorSpeed(int spd){ md30c_drive(FRONT_PWM, FRONT_DIR, spd, FRONT_DIR_FWD); }
void setRearMotorSpeed (int spd){ md30c_drive(REAR_PWM,  REAR_DIR,  spd, REAR_DIR_FWD ); }
void steerRight(){ md30c_drive(STEER_PWM, STEER_DIR, +STEER_SPEED_R, STEER_DIR_RT); }
void steerLeft (){ md30c_drive(STEER_PWM, STEER_DIR, -STEER_SPEED_L, STEER_DIR_RT); }
void steerBrake(){ brake(STEER_PWM); }

// ===== Serial parser (s/l/r 그대로) =====
const unsigned int MAX_INPUT = 20;
void processData(const char *data) {
  int sIndex=-1, fIndex=-1, rIndex=-1;
  for (int i=0; data[i]!='\0'; i++){
    if (data[i]=='s') sIndex=i;
    else if (data[i]=='l') fIndex=i;
    else if (data[i]=='r') rIndex=i;
  }
  if (sIndex!=-1 && fIndex!=-1 && rIndex!=-1){
    int newAngle      = atoi(data + sIndex + 1);
    int newFrontSpeed = atoi(data + fIndex + 1);
    int newRearSpeed  = atoi(data + rIndex + 1);
    angle_cmd   = constrain(newAngle, -MAX_STEERING_STEP, MAX_STEERING_STEP);
    target_front_speed = constrain(newFrontSpeed, -255, 255);
    target_rear_speed  = constrain(newRearSpeed,  -255, 255);
  }
}
void processIncomingByte(const byte b){
  static char line[MAX_INPUT]; static unsigned int pos=0;
  switch(b){
    case '\n': line[pos]=0; processData(line); pos=0; break;
    case '\r': break;
    default: if (pos < (MAX_INPUT-1)) line[pos++]=b; break;
  }
}

// ===== Centered piecewise map with clamping =====
int map_centered(int res_raw) {
  // 센서 값 클램프 (엔드스톱 밖으로 튀어도 스팬을 유지)
  int res = res_raw;
  if (RES_LEFT >= RES_RIGHT) {          // 보통 이렇게 큼->작음
    if (res > RES_LEFT)  res = RES_LEFT;
    if (res < RES_RIGHT) res = RES_RIGHT;
  } else {                              // 혹시 반대 극성 센서면
    if (res < RES_LEFT)  res = RES_LEFT;
    if (res > RES_RIGHT) res = RES_RIGHT;
  }

  long step;
  if (res >= RES_CENTER) {
    // 센터(0) → 좌(-MAX)
    step = map(res, RES_CENTER, RES_LEFT, 0, -MAX_STEERING_STEP);
  } else {
    // 센터(0) → 우(+MAX)
    step = map(res, RES_CENTER, RES_RIGHT, 0, +MAX_STEERING_STEP);
  }
  // 데드밴드
  //if (abs(step) <= DEAD_BAND) step = 0;
  return (int)constrain(step, -MAX_STEERING_STEP, MAX_STEERING_STEP);
}

// ===== Setup & Loop =====
void setup() {
  Serial.begin(115200);

  pinMode(POT, INPUT);
  pinMode(FRONT_PWM, OUTPUT); pinMode(FRONT_DIR, OUTPUT);
  pinMode(REAR_PWM,  OUTPUT); pinMode(REAR_DIR,  OUTPUT);
  pinMode(STEER_PWM, OUTPUT); pinMode(STEER_DIR, OUTPUT);

  brake(FRONT_PWM); brake(REAR_PWM); brake(STEER_PWM);
  digitalWrite(FRONT_DIR, LOW); digitalWrite(REAR_DIR, LOW); digitalWrite(STEER_DIR, LOW);

  Serial.println("MD30C + centered steering mapping ready.");
}

void loop() {
  unsigned long now = millis();
  while (Serial.available() > 0) processIncomingByte(Serial.read());

  if (now - lastCommandTime >= COMMAND_INTERVAL) {
    int res = analogRead(POT);
    int step_now = map_centered(res);
    int err = angle_cmd - step_now;

    // 비례 제어 로직 시작
    if (abs(err) <= STEER_DEAD_BAND) {
      steerBrake(); // 목표 범위 안에 들어오면 정지
    } else {
      // 오차에 비례하여 조향 속도 계산
      int steer_speed = (int)(err * STEER_KP);

      // 최대/최소 속도 제한
      steer_speed = constrain(steer_speed, -255, 255);

      // (옵션) 모터가 약한 힘으로 돌지 못할 때 (Stiction 극복)
      // 계산된 속도가 0은 아니지만, 최소 구동 속도보다 작을 경우
      if (steer_speed > 0 && steer_speed < MIN_STEER_SPEED) {
        steer_speed = MIN_STEER_SPEED;
      } else if (steer_speed < 0 && steer_speed > -MIN_STEER_SPEED) {
        steer_speed = -MIN_STEER_SPEED;
      }
      
      // 계산된 속도로 조향 모터 구동
      md30c_drive(STEER_PWM, STEER_DIR, steer_speed, STEER_DIR_RT);
    }
    
    // 1. Front Motor
    if (current_front_speed < target_front_speed) {
      current_front_speed += MAX_SPEED_CHANGE_PER_INTERVAL;
      if (current_front_speed > target_front_speed) {
        current_front_speed = target_front_speed; // 목표값 초과 방지
      }
    } else if (current_front_speed > target_front_speed) {
      current_front_speed -= MAX_SPEED_CHANGE_PER_INTERVAL;
      if (current_front_speed < target_front_speed) {
        current_front_speed = target_front_speed; // 목표값 초과 방지
      }
    }

    // 2. Rear Motor
    if (current_rear_speed < target_rear_speed) {
      current_rear_speed += MAX_SPEED_CHANGE_PER_INTERVAL;
      if (current_rear_speed > target_rear_speed) {
        current_rear_speed = target_rear_speed;
      }
    } else if (current_rear_speed > target_rear_speed) {
      current_rear_speed -= MAX_SPEED_CHANGE_PER_INTERVAL;
      if (current_rear_speed < target_rear_speed) {
        current_rear_speed = target_rear_speed;
      }
    }
    
    // 계산된 "현재" 속도로 모터 구동
    setFrontMotorSpeed(current_front_speed);
    setRearMotorSpeed(current_rear_speed);
    // (디버그) 필요 시 주석 해제해서 확인
    Serial.print("Target F: "); Serial.print(target_front_speed);
    Serial.print(" | Current F: "); Serial.print(current_front_speed);
    Serial.print(" | Target R: "); Serial.print(target_rear_speed);
    Serial.print(" | Current R: "); Serial.println(current_rear_speed);

    lastCommandTime = now;
  }
}
