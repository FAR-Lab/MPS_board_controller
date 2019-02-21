#define DIR_PIN_R (2)
#define PWM_PIN_R (3)
#define BRAKE_PIN_R (4)

#define DIR_PIN_L (10)
#define PWM_PIN_L (11)
#define BRAKE_PIN_L (12)


#define HAL (A0) // (yellow)
#define HBL (A1) // (blue)
#define HCL (A2) // (green)
#define HAR (A3) // (yellow)
#define HBR (A4) // (blue)
#define HCR (A5) // (green)

//#define MA (A5)
//#define MB (A4)
//#define MC (A3)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(DIR_PIN_L, OUTPUT);
  pinMode(PWM_PIN_L, OUTPUT);
  pinMode(BRAKE_PIN_L, OUTPUT);
  pinMode(DIR_PIN_R, OUTPUT);
  pinMode(PWM_PIN_R, OUTPUT);
  pinMode(BRAKE_PIN_R, OUTPUT);


  digitalWrite(DIR_PIN_L, LOW);
  digitalWrite(PWM_PIN_L, LOW);
  digitalWrite(BRAKE_PIN_L, LOW);
  digitalWrite(DIR_PIN_R, LOW);
  digitalWrite(PWM_PIN_R, LOW);
  digitalWrite(BRAKE_PIN_R, LOW);

  // from https://etechnophiles.com/change-frequency-pwm-pins-arduino-uno/
  TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
  // TCCR2B = TCCR2B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz

  // more info at https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM
}

bool lastHAL, lastHBL, lastHCL,
     lastHAR, lastHBR, lastHCR;

float leftSpeed = 0; // ticks / second
float lastLeftSpeed = 0; // ticks / second
float rightSpeed = 0; // ticks / second
float lastRightSpeed = 0; // ticks / second

unsigned long lastLeftTick = 0;
unsigned long lastRightTick = 0;

float updateRate = 0;
unsigned long lastUpdate = 0;
unsigned long lastPrint = 0;

unsigned long killTime = 0;

void updateSpeed(float &whichSpeed, unsigned long dt) {
  whichSpeed = 1000000 / dt;
}

void setMotorPWM();
void printData();
void checkKillTime();

void loop() {
  unsigned long now = micros();
  bool curHAL = digitalRead(HAL),
       curHBL = digitalRead(HBL),
       curHCL = digitalRead(HCL),
       curHAR = digitalRead(HAR),
       curHBR = digitalRead(HBR),
       curHCR = digitalRead(HCR);

  if (curHAL != lastHAL || curHBL != lastHBL || curHCL != lastHCL) {
    updateSpeed(leftSpeed, now - lastLeftTick);
    lastLeftTick = now;
  } else if (now - lastLeftTick > 100000) {
    leftSpeed = 0;
  }
  if (curHAR != lastHAR || curHBR != lastHBR || curHCR != lastHCR) {
    updateSpeed(rightSpeed, now - lastRightTick);
    lastRightTick = now;
  } else if (now - lastRightTick > 100000) {
    rightSpeed = 0;
  }

  setMotorPWM();

  updateRate = 0.9 * updateRate + 0.1 * (1000000 / (now - lastUpdate));
  lastUpdate = now;
  lastHAL = curHAL;
  lastHBL = curHBL;
  lastHCL = curHCL;
  lastHAR = curHAR;
  lastHBR = curHBR;
  lastHCR = curHCR;
  lastLeftSpeed = leftSpeed;
  lastRightSpeed = rightSpeed;

  if (now > lastPrint + 1000000/5) {
    printData();
    lastPrint = now;
  }

  checkKillTime();
}

#define MAX_PWM (100)
#define SPIKE_PWM (50)
#define LOW_PWM (30)

int targetLeftSpeed = 0;
int targetRightSpeed = 0;

float leftPWM = 0;
float rightPWM = 0;
float k = 0.001;

float epsilon = 0.01;

void setPWM(float &pwm, const int &targetSpeed, float &currentSpeed, float &lastSpeed) {
  if (lastSpeed < epsilon && currentSpeed > epsilon) {
    pwm = LOW_PWM;
  } else if (currentSpeed < epsilon) {
    pwm = SPIKE_PWM;
  } else if (abs(targetSpeed - currentSpeed) > epsilon) {
    pwm = constrain(pwm + k * (targetSpeed - currentSpeed), 0, MAX_PWM);
  }
}

void setMotorPWM() {
  if (targetLeftSpeed > 0 || targetRightSpeed > 0) {
    setPWM(leftPWM, targetLeftSpeed, leftSpeed, lastLeftSpeed);
    setPWM(rightPWM, targetRightSpeed, rightSpeed, lastRightSpeed);
  } else {
    leftPWM = rightPWM = 0;
  }

  analogWrite(PWM_PIN_L, leftPWM);
  analogWrite(PWM_PIN_R, rightPWM);
}

void serialEvent() {
  String input = Serial.readStringUntil('\n');

  if (input[0] == ':') {
    String csv = input.substring(1);
    csv.trim();
    int commaIndex = csv.indexOf(",");
    targetLeftSpeed = csv.substring(0, commaIndex).toInt();
    targetRightSpeed = csv.substring(commaIndex+1).toInt();
    killTime = micros() + 250UL*1000UL;
  } else {
    killTime = 0;
  }

  if (input[0] == 'f') {
    Serial.println("FORWARD!");
    digitalWrite(DIR_PIN_L, LOW);
    digitalWrite(DIR_PIN_R, HIGH);
    digitalWrite(BRAKE_PIN_R, HIGH);
    digitalWrite(BRAKE_PIN_L, HIGH);
  } else if (input[0] == 'b') {
    Serial.println("BACKWARD!");
    digitalWrite(DIR_PIN_L, HIGH);
    digitalWrite(DIR_PIN_R, LOW);
    digitalWrite(BRAKE_PIN_R, HIGH);
    digitalWrite(BRAKE_PIN_L, HIGH);
  } else if (input[0] == 'r') {
    Serial.println("RIGHT!");
    digitalWrite(DIR_PIN_L, LOW);
    digitalWrite(DIR_PIN_R, HIGH);
    digitalWrite(BRAKE_PIN_R, HIGH);
    digitalWrite(BRAKE_PIN_L, LOW);
  } else if (input[0] == 'l') {
    Serial.println("LEFT!");
    digitalWrite(DIR_PIN_L, LOW);
    digitalWrite(DIR_PIN_R, HIGH);
    digitalWrite(BRAKE_PIN_R, LOW);
    digitalWrite(BRAKE_PIN_L, HIGH);
  }

  if (input[0] >= '0' && input[0] <= '9') {
    int i = input.toInt();
    if (i < 30) {
      targetLeftSpeed = targetRightSpeed = 0;
    } else {
      targetLeftSpeed = targetRightSpeed = constrain(i, 30, 200);
    }
    Serial.print("SPEED ");
    Serial.println(targetLeftSpeed);
  }
}

void checkKillTime() {
  if (killTime > 0 && micros() > killTime) {
    targetLeftSpeed = targetRightSpeed = 0;
  }
}

void printData() {
  Serial.print(leftSpeed);
  Serial.print(",");
  Serial.print(rightSpeed);
  Serial.print(",");
  Serial.print(updateRate);
  Serial.print(",");
  Serial.print(leftPWM);
  Serial.print(",");
  Serial.print(rightPWM);

  Serial.println();
}
