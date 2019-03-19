#include <CircularBuffer.h>

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

typedef byte phase;

struct PhaseTransition {
  //  byte from;
  //  byte to;
  signed char direction;
  unsigned long time;
};

typedef CircularBuffer<PhaseTransition, 5> TickBuffer;

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
  // default is 500
  // TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
  // TCCR2B = TCCR2B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz

  // more info at https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM
}

bool lastHAL, lastHBL, lastHCL,
     lastHAR, lastHBR, lastHCR;

long leftSpeed = 0; // ticks / 10 second
long rightSpeed = 0; // ticks / 10 second

TickBuffer leftTicks;
TickBuffer rightTicks;

long updateRate = 0;
unsigned long lastUpdate = 0;
unsigned long lastPrint = 0;

unsigned long killTime = 0;

#define DEAD_TIME (100000UL)

void updateSpeed(long &speed, TickBuffer &ticks) {
  if (ticks.size() == 0) {
    speed = 0;
  }
  if (ticks.size() == 1) {
    speed = 0;
  }
  if (ticks.size() >= 2) {
    unsigned long dt = ticks[0].time - ticks[1].time;
    if (ticks[0].direction == ticks[1].direction && dt < DEAD_TIME) {
      speed = ticks[0].direction * 10000000 / dt;
    } else {
      speed = 0;
    }
  }
}

void setMotorPWM();
void printData(unsigned long);
void checkKillTime();

// based on commutator tables. maps byte from index [HC][HB][HA] to phase number.
const phase sensorsToPhase[] = {
  0,
  2,
  4,
  3,
  6,
  1,
  5
};

phase phaseOf(const bool &a, const bool &b, const bool &c) {
  byte pn = (a ? 1 : 0) | (b ? 2 : 0) | (c ? 4 : 0);
  return sensorsToPhase[pn];
}

int tickDirection(const phase &lastPhase, const phase &curPhase, bool invert = false) {
  if (abs(lastPhase - curPhase) <= 1) {
    return invert ? lastPhase - curPhase : curPhase - lastPhase;
  } else {
    return (invert ? (lastPhase > curPhase) : (curPhase > lastPhase)) ? -1 : 1;
  }
}

void loop() {
  unsigned long t[6];
  
  unsigned long now = t[0] = micros();
  bool curHAL = digitalRead(HAL),
       curHBL = digitalRead(HBL),
       curHCL = digitalRead(HCL),
       curHAR = digitalRead(HAR),
       curHBR = digitalRead(HBR),
       curHCR = digitalRead(HCR);

  t[1] = micros();

  if (curHAL != lastHAL || curHBL != lastHBL || curHCL != lastHCL) {
    leftTicks.unshift({ // make most recent (index 0)
      tickDirection(phaseOf(lastHAL, lastHBL, lastHCL), phaseOf(curHAL, curHBL, curHCL), true),
      now
    });
    updateSpeed(leftSpeed, leftTicks);
  }

  t[2] = micros();
  
  if (curHAR != lastHAR || curHBR != lastHBR || curHCR != lastHCR) {
    rightTicks.unshift({ // make most recent (index 0)
      tickDirection(phaseOf(lastHAR, lastHBR, lastHCR), phaseOf(curHAR, curHBR, curHCR)),
      now
    });
    updateSpeed(rightSpeed, rightTicks);
  }

  t[3] = micros();
  
  setMotorPWM();

  t[4] = micros();

  updateRate = ((updateRate << 3) - updateRate + (now - lastUpdate)) >> 3;

  t[5] = micros();
  
  lastUpdate = now;
  lastHAL = curHAL;
  lastHBL = curHBL;
  lastHCL = curHCL;
  lastHAR = curHAR;
  lastHBR = curHBR;
  lastHCR = curHCR;

  if (now > lastPrint + 200000UL) {
    printData(now);
    lastPrint = now;
    for (int i = 1; i < sizeof(t); i++) {
      Serial.print(",");
      Serial.print(t[i] - t[i-1]);
    }
    Serial.println();
  }

  checkKillTime();
}

#define MAX_PWM (255)
#define SPIKE_PWM (220)
#define LOW_PWM (120)

bool leftSpike = false;
bool rightSpike = false;

long targetLeftSpeed = 0;
long targetRightSpeed = 0;

float leftPWM = 0;
float rightPWM = 0;
float k = 0.001;

float epsilon = 0.01;

void setPWM(float &pwm, const long &targetSpeed, long &currentSpeed, bool &spike) {
//  if (currentSpeed < abs(epsilon)) {
//    pwm = targetSpeed < 0 ? -SPIKE_PWM : SPIKE_PWM;
//    spike = true;
//  } else if (spike == true && pwm == SPIKE_PWM || pwm == -SPIKE_PWM) {
//    pwm = pwm < 0 ? -LOW_PWM : LOW_PWM;
//    spike = false;
//  } else if (abs(targetSpeed - currentSpeed) > epsilon) {
    pwm = constrain(pwm + k * (targetSpeed - currentSpeed), -MAX_PWM, MAX_PWM);
//  }
}

void setMotorPWM() {
  if (abs(targetLeftSpeed) > epsilon || abs(targetRightSpeed) > epsilon) {
    setPWM(leftPWM, targetLeftSpeed, leftSpeed, leftSpike);
    setPWM(rightPWM, targetRightSpeed, rightSpeed, rightSpike);
    // leftPWM = abs(targetLeftSpeed);
    // rightPWM = abs(targetRightSpeed);
  } else {
    leftPWM = rightPWM = 0;
  }

  analogWrite(PWM_PIN_L, abs(leftPWM));
  analogWrite(PWM_PIN_R, abs(rightPWM));
  digitalWrite(DIR_PIN_L, leftPWM < 0 ? HIGH : LOW);
  digitalWrite(DIR_PIN_R, rightPWM < 0 ? LOW : HIGH);
  digitalWrite(BRAKE_PIN_L, abs(leftPWM) > epsilon ? HIGH : LOW);
  digitalWrite(BRAKE_PIN_R, abs(rightPWM) > epsilon ? HIGH : LOW);
}

void serialEvent() {
  String input = Serial.readStringUntil('\n');

  if (input[0] == ':') {
    String csv = input.substring(1);
    csv.trim();
    int commaIndex = csv.indexOf(",");
    targetLeftSpeed = csv.substring(0, commaIndex).toInt();
    targetRightSpeed = csv.substring(commaIndex + 1).toInt();
    killTime = micros() + 250UL * 1000UL;
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
      targetLeftSpeed = targetRightSpeed = constrain(i, 300, 1000);
    }
    Serial.print("SPEED ");
    Serial.println(targetLeftSpeed);
  }
}
void serialEventRun() {
  if (Serial.available()) { 
    serialEvent();
  }
}

void checkKillTime() {
  if (killTime > 0 && micros() > killTime) {
    targetLeftSpeed = targetRightSpeed = 0;
  }
}

void printData(unsigned long start) {
  unsigned long now = micros();
  Serial.print(leftSpeed);
  Serial.print(",");
  Serial.print(rightSpeed);
  Serial.print(",");
  Serial.print(leftPWM);
  Serial.print(",");
  Serial.print(rightPWM);
  Serial.print(",");
  Serial.print(now - start);
  Serial.print(",");
  Serial.print(1000000 / updateRate);

  Serial.println();
}
