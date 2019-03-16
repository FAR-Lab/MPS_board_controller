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

#define MIL (20)

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

long leftSpeed_e6 = 0; // ticks / second
long rightSpeed_e6 = 0; // ticks / second

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
      speed = (ticks[0].direction * 1000000L / dt) << MIL;
    } else {
      speed = 0;
    }
  }
}

void setMotorPWM();
void printData();
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
  unsigned long now = micros();
  bool curHAL = digitalRead(HAL),
       curHBL = digitalRead(HBL),
       curHCL = digitalRead(HCL),
       curHAR = digitalRead(HAR),
       curHBR = digitalRead(HBR),
       curHCR = digitalRead(HCR);

//  unsigned long t1 = micros();

  if (curHAL != lastHAL || curHBL != lastHBL || curHCL != lastHCL) {
    leftTicks.unshift({ // make most recent (index 0)
      tickDirection(phaseOf(lastHAL, lastHBL, lastHCL), phaseOf(curHAL, curHBL, curHCL), true),
      now
    });
    updateSpeed(leftSpeed_e6, leftTicks);
  }

//  unsigned long t2 = micros();
  
  if (curHAR != lastHAR || curHBR != lastHBR || curHCR != lastHCR) {
    rightTicks.unshift({ // make most recent (index 0)
      tickDirection(phaseOf(lastHAR, lastHBR, lastHCR), phaseOf(curHAR, curHBR, curHCR)),
      now
    });
    updateSpeed(rightSpeed_e6, rightTicks);
  }

//  unsigned long t3 = micros();
  
  setMotorPWM();

//  unsigned long t4 = micros();

  updateRate = ((updateRate << 4) - updateRate + (now-lastUpdate)) >> 4;

//  unsigned long t5 = micros();

  lastUpdate = now;
  lastHAL = curHAL;
  lastHBL = curHBL;
  lastHCL = curHCL;
  lastHAR = curHAR;
  lastHBR = curHBR;
  lastHCR = curHCR;

//  unsigned long t6 = micros();

  if (now > lastPrint + 1000000UL / 5UL) {
    printData();
    lastPrint = now;
//    Serial.print(t1-now);
//    Serial.print('\t');
//    Serial.print(t2-t1);
//    Serial.print('\t');
//    Serial.print(t3-t2);
//    Serial.print('\t');
//    Serial.print(t4-t3);
//    Serial.print('\t');
//    Serial.print(t5-t4);
//    Serial.print('\t');
//    Serial.println(t6-t5);
  }

  checkKillTime();
}

#define MAX_PWM (255<<MIL)
#define SPIKE_PWM (220<<MIL)
#define LOW_PWM (120<<MIL)

bool leftSpike = false;
bool rightSpike = false;

long targetLeftSpeed_e6 = 0;
long targetRightSpeed_e6 = 0;

long leftPWM_e6 = 0;
long rightPWM_e6 = 0;
int kInverse = 8; // factor of 256;

int epsilon = 100;

void setPWM(long &pwm, const int &targetSpeed, long &currentSpeed, bool &spike) {
  if (abs(currentSpeed) < epsilon) {
    pwm = targetSpeed > 0 ? SPIKE_PWM : -SPIKE_PWM;
    spike = true;
  } else if (spike == true && (pwm == SPIKE_PWM || pwm == -SPIKE_PWM)) {
    pwm = pwm > 0 ? LOW_PWM : -LOW_PWM;
    spike = false;
  } else if (abs(targetSpeed - currentSpeed) > epsilon) {
    pwm = constrain(pwm + (targetSpeed - currentSpeed) >> kInverse, -MAX_PWM, MAX_PWM);
  }
}

void setMotorPWM() {
  if (abs(targetLeftSpeed_e6) > epsilon || abs(targetRightSpeed_e6) > epsilon) {
    setPWM(leftPWM_e6, targetLeftSpeed_e6, leftSpeed_e6, leftSpike);
    setPWM(rightPWM_e6, targetRightSpeed_e6, rightSpeed_e6, rightSpike);
  } else {
    leftPWM_e6 = rightPWM_e6 = 0;
  }

  analogWrite(PWM_PIN_L, abs(leftPWM_e6 >> MIL));
  analogWrite(PWM_PIN_R, abs(rightPWM_e6 >> MIL));
  digitalWrite(DIR_PIN_L, leftPWM_e6 < 0 ? HIGH : LOW);
  digitalWrite(DIR_PIN_R, rightPWM_e6 < 0 ? LOW : HIGH);
  digitalWrite(BRAKE_PIN_L, abs(leftPWM_e6) > epsilon ? HIGH : LOW);
  digitalWrite(BRAKE_PIN_R, abs(rightPWM_e6) > epsilon ? HIGH : LOW);
}

void serialEventRun() {
  if (Serial.available()) serialEvent();
}
void serialEvent() {
  String input = Serial.readStringUntil('\n');

  if (input[0] == ':') {
    String csv = input.substring(1);
    csv.trim();
    int commaIndex = csv.indexOf(",");
    targetLeftSpeed_e6 = csv.substring(0, commaIndex).toInt() << MIL;
    targetRightSpeed_e6 = csv.substring(commaIndex + 1).toInt() << MIL;
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
      targetLeftSpeed_e6 = targetRightSpeed_e6 = 0;
    } else {
      targetLeftSpeed_e6 = targetRightSpeed_e6 = constrain(i << MIL, 30 << MIL, MAX_PWM);
    }
    Serial.print("SPEED ");
    Serial.println(targetLeftSpeed_e6 >> MIL);
  }
}

void checkKillTime() {
  if (killTime > 0 && micros() > killTime) {
    targetLeftSpeed_e6 = targetRightSpeed_e6 = 0;
  }
}

void printData() {
  Serial.print(leftSpeed_e6 >> MIL);
  Serial.print(",");
  Serial.print(rightSpeed_e6 >> MIL);
  Serial.print(",");
  Serial.print(leftPWM_e6 >> MIL);
  Serial.print(",");
  Serial.print(rightPWM_e6 >> MIL);
  Serial.print(",");
  Serial.print(1000000 / updateRate);

  Serial.println();
}
