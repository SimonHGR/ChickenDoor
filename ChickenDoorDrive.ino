/*
 * Stepper motor driver for
 * Chicken Door
 * 
 * TODO / CONSIDER
 * - change driver type to use "MyStepperDriver"
 * - photosensor with smoothing/debouncing and thresholds
 * - real time clock???
 * - end stop switch
 * 
 */
#define LED_PIN          13

#define DIRECTION_PIN     3
#define STEP_PIN          2
#define MOTOR_ENABLE      7

#define STOP_PIN          8
#define CONTROL_PIN       9

#define STEP_SIZE_MS1_PIN 4
#define STEP_SIZE_MS2_PIN 5
#define STEP_SIZE_MS3_PIN 6

#define MOTOR_RETURN 0
#define MOTOR_OUT    1
#define MOTOR_STOP   0
#define MOTOR_SLOW   1
#define MOTOR_FAST   2

// 400 steps per drive revoltion
// 5 drive revolution per 1 turn
// 20 turns per inch
// 5 inches nominal max screw
// max range = 400 x 5 x 20 x 5 => 200000
#define DEFAULT_MAX_POSITION 200000
#define TCCR1B_PRESCALE_MASK 0xE0

//=======================================
// step speeds @ 400 steps/rev:
//         Hz      RPM
// fast = 800      120
// slow = 200/6    1/5

/* Driver microstep control
 * MS1 MS2 MS3 Microstep Resolution
 *   L   L   L  Full Step
 *   H   L   L  Half Step
 *   L   H   L  Quarter Step
 *   H   H   L  Eigth Step
 *   H   H   H  Sixteenth Step
 */

int fastMotorOCR1A    = 40000; // temp; should be 20000;
int fastMotorPrescale = 0x09;  // ps 1
int fastMotorMicrostepPattern = 0x00; // full steps

int slowMotorOCR1A    = 1875;
int slowMotorPrescale = 0x0C;  // ps 256
int slowMotorMicrostepPattern = 0x00; // full steps
//=======================================

// null routine
void idle() {}

void (* volatile isr)() = idle;
volatile long lifeCounter = 0;
volatile long currentPosition = 0;
volatile long maxPosition = DEFAULT_MAX_POSITION;

// trigger the motor routine 
void stepMotor()
{
  digitalWrite(STEP_PIN, HIGH);
  digitalWrite(STEP_PIN, LOW);
  int offset = digitalRead(DIRECTION_PIN) == 0 ? -1 : 1;
  currentPosition += offset;
}

// stepping until stop pin
void stepToStopISR() {
  stepMotor();
  if (digitalRead(STOP_PIN) == 0) {
    setModeWaitToStart();
    currentPosition = 0;
  }
}

// wait until control pin then start
void waitToStartISR() {
  if (digitalRead(CONTROL_PIN) == 0) {
    isr = waitStartReleaseISR;
  }
}

// wait until control pin released then start
void waitStartReleaseISR() {
  if (digitalRead(CONTROL_PIN) == 1) {
    setModeRunning();
  }
}

// running until control pin
void stepToControlISR() {
  stepMotor();
  if (currentPosition >= maxPosition
    || digitalRead(CONTROL_PIN) == 0) {
    isr = waitForControlReleaseOnStopISR;
  }
}

// wait for control release prior to second press for rewind
void waitForControlReleaseOnStopISR() {
  if (digitalRead(CONTROL_PIN) == 1) {
    isr = waitToRewindISR;
  }  
}

// waiting to start rewind
void waitToRewindISR() {
  if (digitalRead(CONTROL_PIN) == 0) {
    setModeRewinding();
  }  
}

// master interrupt service routine 
ISR(TIMER1_COMPA_vect)
{
  lifeCounter++;
  if (lifeCounter % 16 == 0) {
      digitalWrite(LED_PIN, digitalRead(LED_PIN) ^ 1);
  }
  // call supporting mode behavior
  (*isr)();
}

void stopMotorInterrupts() {
  TIMSK1 &= ~(1 << OCIE1A);
}

void startMotorInterrupts() {
  TIMSK1 |= (1 << OCIE1A);
}

void setMicrostepBitPattern(int pattern) {
  digitalWrite(STEP_SIZE_MS1_PIN, pattern & 1);
  digitalWrite(STEP_SIZE_MS2_PIN, (pattern >> 1) & 1);
  digitalWrite(STEP_SIZE_MS3_PIN, (pattern >> 2) & 1);
}

void motorControl(int direction, int speed) {
  int dirBit = (direction == MOTOR_OUT) ? 1 : 0;
  digitalWrite(DIRECTION_PIN, dirBit);
  int tccr1bTemplate = TCCR1B & TCCR1B_PRESCALE_MASK;
  if (speed == MOTOR_FAST) {
    TCCR1A = 0;
    OCR1A = fastMotorOCR1A;
    TCNT1 = 0;
    tccr1bTemplate |= fastMotorPrescale;
    TCCR1B = tccr1bTemplate;
    setMicrostepBitPattern(fastMotorMicrostepPattern);
  } else {
    TCCR1A = 0;
    OCR1A = slowMotorOCR1A;
    TCNT1 = 0;
    tccr1bTemplate |= slowMotorPrescale;
    TCCR1B = tccr1bTemplate;
    setMicrostepBitPattern(slowMotorMicrostepPattern);
  }
}

void setModeRewinding() {
  Serial.println("Rewinding mode...");
  motorControl(MOTOR_RETURN, MOTOR_FAST);
  isr = stepToStopISR;
}

void setModeFastForward() {
  Serial.println("FastForward mode...");
  motorControl(MOTOR_OUT, MOTOR_FAST);
  isr = stepToControlISR;
}

void setModeWaitToStart() {
  Serial.println("Wait for start mode...");
  motorControl(MOTOR_OUT, MOTOR_SLOW); // get ready
  isr = waitToStartISR;
}

void setModeRunning() {
  Serial.println("Normal run mode...");
  motorControl(MOTOR_OUT, MOTOR_SLOW);
  isr = stepToControlISR;
}

void setup() {
  Serial.begin(9600);
  Serial.println("Command (stop, rew, run):");

  pinMode(LED_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(MOTOR_ENABLE, OUTPUT);

  pinMode(STEP_SIZE_MS1_PIN, OUTPUT);
  pinMode(STEP_SIZE_MS2_PIN, OUTPUT);
  pinMode(STEP_SIZE_MS3_PIN, OUTPUT);

  pinMode(STOP_PIN, INPUT_PULLUP);
  pinMode(CONTROL_PIN, INPUT_PULLUP);

  digitalWrite(STEP_SIZE_MS1_PIN, LOW);
  digitalWrite(STEP_SIZE_MS2_PIN, LOW);
  digitalWrite(STEP_SIZE_MS3_PIN, LOW);

  setModeRewinding();
  startMotorInterrupts();
}

char inData[128];
int dataCount = 0;

void loop() {
  while (Serial.available() > 0) {
    char charRead = Serial.read();
    if (charRead == 0x0A || charRead == 0x0D) {
      inData[dataCount] = 0;
      if (strcmp(inData, "stop") == 0) {
        setModeWaitToStart();
      } else if (strcmp(inData, "rew") == 0) {
        setModeRewinding();
      } else if (strcmp(inData, "run") == 0) {
        setModeRunning();
      } else if (strcmp(inData, "ff") == 0) {
        setModeFastForward();
      } else if (strcmp(inData, "pos") == 0) {
        Serial.print("Position: " );
        Serial.println(currentPosition);
      } else if (strcmp(inData, "limit") == 0) {
        Serial.print("Setting limit at current position: " );
        Serial.println(currentPosition);
        maxPosition = currentPosition;
      } else if (strcmp(inData, "unlimit") == 0) {
        Serial.println("Setting default limit position" );
        maxPosition = DEFAULT_MAX_POSITION;
      } else {
        Serial.println("Huh?");
      }
      dataCount = 0;
    } else {
      if (dataCount < 127) {
        inData[dataCount++] = charRead;
      }
    }
  }
}

