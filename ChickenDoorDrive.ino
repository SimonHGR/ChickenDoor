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

#define LED_PIN              13
#define SHOW_IRQ_LIFE
#define TCCR1B_PRESCALE_MASK 0xE0

//=======================================
// speed 300.03 steps/second
int countOCR1A  = 6666;  // 
int prescale    = 0x0A;  // 8
//=======================================

// null routine
void idle() {}

void (* volatile isr)() = idle;
#ifdef SHOW_IRQ_LIFE
volatile long lifeCounter = 0;
#endif

//------------------------------------------------

const boolean positions [4][4] = {
  {true, false, false, true},
  {true, true, false, false},
  {false, true, true, false},
  {false, false, true, true}
};

const unsigned long ZERO_POSITION = ((unsigned long)-1) / 2;
volatile unsigned long currentPosition = ZERO_POSITION;
volatile unsigned long targetPosition = ZERO_POSITION;

const int STEP_PINS[4] = {2, 3, 4, 5};

void setStepOutputForPosition(int pos) {
  if (pos < 0 || pos > 3){
    Serial.println("ERROR, ILLEGAL POSITION REQUESTED");
    return;
  }
  
  for (int pinID = 0; pinID < 4; pinID++) {
    digitalWrite(STEP_PINS[pinID], positions[pos][pinID]);
  }
}

void stepMotor()
{
  if (targetPosition != currentPosition) {
    if (targetPosition > currentPosition) {
      currentPosition++;
    } else {
      currentPosition--;
    }
    setStepOutputForPosition(currentPosition % 4);
  } else {
    // got there...
    isr = idle;
  }
}

void initStepper() {
  for (int i = 0; i < 4; i++) {
    pinMode(STEP_PINS[i],OUTPUT);
  }
  setStepOutputForPosition(currentPosition % 4);
  configureTimerInterrupt();
  startMotorInterrupts();
}

//------------------------------------------------

// master interrupt service routine 
ISR(TIMER1_COMPA_vect)
{
#ifdef SHOW_IRQ_LIFE
  lifeCounter++;
  if (lifeCounter % 300 == 0) {
      digitalWrite(LED_PIN, digitalRead(LED_PIN) ^ 1);
  }
#endif
  // call supporting mode behavior
  (*isr)();
}

void stopMotorInterrupts() {
  TIMSK1 &= ~(1 << OCIE1A);
}

void startMotorInterrupts() {
  TIMSK1 |= (1 << OCIE1A);
}

void configureTimerInterrupt() {
  int tccr1bTemplate = TCCR1B & TCCR1B_PRESCALE_MASK;
    TCCR1A = 0;
    OCR1A = countOCR1A;
    TCNT1 = 0;
    tccr1bTemplate |= prescale;
    TCCR1B = tccr1bTemplate;
}

void setModeTarget(unsigned long target) {
  Serial.println("Run to target...");
  targetPosition = target;
  isr = stepMotor;
}

void setup() {
  Serial.begin(9600);
  Serial.println("Command (stop, up, down, pos):");

  pinMode(LED_PIN, OUTPUT);
  initStepper();
}

char inData[128];
int dataCount = 0;

void loop() {
  while (Serial.available() > 0) {
    char charRead = Serial.read();
    if (charRead == 0x0A || charRead == 0x0D) {
      inData[dataCount] = 0;
      if (strcmp(inData, "stop") == 0) {
        targetPosition = currentPosition;
        isr = idle;
      } else if (strcmp(inData, "down") == 0) {
        setModeTarget(currentPosition - 1000);
      } else if (strcmp(inData, "up") == 0) {
        setModeTarget(currentPosition + 1000);
      } else if (strcmp(inData, "pos") == 0) {
        Serial.print("Position:        " );
        Serial.println(currentPosition);
        Serial.print("Target position: ");
        Serial.println(targetPosition);
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

