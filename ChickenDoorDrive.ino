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
#define END_STOP              8
#define LDR_PORT              0
#define TCCR1B_PRESCALE_MASK 0xE0
#define DOOR_STROKE       14700

//=======================================
// speed 300.03 steps/second
const int countOCR1A  = 6666;  // 
const int prescale    = 0x0A;  // 8
const int hertz       = 300;
//=======================================

// null routine
void idle() {}

void (* volatile isr)() = idle;
volatile long lifeCounter = 0;
volatile int lightSampleTrigger = 0;

volatile int initISRcounter = 0;

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

void initialOpenDoor() {
  initISRcounter++;
  if (digitalRead(END_STOP)) {
    currentPosition++;
    setStepOutputForPosition(currentPosition % 4);
  } else {
    currentPosition = targetPosition = ZERO_POSITION;
    isr = idle;
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
  lifeCounter++;
  if (lifeCounter % hertz == 0) { // once per second
      digitalWrite(LED_PIN, digitalRead(LED_PIN) ^ 1);
      lightSampleTrigger++;
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

void closeDoor() {
  setModeTarget(currentPosition - DOOR_STROKE);
}

void setup() {
  Serial.begin(9600);
  Serial.println("Command (init, stop, up, down, debug):");

  pinMode(LED_PIN, OUTPUT);
  pinMode(END_STOP, INPUT_PULLUP);
  initStepper();
}

boolean day = true;
int lightThreshold = 800;
int lightHysteresis = 50;
const int lightInterval = 3;
const int LIGHT_SAMPLE_COUNT = 20;
int lightSamples[LIGHT_SAMPLE_COUNT];
int thisSample = 0;
int validSamples = 0;

boolean looksLikeNight() {
  // check if all samples indicate nighttime
  boolean night = true; // tenative
  // looking for dark
  for (int idx = 0; night && idx < LIGHT_SAMPLE_COUNT; idx++) {
    if (lightSamples[idx] < (lightThreshold + lightHysteresis)) {
      night = false;
    }
  }
  return night;  
}

boolean looksLikeDay() {
  // check if all samples indicate daytime
  boolean day = true; // tenative
  // looking for light
  for (int idx = 0; day && idx < LIGHT_SAMPLE_COUNT; idx++) {
    if (lightSamples[idx] > (lightThreshold - lightHysteresis)) {
      day = false;
    }
  }
  return day;  
}

void processLight(int triggerSample) {
  if (triggerSample % lightInterval == 0) {
    lightSamples[thisSample++] = analogRead(LDR_PORT);
    thisSample %= LIGHT_SAMPLE_COUNT;
    if (validSamples < LIGHT_SAMPLE_COUNT) {
      validSamples++;
    } else {
      if (day) {
        if (looksLikeNight()) {
          Serial.println("Going to bed");
          day = false;
          closeDoor();
        }
      } else {
        if (looksLikeDay()) {
          Serial.println("Waking up");
          day = true;
          isr = initialOpenDoor;
        }
      }
    }
  }
}

char inData[128];
int dataCount = 0;
int lastLightSample = 0;

void loop() {
  if (lightSampleTrigger != lastLightSample) {
    lastLightSample = lightSampleTrigger;
    processLight(lightSampleTrigger);  
  }
  
  while (Serial.available() > 0) {
    char charRead = Serial.read();
    if (charRead == 0x0A || charRead == 0x0D) {
      inData[dataCount] = 0;
      if (strcmp(inData, "stop") == 0) {
        targetPosition = currentPosition;
        isr = idle;
      } else if (strcmp(inData, "init") == 0) {
        isr = initialOpenDoor;
      } else if (strcmp(inData, "down") == 0) {
        setModeTarget(currentPosition - 1000);
      } else if (strcmp(inData, "up") == 0) {
        setModeTarget(currentPosition + 1000);
      } else if (strcmp(inData, "debug") == 0) {
        Serial.print("initISRcount: ");
        Serial.println(initISRcounter);
        Serial.print("Position:        " );
        Serial.println(currentPosition);
        Serial.print("Target position: ");
        Serial.println(targetPosition);
        boolean endStop = digitalRead(END_STOP);
        Serial.print("End stop is: ");
        Serial.println(endStop);
        if (validSamples >= LIGHT_SAMPLE_COUNT) {
          Serial.print("light readings:" );
          if (looksLikeDay()) Serial.println("day-like");
          if (looksLikeNight()) Serial.println("night-like");
          for (int idx = 0; idx < LIGHT_SAMPLE_COUNT; idx++) {
            Serial.println(lightSamples[idx]);
          }
        } else {
          Serial.println("Light not yet valid");
        }
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

