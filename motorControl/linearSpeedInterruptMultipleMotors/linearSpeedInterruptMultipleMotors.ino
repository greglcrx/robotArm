#define BASE              0
#define EPAULE_DROITE     1
#define EPAULE_GAUCHE     2
#define COUDE             3
#define TIBIA             4
#define POIGNET           5

// For RAMPS 1.4
#define B_DIR_PIN          52
#define B_STEP_PIN         53
//#define B_ENABLE_PIN       30

#define ED_DIR_PIN         48
#define ED_STEP_PIN        49
//#define ED_ENABLE_PIN      38

#define EG_DIR_PIN          46
#define EG_STEP_PIN         47
//#define EG_ENABLE_PIN       56

#define C_DIR_PIN          42
#define C_STEP_PIN         43
//#define C_ENABLE_PIN       7

#define T_DIR_PIN          40
#define T_STEP_PIN         41
//#define Z_ENABLE_PIN       62

#define P_DIR_PIN          28
#define P_STEP_PIN         26
//#define A_ENABLE_PIN       24

#define GND_1   51
#define GND_2   50
#define GND_3   44
#define GND_4   45
#define GND_5   38
#define GND_6   39

#define X_STEP_HIGH             PORTF |=  0b00000001;
#define X_STEP_LOW              PORTF &= ~0b00000001;

#define Y_STEP_HIGH             PORTF |=  0b01000000;
#define Y_STEP_LOW              PORTF &= ~0b01000000;

#define Z_STEP_HIGH             PORTL |=  0b00001000;
#define Z_STEP_LOW              PORTL &= ~0b00001000;

#define A_STEP_HIGH             PORTA |=  0b00010000;
#define A_STEP_LOW              PORTA &= ~0b00010000;

#define B_STEP_HIGH             PORTC |=  0b00000010;
#define B_STEP_LOW              PORTC &= ~0b00000010;

#define C_STEP_HIGH             PORTL |=  0b00000100;
#define C_STEP_LOW              PORTL &= ~0b00000100;

#define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1 << OCIE1A);
#define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1 << OCIE1A);

struct stepperInfo {
  // externally defined parameters
  float acceleration;
  volatile unsigned int minStepInterval;   // ie. max speed, smaller is faster
  void (*dirFunc)(int);
  void (*stepFunc)();

  // derived parameters
  unsigned int c0;                // step interval for first step, determines acceleration
  long stepPosition;              // current position of stepper (total of all movements taken so far)

  // per movement variables (only changed once per movement)
  volatile int dir;                        // current direction of movement, used to keep track of position
  volatile unsigned int totalSteps;        // number of steps requested for current movement
  volatile bool movementDone = false;      // true if the current movement has been completed (used by main program to wait for completion)
  volatile unsigned int rampUpStepCount;   // number of steps taken to reach either max speed, or half-way to the goal (will be zero until this number is known)

  // per iteration variables (potentially changed every interrupt)
  volatile unsigned int n;                 // index in acceleration curve, used to calculate next interval
  volatile float d;                        // current interval length
  volatile unsigned long di;               // above variable truncated
  volatile unsigned int stepCount;         // number of steps completed in current movement
};

void edStep(){
  digitalWrite(ED_STEP_PIN, HIGH);
  digitalWrite(ED_STEP_PIN, LOW);
}

void edDir(int dir){
  digitalWrite(ED_DIR_PIN, dir);
}

void egStep(){
  digitalWrite(EG_STEP_PIN, HIGH);
  digitalWrite(EG_STEP_PIN, LOW);
}

void egDir(int dir){
  digitalWrite(EG_DIR_PIN, dir);
}

void tStep(){
  digitalWrite(T_STEP_PIN, HIGH);
  digitalWrite(T_STEP_PIN, LOW);
}

void tDir(int dir){
  digitalWrite(T_DIR_PIN, dir);
}

void pStep(){
  digitalWrite(P_STEP_PIN, HIGH);
  digitalWrite(P_STEP_PIN, LOW);
}

void pDir(int dir){
  digitalWrite(P_DIR_PIN, dir);
}

void bStep(){
  digitalWrite(B_STEP_PIN, HIGH);
  digitalWrite(B_STEP_PIN, LOW);
}

void bDir(int dir){
  digitalWrite(B_DIR_PIN, dir);
}

void cStep(){
  digitalWrite(C_STEP_PIN, HIGH);
  digitalWrite(C_STEP_PIN, LOW);
}

void cDir(int dir){
  digitalWrite(C_DIR_PIN, dir);
}

void resetStepperInfo( stepperInfo& si ) {
  si.n = 0;
  si.d = 0;
  si.di = 0;
  si.stepCount = 0;
  si.rampUpStepCount = 0;
  si.totalSteps = 0;
  si.stepPosition = 0;
  si.movementDone = false;
}

#define NUM_STEPPERS 6

volatile stepperInfo steppers[NUM_STEPPERS];

void setup() {

  pinMode(ED_STEP_PIN,   OUTPUT);
  pinMode(ED_DIR_PIN,    OUTPUT);
//  pinMode(ED_ENABLE_PIN, OUTPUT);

  pinMode(EG_STEP_PIN,   OUTPUT);
  pinMode(EG_DIR_PIN,    OUTPUT);
//  pinMode(EG_ENABLE_PIN, OUTPUT);

  pinMode(T_STEP_PIN,   OUTPUT);
  pinMode(T_DIR_PIN,    OUTPUT);
//  pinMode(Z_ENABLE_PIN, OUTPUT);

  pinMode(P_STEP_PIN,   OUTPUT);
  pinMode(P_DIR_PIN,    OUTPUT);
//  pinMode(A_ENABLE_PIN, OUTPUT);

  pinMode(B_STEP_PIN,   OUTPUT);
  pinMode(B_DIR_PIN,    OUTPUT);
//  pinMode(B_ENABLE_PIN, OUTPUT);

  pinMode(C_STEP_PIN,   OUTPUT);
  pinMode(C_DIR_PIN,    OUTPUT);
//  pinMode(C_ENABLE_PIN, OUTPUT);

  pinMode(GND_1,    OUTPUT);
  pinMode(GND_2,    OUTPUT);
  pinMode(GND_3,    OUTPUT);
  pinMode(GND_4,    OUTPUT);
  pinMode(GND_5,    OUTPUT);
  pinMode(GND_6,    OUTPUT);

  digitalWrite(GND_1, LOW);
  digitalWrite(GND_2, LOW);
  digitalWrite(GND_3, LOW);
  digitalWrite(GND_4, LOW);
  digitalWrite(GND_5, LOW);
  digitalWrite(GND_6, LOW);
//  digitalWrite(ED_ENABLE_PIN, LOW);
//  digitalWrite(EG_ENABLE_PIN, LOW);
//  digitalWrite(Z_ENABLE_PIN, LOW);
//  digitalWrite(A_ENABLE_PIN, LOW);
//  digitalWrite(B_ENABLE_PIN, LOW);
//  digitalWrite(C_ENABLE_PIN, LOW);

  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 1000;                             // compare value
  TCCR1B |= (1 << WGM12);                   // CTC mode
  TCCR1B |= ((1 << CS11) | (1 << CS10));    // 64 prescaler
  interrupts();

  steppers[0].dirFunc = bDir;
  steppers[0].stepFunc = bStep;
  steppers[0].acceleration = 1000;
  steppers[0].minStepInterval = 50;

  steppers[5].dirFunc = pDir;
  steppers[5].stepFunc = pStep;
  steppers[5].acceleration = 4000;
  steppers[5].minStepInterval = 50;

  steppers[3].dirFunc = cDir;
  steppers[3].stepFunc = cStep;
  steppers[3].acceleration = 1000;
  steppers[3].minStepInterval = 250;

  steppers[1].dirFunc = edDir;
  steppers[1].stepFunc = edStep;
  steppers[1].acceleration = 1000;
  steppers[1].minStepInterval = 250;

  steppers[2].dirFunc = egDir;
  steppers[2].stepFunc = egStep;
  steppers[2].acceleration = 1000;
  steppers[2].minStepInterval = 250;

  steppers[4].dirFunc = tDir;
  steppers[4].stepFunc = tStep;
  steppers[4].acceleration = 1000;
  steppers[4].minStepInterval = 450;
}

void resetStepper(volatile stepperInfo& si) {
  si.c0 = si.acceleration;
  si.d = si.c0;
  si.di = si.d;
  si.stepCount = 0;
  si.n = 0;
  si.rampUpStepCount = 0;
  si.movementDone = false;
}

volatile byte remainingSteppersFlag = 0;

void prepareMovement(int whichMotor, int steps) {
  volatile stepperInfo& si = steppers[whichMotor];
  si.dirFunc( steps < 0 ? HIGH : LOW );
  si.dir = steps > 0 ? 1 : -1;
  si.totalSteps = abs(steps);
  resetStepper(si);
  remainingSteppersFlag |= (1 << whichMotor);
}

volatile byte nextStepperFlag = 0;

volatile int ind = 0;
volatile unsigned int intervals[100];

void setNextInterruptInterval() {

  bool movementComplete = true;

  unsigned int mind = 999999;
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ((1 << i) & remainingSteppersFlag) && steppers[i].di < mind ) {
      mind = steppers[i].di;
    }
  }

  nextStepperFlag = 0;
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ! steppers[i].movementDone )
      movementComplete = false;

    if ( ((1 << i) & remainingSteppersFlag) && steppers[i].di == mind )
      nextStepperFlag |= (1 << i);
  }

  if ( remainingSteppersFlag == 0 ) {
    OCR1A = 65500;
  }

  OCR1A = mind;
}

ISR(TIMER1_COMPA_vect)
{
  unsigned int tmpCtr = OCR1A;

  OCR1A = 65500;

  for (int i = 0; i < NUM_STEPPERS; i++) {

    if ( ! ((1 << i) & remainingSteppersFlag) )
      continue;

    if ( ! (nextStepperFlag & (1 << i)) ) {
      steppers[i].di -= tmpCtr;
      continue;
    }

    volatile stepperInfo& s = steppers[i];

    if ( s.stepCount < s.totalSteps ) {
      s.stepFunc();
      s.stepCount++;
      s.stepPosition += s.dir;
      if ( s.stepCount >= s.totalSteps ) {
        s.movementDone = true;
        remainingSteppersFlag &= ~(1 << i);
      }
    }

    if ( s.rampUpStepCount == 0 ) {
      s.n++;
      s.d = s.d - (2 * s.d) / (4 * s.n + 1);
      if ( s.d <= s.minStepInterval ) {
        s.d = s.minStepInterval;
        s.rampUpStepCount = s.stepCount;
      }
      if ( s.stepCount >= s.totalSteps / 2 ) {
        s.rampUpStepCount = s.stepCount;
      }
    }
    else if ( s.stepCount >= s.totalSteps - s.rampUpStepCount ) {
      s.d = (s.d * (4 * s.n + 1)) / (4 * s.n + 1 - 2);
      s.n--;
    }

    s.di = s.d; // integer
  }

  setNextInterruptInterval();

  TCNT1  = 0;
}

void runAndWait() {
  setNextInterruptInterval();
  while ( remainingSteppersFlag );
}

void loop() {

  TIMER1_INTERRUPTS_ON

  //prepareMovement( 4,  -400 );
  //prepareMovement( 0,  800);
  //prepareMovement( 1,  -400);
  //prepareMovement( 2,  400);

  prepareMovement(COUDE, 1600); 
  runAndWait();

  while(true);
}
