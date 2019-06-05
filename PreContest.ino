//PRE CONTEST LATEST VERSION//2.5.19
#include <TimerFour.h>
#include <TimerThree.h>
#include <math.h>
#include <Servo.h>
#include <NRFLite.h>

#define stPinL 4
#define stPinR 3
#define stDirPinL 6
#define stDirPinR 5
#define enablePin 7

#define servoAttach 2//wire which controls servo

#define TICK_COUNTS 4000

#define trigUS A0
#define echoPinUSf 18
#define echoPinUSl 19
#define echoPinUSr 20
#define startPin 49

#define spaceToObstacle 20//the distance to a obstacle to make the bot stop
#define armsOpen 145//degrees of servo when arms are open
#define armsClosed 90//degrees of servo when arms are close

volatile int count = 0;
volatile long echo_start = 0;
volatile int distanceF = 0;
volatile int distanceR = 0;
volatile int distanceL = 0;
volatile int trigger_time_count = 0;

/*
  // RF-Activation || BEGIN//


  Radio    Arduino
  CE    -> 9
  CSN   -> 10 (Hardware SPI SS)
  MOSI  -> 51 (Hardware SPI MOSI)
  MISO  -> 50 (Hardware SPI MISO)
  SCK   -> 52 (Hardware SPI SCK)
  IRQ   -> No connection
  VCC   -> No more than 3.6 volts
  GND   -> GND
*/

const static uint8_t RADIO_ID = 38;                   // BIG BOT
const static uint8_t DESTINATION_RADIO_ID_LILB = 86;  // this bot

const static uint8_t PIN_RADIO_CE = 9;
const static uint8_t PIN_RADIO_CSN = 10;

struct RadioPacket
{
  uint8_t FromRadioId;
  uint8_t TeamID;
  uint16_t FailedTxCount;
};

NRFLite radio;
RadioPacket radioData_LILB;

int team = NULL;

// RF-Activation || END//

Servo arms;

const float tPerStep = 540.0;
const float tPerStepTurn = 420.0;

const float wheelDiameter = 108; //mm
const float outerWheelDistance = 21.5; //cm
const float wheelWidth = 50; //mm
const float stepsPerRev = 1000;

const float midWheelDistance = outerWheelDistance * 10 - wheelWidth;
const float distancePerStep = PI * wheelDiameter / stepsPerRev;
const float anglePerStep = 360 / ((PI * midWheelDistance) / distancePerStep);
const float outerRadius = 40;//cm //radius to outer side of bot for curves
const float innerRadius = outerRadius - outerWheelDistance;


/////////
//Setup//
/////////

void setupRF() {
  if (!radio.init(DESTINATION_RADIO_ID_LILB, PIN_RADIO_CE, PIN_RADIO_CSN))
  {
    Serial.println("Cannot communicate with radio");
    //while (1){delay(1000);}
  }
}


void initUS() {
  pinMode(trigUS, OUTPUT);
  pinMode(echoPinUSf, INPUT);
  pinMode(echoPinUSr, INPUT);
  pinMode(echoPinUSl, INPUT);
  digitalWrite(trigUS, LOW);
  attachInterrupt(digitalPinToInterrupt(echoPinUSf), echo_interruptF, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echoPinUSr), echo_interruptR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echoPinUSl), echo_interruptL, CHANGE);  // Attach interrupt for US
}

void configBot() {
  Serial.begin(115200);
  Serial.println("Init begin");
  Timer3.initialize(1000000); // 1 sec repeat rate
  Timer4.initialize(50);
  Timer4.attachInterrupt(trigger_pulse);
  initUS();
  setupSteppersAndServo();
  setupRF();
  Serial.println("config done");
}

void setupSteppersAndServo() {
  Serial.println("Stepper IN");
  pinMode(stPinL , OUTPUT);
  pinMode(stPinR , OUTPUT);
  pinMode(stDirPinL , OUTPUT);
  pinMode(stDirPinR , OUTPUT);
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH);

  arms.attach(servoAttach);
  arms.write(armsClosed);
  Serial.println("Stepper OUT");

  pinMode(startPin, INPUT_PULLUP);
}

void setup() {
  configBot();
  Serial.println("Init done");
  Serial.println("Init done");
  Serial.println("Init done");
  
  //team = 2;
  
  Serial.print("TEAM = ");
  Serial.println(team);
  Serial.print("TEAM-STATE = ");
  Serial.println(team == NULL);
  while (team == NULL) { // As long as we dont't have an ID
    while (radio.hasData())
    {
        radio.readData(&radioData_LILB);

        String msg = "Radio ";
        msg += radioData_LILB.FromRadioId;
        msg += ", ";
        msg += radioData_LILB.TeamID;
        msg += " Team, ";
        msg += radioData_LILB.FailedTxCount;
        msg += " Failed TX";

        Serial.println(msg); //Print data of packet

        if (radioData_LILB.FromRadioId == RADIO_ID) { //Validate that the sender was OUR bot
          team = radioData_LILB.TeamID; //activate and save the team ID
          Serial.println("Activated!");
        }
    }
    }
    
  Serial.print("la");
  Serial.println(digitalRead(startPin));
  while (digitalRead(startPin) == LOW) {
  Serial.println(digitalRead(startPin));
    delay(50);
  }
  Serial.println(digitalRead(startPin));
  Serial.print("la");
  Timer3.attachInterrupt(timerInt);
}

//////////////////////////
//What happens reapeated//
//////////////////////////

void loop() {

  if (team == 2) {
    safePathOrange();
    while (true) {
      delay(1000);
    }
  }
  if (team == 1) {
    safePathPurple();
    while (true) {
      delay(1000);
    }

  }
  while(true){
    delay(1000);
  }
}

  ////////////////////////
  //Hardcode for driving//
  ////////////////////////


void safePathOrange() {
    driveWoCoA(-250);
    turnWoCoA(-90);
    driveWoCoA(-200);
    driveCoANoLeft(1300);
    turnWoCoA(90);
    driveWoCoA(-350);
    driveCoA(700);
    openArms();
    driveCoA(320);
    turnWoCoA(95);
    driveCoA(1500);

    
    driveWoCoA(-350);
    turnWoCoA(-90);
    driveCoA(300);
    turnWoCoA(-180);
    driveWoCoA(-400);
    driveCoA(100);
    turnWoCoA(90);
    driveWoCoA(-900);
    driveCoA(100);
    turnWoCoA(-90);
    driveWoCoA(-800);
    driveCoA(120);
    turnWoCoA(90);
    driveWoCoA(-200);
    driveWoCoA(8000);
    driveWoCoA(-150);
    turnWoCoA(-180);
    driveWoCoA(-300);
}

void safePathPurple() {
    driveWoCoA(-250);
    turnWoCoA(90);
    driveWoCoA(-200);
    driveCoANoRight(1300);
    turnWoCoA(-90);
    driveWoCoA(-350);
    driveCoA(700);
    openArms();
    driveCoA(320);
    turnWoCoA(-95);
    driveCoA(1500);

    
    driveWoCoA(-350);
    turnWoCoA(90);
    driveCoA(300);
    turnWoCoA(180);
    driveWoCoA(-400);
    driveCoA(100);
    turnWoCoA(-90);
    driveWoCoA(-900);
    driveCoA(100);
    turnWoCoA(90);
    driveWoCoA(-800);
    driveCoA(120);
    turnWoCoA(-90);
    driveWoCoA(-200);
    driveWoCoA(8000);
    driveWoCoA(-150);
    turnWoCoA(180);
    driveWoCoA(-300);
}


  /////////////////////////////
  //Driving without avoidance//
  /////////////////////////////

  void driveWoCoA(int dist) {
    unsigned int steps = (abs(dist) / distancePerStep ) + 1;
    Serial.print("Steps to do: ");
    Serial.println(steps);
    bool pol = dist > 0;
    driveWoCoA(steps , pol);
  }

  void driveWoCoA(unsigned int steps , bool dir) {
    digitalWrite(enablePin, LOW);
    digitalWrite(stDirPinL , dir ? HIGH : LOW);
    digitalWrite(stDirPinR , dir ? LOW : HIGH);

    delay(10);

    float a = 0;
    for (unsigned int i = 0; i < steps; i++) {
      a = 0;
      if (i < 300 && i <= steps / 2)
        a = tPerStep * 0.004 * (300 - i);
      if (steps - i < 300 && i > steps / 2)
        a = tPerStep * 0.0055 * (300 - (steps - i));

      delayMicroseconds(tPerStep + a);
      digitalWrite(stPinL , HIGH);
      digitalWrite(stPinR , HIGH);
      delayMicroseconds(tPerStep + a);
      digitalWrite(stPinL , LOW);
      digitalWrite(stPinR , LOW);
    }
    digitalWrite(enablePin, HIGH);
  }

  //////////////////////////
  //Driving with avoidance//
  //////////////////////////

  void driveCoA(int dist) {
    /*@FABIAN
      Berechnungsformel benutzen mit variablen Werten (siehe oben)*/
    unsigned int steps = (abs(dist) / distancePerStep ) + 1;
    Serial.println(steps);
    bool pol = dist > 0;
    if (dist > 0) {
      driveCoA(steps , pol, dist);
    } else {
      driveWoCoA(steps , dist);
    }
  }

  void driveCoA(unsigned int steps, bool dir, int dist) {
    digitalWrite(enablePin, LOW);
    float distanceStillToDrive = float(dist);
    digitalWrite(stDirPinL, dir ? HIGH : LOW);
    digitalWrite(stDirPinR, dir ? LOW : HIGH);

    delay(10);

    float distancePerStep = 32.3 / 1036;
    float a = 0;
    for (unsigned int i = 0; i < steps; i++) {
      distanceStillToDrive = distanceStillToDrive - distancePerStep;
      if (i % 100 == 0) {
        if (((distanceF + 5) < int(distanceStillToDrive) && distanceF < 15) ||
            ((distanceF + 8) < int(distanceStillToDrive) && distanceR < 18) ||
            ((distanceF + 8) < int(distanceStillToDrive) && distanceL < 18)) {
          steps += 30;
          for (int d = 0; d < 6; d++) {
            i++;
            a = tPerStep * 0.0055 * d;
            digitalWrite(stPinR , HIGH);
            digitalWrite(stPinL , HIGH);
            delayMicroseconds(tPerStep + a);
            digitalWrite(stPinL , LOW);
            digitalWrite(stPinR , LOW);
          }
          while (((distanceF + 5) < int(distanceStillToDrive) && distanceF < 15) ||
                 ((distanceF + 8) < int(distanceStillToDrive) && distanceR < 18) ||
                 ((distanceF + 8) < int(distanceStillToDrive) && distanceL < 18)) {
            delay(250);
          }
          for (int d = 0; d < 3; d++) {
            i++;
            a = tPerStep * 0.0055 * (2 - d);
            digitalWrite(stPinR , HIGH);
            digitalWrite(stPinL , HIGH);
            delayMicroseconds(tPerStep + a);
            digitalWrite(stPinL , LOW);
            digitalWrite(stPinR , LOW);
          }
        }
      }
      a = 0;
      if (i < 400 && i <= steps / 2)
        a = tPerStep * 0.0025 * (400 - i);
      if (steps - i < 400 && i > steps / 2)
        a = tPerStep * 0.0025 * (400 - (steps - i));

      digitalWrite(stPinL, HIGH);
      digitalWrite(stPinR, HIGH);
      delayMicroseconds(tPerStep + a);
      digitalWrite(stPinL, LOW);
      digitalWrite(stPinR, LOW);
      delayMicroseconds(tPerStep + a);
    }
    digitalWrite(enablePin, HIGH);
  }


void driveCoANoLeft(int dist) {
    /*@FABIAN
      Berechnungsformel benutzen mit variablen Werten (siehe oben)*/
    unsigned int steps = (abs(dist) / distancePerStep ) + 1;
    Serial.println(steps);
    bool pol = dist > 0;
    if (dist > 0) {
      driveCoANoLeft(steps , pol, dist);
    } else {
      driveWoCoA(steps , dist);
    }
  }

  void driveCoANoLeft(unsigned int steps, bool dir, int dist) {
    digitalWrite(enablePin, LOW);
    float distanceStillToDrive = float(dist);
    digitalWrite(stDirPinL, dir ? HIGH : LOW);
    digitalWrite(stDirPinR, dir ? LOW : HIGH);

    delay(10);

    float distancePerStep = 32.3 / 1036;
    float a = 0;
    for (unsigned int i = 0; i < steps; i++) {
      distanceStillToDrive = distanceStillToDrive - distancePerStep;
      if (i % 100 == 0) {
        if (((distanceF + 5) < int(distanceStillToDrive) && distanceF < 15) ||
            ((distanceF + 8) < int(distanceStillToDrive) && distanceR < 18)) {
          steps += 30;
          for (int d = 0; d < 6; d++) {
            i++;
            a = tPerStep * 0.0055 * d;
            digitalWrite(stPinR , HIGH);
            digitalWrite(stPinL , HIGH);
            delayMicroseconds(tPerStep + a);
            digitalWrite(stPinL , LOW);
            digitalWrite(stPinR , LOW);
          }
          while (((distanceF + 5) < int(distanceStillToDrive) && distanceF < 15) ||
                 ((distanceF + 8) < int(distanceStillToDrive) && distanceR < 18)) {
            delay(250);
          }
          for (int d = 0; d < 3; d++) {
            i++;
            a = tPerStep * 0.0055 * (2 - d);
            digitalWrite(stPinR , HIGH);
            digitalWrite(stPinL , HIGH);
            delayMicroseconds(tPerStep + a);
            digitalWrite(stPinL , LOW);
            digitalWrite(stPinR , LOW);
          }
        }
      }
      a = 0;
      if (i < 400 && i <= steps / 2)
        a = tPerStep * 0.0025 * (400 - i);
      if (steps - i < 400 && i > steps / 2)
        a = tPerStep * 0.0025 * (400 - (steps - i));

      digitalWrite(stPinL, HIGH);
      digitalWrite(stPinR, HIGH);
      delayMicroseconds(tPerStep + a);
      digitalWrite(stPinL, LOW);
      digitalWrite(stPinR, LOW);
      delayMicroseconds(tPerStep + a);
    }
    digitalWrite(enablePin, HIGH);
  }


void driveCoANoRight(int dist) {
    /*@FABIAN
      Berechnungsformel benutzen mit variablen Werten (siehe oben)*/
    unsigned int steps = (abs(dist) / distancePerStep ) + 1;
    Serial.println(steps);
    bool pol = dist > 0;
    if (dist > 0) {
      driveCoANoRight(steps , pol, dist);
    } else {
      driveWoCoA(steps , dist);
    }
  }

  void driveCoANoRight(unsigned int steps, bool dir, int dist) {
    digitalWrite(enablePin, LOW);
    float distanceStillToDrive = float(dist);
    digitalWrite(stDirPinL, dir ? HIGH : LOW);
    digitalWrite(stDirPinR, dir ? LOW : HIGH);

    delay(10);

    float distancePerStep = 32.3 / 1036;
    float a = 0;
    for (unsigned int i = 0; i < steps; i++) {
      distanceStillToDrive = distanceStillToDrive - distancePerStep;
      if (i % 100 == 0) {
        if (((distanceF + 5) < int(distanceStillToDrive) && distanceF < 15) ||
            ((distanceF + 8) < int(distanceStillToDrive) && distanceL < 18)) {
          steps += 30;
          for (int d = 0; d < 6; d++) {
            i++;
            a = tPerStep * 0.0055 * d;
            digitalWrite(stPinR , HIGH);
            digitalWrite(stPinL , HIGH);
            delayMicroseconds(tPerStep + a);
            digitalWrite(stPinL , LOW);
            digitalWrite(stPinR , LOW);
          }
          while (((distanceF + 5) < int(distanceStillToDrive) && distanceF < 15) ||
                 ((distanceF + 8) < int(distanceStillToDrive) && distanceL < 18)) {
            delay(250);
          }
          for (int d = 0; d < 3; d++) {
            i++;
            a = tPerStep * 0.0055 * (2 - d);
            digitalWrite(stPinR , HIGH);
            digitalWrite(stPinL , HIGH);
            delayMicroseconds(tPerStep + a);
            digitalWrite(stPinL , LOW);
            digitalWrite(stPinR , LOW);
          }
        }
      }
      a = 0;
      if (i < 400 && i <= steps / 2)
        a = tPerStep * 0.0025 * (400 - i);
      if (steps - i < 400 && i > steps / 2)
        a = tPerStep * 0.0025 * (400 - (steps - i));

      digitalWrite(stPinL, HIGH);
      digitalWrite(stPinR, HIGH);
      delayMicroseconds(tPerStep + a);
      digitalWrite(stPinL, LOW);
      digitalWrite(stPinR, LOW);
      delayMicroseconds(tPerStep + a);
    }
    digitalWrite(enablePin, HIGH);
  }


  /////////////////////////////
  //Turning without avoidance//
  /////////////////////////////

  void turnWoCoA(short angle) {//postitive angle for clockwise rotation
    /*@FABIAN
      Berechnungsformel benutzen mit variablen Werten (siehe oben)*/
    unsigned int steps = abs(angle) / anglePerStep;
    bool pol = angle > 0;
    if (!pol) steps *= 1.083;
    if (pol) steps *= 1.097;
    Serial.println(steps);
    turnWoCoA(steps, pol);
    delay(100);
  }

  void turnWoCoA(unsigned int steps, bool left) {
    digitalWrite(enablePin, LOW);
    /*@FABIAN
      siehe oben für richtige polarisation*/
    digitalWrite(stDirPinL, left ? HIGH : LOW);
    digitalWrite(stDirPinR, left ? HIGH : LOW);

    delay(10);

    const unsigned int aOverSteps = 1490;

    float a = 0; //for acceleration
    for (unsigned int i = 0; i < steps; i++) {
      a = 0;
      if (i < aOverSteps && i <= steps / 2) {
        a = (tPerStepTurn / aOverSteps) * 2.0 * (aOverSteps - i);
      }
      if (steps - i < aOverSteps && i > steps / 2) {
        a = (tPerStepTurn / aOverSteps) * 2.3 * (aOverSteps - (steps - i));
      }

      digitalWrite(stPinL, HIGH);
      digitalWrite(stPinR , HIGH);
      delayMicroseconds(tPerStepTurn + a);
      digitalWrite(stPinL , LOW);
      digitalWrite(stPinR , LOW);
      delayMicroseconds(tPerStepTurn + a);
    }
    digitalWrite(enablePin, HIGH);
  }


  /////////
  //Servo//
  /////////

  void openArms() {
    Serial.println("Opening Arms");
    arms.write(armsOpen);
    delay(500);
    arms.write(armsClosed);
  }

  void stopAllTimers() {
    TCCR0B = 0;
    TCCR1B = 0;
    TCCR2B = 0;
    Serial.println("Stopped Timer0");
    Serial.println("Stopped Timer1");
    Serial.println("Stopped Timer2");
    Timer3.detachInterrupt();
  }

  void timerInt() {
    count++;
    Serial.println(count);

    if (count >= 100) {
      stopAllTimers();
    }
  }

  void echo_interruptF(){
    switch (digitalRead(echoPinUSf))                     // Test to see if the signal is high or low
    {
      case HIGH:                                      // High so must be the start of the echo pulse
        echo_start = micros();                        // Save the start time
        break;

      case LOW:                                       // Low so must be the end of hte echo pulse
        distanceF = (micros() - echo_start) / 60;
        break;
    }
  }

  void echo_interruptR()
  {
    switch (digitalRead(echoPinUSr))                     // Test to see if the signal is high or low
    {
      case HIGH:                                      // High so must be the start of the echo pulse
        echo_start = micros();                        // Save the start time
        break;

      case LOW:                                       // Low so must be the end of hte echo pulse
        distanceR = (micros() - echo_start) / 60;
        break;
    }
  }

  void echo_interruptL()
  {
    switch (digitalRead(echoPinUSl))                     // Test to see if the signal is high or low
    {
      case HIGH:                                      // High so must be the start of the echo pulse
        echo_start = micros();                        // Save the start time
        break;

      case LOW:                                       // Low so must be the end of hte echo pulse
        distanceL = (micros() - echo_start) / 60;
        break;
    }
  }
  void trigger_pulse() {
    static volatile int state = 0;                 // State machine variable

    if (!(--trigger_time_count))                   // Count to 200mS
    { // Time out - Initiate trigger pulse
      trigger_time_count = TICK_COUNTS;           // Reload
      state = 1;                                  // Changing to state 1 initiates a pulse
    }

    switch (state)                                 // State machine handles delivery of trigger pulse
    {
      case 0:                                      // Normal state does nothing
        break;

      case 1:                                      // Initiate pulse
        digitalWrite(trigUS, HIGH);              // Set the trigger output high
        state = 2;                                // and set state to 2
        break;

      case 2:                                      // Complete the pulse
      default:
        digitalWrite(trigUS, LOW);               // Set the trigger output low
        state = 0;                                // and return state to normal 0
        break;
    }
  }
