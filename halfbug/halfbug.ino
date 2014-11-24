/*

     MOSI     SCK
       SS     MISO
       TX     VIN   BAT-VCC
       RX     GND   BAT-GND
      RST     RST
      GND     5V
  LEFT  2     NC
  LIFT  3     NC
 RIGHT  4     A5
 HEADT  5     A4
 HEADL  6     A3
        7     A2
  PING  8     A1
        9     A0
       10     AREF
       11     3.3V
       12     13
          USB

Servo:   GND Vcc Signal

Ping:    GND 5V Signal 

 */

#include <Servo.h>
#include "halfbug.h"


#define LED 13
#define LIFT_SERVO 3
#define LEFT_LEG 2
#define RIGHT_LEG 4
#define HEAD_LIFT 5
#define HEAD_TURN 6
#define PING 8

#define   UP_BODY_LIFT 2000
#define DOWN_BODY_LIFT 1600

// 1700

#define MIN_HEAD_LIFT 1400
#define MID_HEAD_LIFT 1600
#define MAX_HEAD_LIFT 1700

#define  LEFT_HEAD_TURN 1850
#define   MID_HEAD_TURN 1500
#define RIGHT_HEAD_TURN 1150

#define FRONT_LEFT 1800
#define  BACK_LEFT 1300

#define FRONT_RIGHT 1000
#define  BACK_RIGHT 1500

#define STATE_IDLE         0
#define STATE_ACCELERATING 1
#define STATE_CONST_SPEED  2
#define STATE_DECELERATING 3
#define STATE_WAITING      4

#define STATE_CREATURE_IDLE        0
#define STATE_CREATURE_WAKING      1
#define STATE_CREATURE_WAKING_DONE 2
#define STATE_CREATURE_SCANNING    3
#define STATE_CREATURE_SCAN_LEFT   4
#define STATE_CREATURE_SCAN_MID    5
#define STATE_CREATURE_SCAN_RIGHT  6
#define STATE_CREATURE_WALKING     7
#define STATE_CREATURE_WALK_LEFT   8
#define STATE_CREATURE_WALK_RIGHT  9
#define STATE_CREATURE_TURN_LEFT   10
#define STATE_CREATURE_TURN_RIGHT  11
#define STATE_CREATURE_TURN_180    12

#define write(s) 
#define writeln(s) 
#define serial(baud)

//#define write(s)   (Serial.print(s))
//#define writeln(s) (Serial.println(s))
//#define serial(baud) (Serial.begin(baud))

Servo bodyLift;
Servo leftLeg;
Servo rightLeg;
Servo headLift;
Servo headTurn;

ServoState leftState;
ServoState rightState;
ServoState liftState;
ServoState headTurnState;
ServoState headLiftState;

char headCmds[20];
byte headPtr = 0;
char headCmd;
boolean headDone = true;

boolean walkDone = true;
char walkCmd;
char walkCmds[20];
byte walkPtr = 0;

void setup() { 
  pinMode(LED, OUTPUT);     

  initServo(&leftState, leftLeg, LEFT_LEG, BACK_LEFT, 0.6, 0.001);
  initServo(&liftState, bodyLift, LIFT_SERVO, DOWN_BODY_LIFT, 0.3, 0.001);
  initServo(&rightState, rightLeg, RIGHT_LEG, BACK_RIGHT, 0.6, 0.001);
  initServo(&headLiftState, headLift, HEAD_LIFT, MIN_HEAD_LIFT, 0.6, 0.001);
  initServo(&headTurnState, headTurn, HEAD_TURN, MID_HEAD_TURN, 0.6, 0.001);

  serial(9600);
  delay(7000);
  writeln("ok");
}

void initServo(ServoState* state, Servo servo, int pin, int pos, float maxSpeed, float deltaSpeed) {
  servo.attach(pin);
  servo.writeMicroseconds(pos);
  state->state = STATE_IDLE;
  state->currentPos = pos;
  state->targetPos = pos;
  state->servo = servo;
  state->maxSpeed = maxSpeed;
  state->deltaSpeed = deltaSpeed;
  delay(100);
}

void checkState(ServoState* s) {  

  switch (s->state) {
  case STATE_WAITING:
    if (s->waiting > 0) {
      s->waiting--;
      return;
    }  // NOTE: fall thru
    s->state = STATE_IDLE;
  case STATE_IDLE:
    if (s->currentPos != s->targetPos) {
      s->up = (s->targetPos > s->currentPos);
      s->state = STATE_ACCELERATING;
      s->currentSpeed = s->maxSpeed * 0.1;
      s->startPos = s->currentPos;
      s->workingPos = s->startPos;
    }
    break;
  case STATE_ACCELERATING:
    if (s->currentSpeed < s->maxSpeed) {
      s->currentSpeed += s->deltaSpeed;
    }
    else {
      s->decelerateDist = (s->up) ? s->currentPos - s->startPos : s->startPos - s->currentPos;
      s->state = STATE_CONST_SPEED;
    }
    break;
  case STATE_CONST_SPEED:
    if (( s->up && s->currentPos + s->decelerateDist > s->targetPos) || 
	(!s->up && s->currentPos - s->decelerateDist < s->targetPos)) {
      s->state = STATE_DECELERATING;
    }
    break;
  case STATE_DECELERATING:
    if (s->currentSpeed > 0.1) {
      s->currentSpeed -= s->deltaSpeed;
    }
    break;
  }

  if (s->state != STATE_IDLE && s->state != STATE_WAITING) {
    s->workingPos = (s->up) ? s->workingPos + s->currentSpeed : s->workingPos - s->currentSpeed;
    s->currentPos = s->workingPos;
    if ((s->up && s->currentPos >= s->targetPos) || (!s->up && s->currentPos <= s->targetPos)) {
      s->state = STATE_IDLE;
    }
    s->servo.writeMicroseconds(s->currentPos);
  }

}

void refresh() {
  checkState(&leftState);
  checkState(&rightState);
  checkState(&liftState);
  checkState(&headTurnState);
  checkState(&headLiftState);
}

long cm = 200;
long leftDistance;
long rightDistance;
long midDistance;
long nextScan;
boolean leftClose;
boolean midClose;
boolean rightClose;
int headCycle = 5;
int walkCycle = 0;

int creatureState = STATE_CREATURE_WAKING;

#define WALK_WAKEUP "WWudWWWW"
#define FORWARD     "fubdww"
#define WALK_LEFT   "lubdww"
#define WALK_RIGHT  "rubdww"
#define TURN_LEFT   "2u1dww"
#define TURN_RIGHT  "LuRdww"
#define TURN_180    "LuRdLuRdLuRdww"

#define SCAN_WAKEUP   "WWmcwdWWWWcWW"
#define SCAN_SLOW     "lwwrwwmWW"
#define SCAN_LEFT     "cl"
#define SCAN_RIGHT    "r"
#define SCAN_STRAIGHT "m"

//#define CLOSE_LIMIT 40
#define NEAR_LIMIT 60

void loop() {

  long now;

  refresh();

  switch (creatureState) {
  case STATE_CREATURE_IDLE:
    break;
  case STATE_CREATURE_WAKING:
    setHeadCmds(SCAN_WAKEUP);
    setWalkCmds(WALK_WAKEUP);
    creatureState = STATE_CREATURE_WAKING_DONE;
    break;
  case STATE_CREATURE_WAKING_DONE:
    if (headDone) {
      creatureState = STATE_CREATURE_WALKING;
    }
    break;
  case STATE_CREATURE_WALKING:
    now = millis();
    if (now > nextScan && isIdleWalking() && cm > NEAR_LIMIT) {
      cm = checkDistance();
      write("distance: "); 
      writeln(cm);
      nextScan = now + 500;
      digitalWrite(LED, (cm < NEAR_LIMIT) ? HIGH : LOW);
    }
    if (cm < NEAR_LIMIT) {
      // stopWalking(); 
      // stopScanning();
      creatureState = STATE_CREATURE_SCANNING;
      cm = 200;
      leftDistance = 0;
      rightDistance = 0;
      midDistance = 0;
      writeln("scanning");
    }
    else {
      if (walkDone) {
	writeln("WALKING");
	setWalkCmds(FORWARD);
      }
      if (headDone) {
	setHeadCmds(SCAN_SLOW);
      }
    }
    break;
  case STATE_CREATURE_SCANNING:
    if (headDone && walkDone) {
      writeln("SCANNING");
      setHeadCmds(SCAN_LEFT);
      creatureState = STATE_CREATURE_SCAN_LEFT;
    }
    break;
  case STATE_CREATURE_SCAN_LEFT:
    if (headDone) {
      leftDistance = checkDistance();
      leftClose = (leftDistance < NEAR_LIMIT);
      setHeadCmds(SCAN_RIGHT);
      creatureState = STATE_CREATURE_SCAN_RIGHT;
    }
    break;
  case STATE_CREATURE_SCAN_RIGHT:
    if (headDone) {
      rightDistance = checkDistance();
      rightClose = (rightDistance < NEAR_LIMIT);
      setHeadCmds(SCAN_STRAIGHT);
      creatureState = STATE_CREATURE_SCAN_MID;
    }    
    break;
  case STATE_CREATURE_SCAN_MID:
    if (headDone) {
      midDistance = checkDistance();
      midClose = (midDistance < NEAR_LIMIT);
      write("distance: l/m/r: ");
      write(leftDistance); write("/");
      write(midDistance); write("/");
      writeln(rightDistance);
      if (rightClose) {
	if (midClose) {
	  creatureState = (leftClose) ? STATE_CREATURE_TURN_180 : STATE_CREATURE_TURN_LEFT;
	}
	else {
	  creatureState = (leftClose) ? STATE_CREATURE_WALKING : STATE_CREATURE_WALK_LEFT;
	}
      }
      else {
	if (midClose) {
	  creatureState = (leftClose) ? STATE_CREATURE_TURN_RIGHT : STATE_CREATURE_TURN_LEFT;
	}
	else {
	  creatureState = (leftClose) ? STATE_CREATURE_WALK_RIGHT : STATE_CREATURE_WALKING;
	}
      }
      cm = 200;
    }    
    break;
  case STATE_CREATURE_WALK_LEFT:
    if (walkDone) {
      writeln("WALK LEFT");
      setHeadCmds(SCAN_LEFT);
      setWalkCmds(WALK_LEFT);
      creatureState = STATE_CREATURE_SCANNING;
    }
    break;
  case STATE_CREATURE_WALK_RIGHT:
    if (walkDone) {
      writeln("WALK RIGHT");
      setHeadCmds(SCAN_RIGHT);
      setWalkCmds(WALK_RIGHT);
      creatureState = STATE_CREATURE_SCANNING;
    }
    break;
  case STATE_CREATURE_TURN_LEFT:
    if (walkDone) {
      writeln("TURN LEFT+");
      setHeadCmds(SCAN_LEFT);
      setWalkCmds(TURN_LEFT);
      creatureState = STATE_CREATURE_SCANNING;
    }
    break;
  case STATE_CREATURE_TURN_RIGHT:
    if (walkDone) {
      writeln("TURN RIGHT+");
      setHeadCmds(SCAN_RIGHT);
      setWalkCmds(TURN_RIGHT);
      creatureState = STATE_CREATURE_SCANNING;
    }
    break;
  case STATE_CREATURE_TURN_180:
    if (walkDone) {
      setHeadCmds(SCAN_RIGHT);
      setWalkCmds(TURN_180);
      creatureState = STATE_CREATURE_SCANNING;
    }
    break;
  }

  checkHeadState();
  checkWalkState();

  // delay(1);
  delayMicroseconds(750);
  
}


void checkHeadState() {
  if (headLiftState.state == STATE_IDLE && headTurnState.state == STATE_IDLE) {
    if (headDone) {
      return;
    }
    headCmd = headCmds[headPtr];
    if (headCmd == 0) {
      headDone = true;
      headPtr = 0;
      return;
    }
    headPtr++;
    write("HEAD: "); writeln(headCmd);
    switch (headCmd) {
    case 'l':
      setTarget(&headTurnState, LEFT_HEAD_TURN, 100);
      break;
    case 'r':
      setTarget(&headTurnState, RIGHT_HEAD_TURN, 100);
      break;
    case 'm':
      setTarget(&headTurnState, MID_HEAD_TURN, 100);
      break;
    case 'u':
      setTarget(&headLiftState, MAX_HEAD_LIFT, 100);
      break;
    case 'd':
      setTarget(&headLiftState, MIN_HEAD_LIFT, 100);
      break;
    case 'c':
      setTarget(&headLiftState, MID_HEAD_LIFT, 100);
      break;
    case '1':
      setTarget(&headLiftState, MAX_HEAD_LIFT, 100);
      setTarget(&headTurnState, MID_HEAD_TURN, 100);
      break;
    case '2':
      setTarget(&headLiftState, MID_HEAD_LIFT, 100);
      setTarget(&headTurnState, RIGHT_HEAD_TURN, 100);
      break;
    case '3':
      setTarget(&headLiftState, MIN_HEAD_LIFT, 100);
      setTarget(&headTurnState, MID_HEAD_TURN, 100);
      break;
    case '4':
      setTarget(&headLiftState, MID_HEAD_LIFT, 100);
      setTarget(&headTurnState, LEFT_HEAD_TURN, 100);
      break;
    case 'w':
      setWaiting(&headTurnState, 200);
      break;
    case 'W':
      setWaiting(&headTurnState, 1000);
      break;
    }
  }
}


void checkWalkState() {
  if (isIdleWalking()) {
    // Serial.print("walk0: "); Serial.print(leftState.state); 
    // Serial.print(", "); Serial.print(rightState.state);
    // Serial.print(", "); Serial.println(liftState.state);
    if (walkDone) {
      return;
    }
    walkCmd = walkCmds[walkPtr];
    if (walkCmd == 0) {
      walkDone = true;
      walkPtr = 0;
      return;
    }
    // Serial.print("walk1: "); Serial.println(walkCmd);
    switch (walkCmd) {
    case 'f':
      setTarget(&leftState, FRONT_LEFT, 100, 1.4, 0.005);
      setTarget(&rightState, FRONT_RIGHT, 150, 1.2, 0.004);      
      break;
    case 'l':
      setTarget(&rightState, FRONT_RIGHT, 150, 1.2, 0.004);      
      break;
    case 'L':
      setTarget( &leftState, FRONT_LEFT, 100, 1.4, 0.005);
      setTarget(&rightState, BACK_RIGHT, 150, 1.2, 0.004);            
      break;
    case '1':
      setTarget( &leftState, FRONT_LEFT, 100, 0.5, 0.001);
      setTarget(&rightState, BACK_RIGHT, 150, 0.5, 0.001);            
      break;
    case 'r':
      setTarget(&leftState, FRONT_LEFT, 150, 1.2, 0.004);      
      break;
    case 'R':
      setTarget( &leftState, BACK_LEFT,   100, 0.5, 0.001);
      setTarget(&rightState, FRONT_RIGHT, 100, 0.5, 0.001);            
      break;
    case '2':
      setTarget( &leftState, BACK_LEFT,   100, 1.4, 0.005);
      setTarget(&rightState, FRONT_RIGHT, 100, 1.4, 0.005);            
      break;
    case 'b':
      setTarget(&leftState, BACK_LEFT, 100, 0.6, 0.001);
      setTarget(&rightState, BACK_RIGHT, 100, 0.6, 0.001);
      break;
    case 'u':
      setTarget(&liftState, UP_BODY_LIFT, 100, 0.3, 0.001);      
      break;
    case 'd':
      setTarget(&liftState, DOWN_BODY_LIFT, 0, 0.4, 0.005);
      break;
    case 'w':
      setWaiting(&leftState, 200);
      setWaiting(&rightState, 200);
      setWaiting(&liftState, 200);
      break;
    case 'W':
      setWaiting(&leftState, 1000);
      break;
    }
    walkPtr++;
  }
}

boolean isIdleWalking() {
  return ((leftState.state == STATE_IDLE) && (rightState.state == STATE_IDLE) && (liftState.state == STATE_IDLE));
}

void setHeadCmds(char* cmds) {
  strcpy(headCmds, cmds);
  headPtr = 0;
  headDone = false;
}

void setWalkCmds(char* cmds) {
  strcpy(walkCmds, cmds);
  walkPtr = 0;
  walkDone = false;
}

void setTarget(ServoState* state, int target, int waiting) {
  state->state = STATE_WAITING;
  state->targetPos = target;
  state->waiting = waiting;
}

void setWaiting(ServoState* state, int waiting) {
  state->state = STATE_WAITING;
  state->targetPos = state->currentPos;
  state->waiting = waiting;
}

void setTarget(ServoState* state, int target, int waiting, float maxSpeed, float deltaSpeed) {
  setTarget(state, target, waiting);
  state->maxSpeed = maxSpeed;
  state->deltaSpeed = deltaSpeed;
}

void stop(ServoState* state) {
  state->state = STATE_IDLE;
  state->targetPos = state->currentPos;
}

void stopWalking() {
  stop(&leftState);
  stop(&rightState);
  walkDone = true;
}

void stopScanning() {
  stop(&headTurnState);
  stop(&headLiftState);
  headDone = true;
}

// float cmSum = 0;

long checkDistance() {
  long duration, cm;

  pinMode(PING, OUTPUT);
  digitalWrite(PING, LOW);
  delayMicroseconds(2);
  digitalWrite(PING, HIGH);
  delayMicroseconds(5);
  digitalWrite(PING, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(PING, INPUT);
  duration = pulseIn(PING, HIGH);

  // convert the time into a distance
  cm = duration / 29 / 2;
  //float fcm = cm;
  //cmSum *= 0.9;
  //cmSum += fcm / 10;

  //Serial.print("cm: ");
  //Serial.print(cm);
  // Serial.print(", sum: ");
  // Serial.println(cmSum);
  //Serial.println();
  return cm;
  //return cmSum;
}
