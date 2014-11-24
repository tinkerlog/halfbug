
#include "Arduino.h"

typedef struct {
  int currentPos;
  int startPos;
  float workingPos;
  int targetPos;
  float maxSpeed;
  float deltaSpeed;
  float currentSpeed;
  int delta;
  int decelerateDist;
  int waiting;
  int state;
  boolean up;
  Servo servo;
} ServoState;

