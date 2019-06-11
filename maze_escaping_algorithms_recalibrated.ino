#include <NewPing.h>
#include <Servo.h>
#include <Ashbot.h>

#define SONAR_NUM     3                        // Number of sensors.
#define MAX_DISTANCE  100.0                    // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33                       // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define RS            0
#define FS            1
#define LS            2

const int LPIN     = 12;
const int RPIN     = 11;
const int LCENTER  = 1496;                     // Calibrated for left servo
const int RCENTER  = 1498;                     // Calibrated for right servo
const int LFAST_FW = 48;                       // Change from center for Left  servo to get ~30RPM forward direction
const int RFAST_FW = 48;                       // Change from center for Right servo to get ~30RPM forward direction
const int LSLOW_FW = 19;                       // Change from center for Left  servo to get ~12RPM forward direction
const int RSLOW_FW = 17;                       // Change from center for Right servo to get ~12RPM forward direction
const int LFAST_BK = 45;                       // Change from center for Left  servo to get ~30RPM backward direction
const int RFAST_BK = 50;                       // Change from center for Right servo to get ~30RPM backward direction
const int LSLOW_BK = 18;                       // Change from center for Left  servo to get ~12RPM backward direction
const int RSLOW_BK = 20;                       // Change from center for Right servo to get ~12RPM backward direction

int algorithm;

//-////////////////////////////////////////////////////////////////////////
// Random Mouse Algorithm Constants for classifying distances from wall
//-////////////////////////////////////////////////////////////////////////
const float RAND_COL             = 5;
const float RAND_TOO_FAR         = 15;
const float RAND_NEAR_WALL       = 5.5;
const float RAND_GRID_D          = 35;
const float RAND_WALL_MIN_D      = 7.5;
const float RAND_WALL_MAX_D      = 9.5;

//-////////////////////////////////////////////////////////////////////////
// Random Mouse Algorithm Decision Variables
boolean gor      = false;
boolean gol      = false;
boolean gof      = true;
boolean prev_gor = false;
boolean prev_gol = false;
boolean prev_gof = false;
char    openings = 0;
boolean prevTurnSmooth = false;

// Functions
void RAND_getDistances(void);
void RAND_goForward(int cm);
void RAND_goForward(int cm, boolean det);
void stopServos(void);
void RAND_findWall(void);
void RAND_followWall(int n);
boolean RAND_edgeDetected(void);
int RAND_getRandom(int n);
boolean RAND_detectCollision(void);
void doWallFollowerAlgorithm(void);
void doPledgeAlgorithm(void);
void doRandomMouseAlgorithm(void);
void smoothturnCW(void);
void smoothturnCCW(void);
//-////////////////////////////////////////////////////////////////////////
// Wall Follower Algorithm Constants
//-////////////////////////////////////////////////////////////////////////
const float WALL_COL             = 2.5;
const float WALL_RWALL_MIN_D     = 4.8;
const float WALL_RWALL_MAX_D     = 6;
const float WALL_RWALL_TOO_CLOSE = 2.7;
const float WALL_TOO_FAR         = 25;
const float WALL_NEAR_WALL       = 5;

//-////////////////////////////////////////////////////////////////////////
// Wall Follower Algorithm Functions
void WALL_findWall(void);
void WALL_goForward(void);
void WALL_goBackward(void);
void WALL_goForwardSlow(void);
void WALL_goBackwardSLOW(void);
void WALL_followTurn(void);

//-////////////////////////////////////////////////////////////////////////
// Pledge Algorithm Constants
//-////////////////////////////////////////////////////////////////////////
const float PLEDGE_COL             = 2.4;
const float PLEDGE_RWALL_MIN_D     = 5.2;
const float PLEDGE_RWALL_MAX_D     = 6.4;
const float PLEDGE_RWALL_TOO_CLOSE = 2.7;
const float PLEDGE_TOO_FAR         = 25;
const float PLEDGE_NEAR_WALL       = 4;
int angleCount = 0;

//-////////////////////////////////////////////////////////////////////////
// Pledge Algorithm Functions
void PLEDGE_leaveWall(void);
void PLEDGE_followTurn(void);
void turnCCW(float angle);
void turnCW(float angle);

// Global Variables
unsigned long pingTimer[SONAR_NUM] = {0, 0, 0}; // Holds the times when the next ping should happen for each sensor.
float   cm[SONAR_NUM];                    // Where the ping distances are stored.
float   prev_cm[SONAR_NUM];               // Where the ping distances are stored.
uint8_t currentSensor = 0;                // Keeps track of which sensor is active.
Ashbot  bot;
NewPing sonar[SONAR_NUM] = {              // Sensor object array.
  NewPing(3, 2, MAX_DISTANCE),            // Right Sensor
  NewPing(5, 4, MAX_DISTANCE),            // Front Sensor
  NewPing(7, 6, MAX_DISTANCE)             // Left Sensor
};

boolean followingWall = false;


void setup() {
  randomSeed(analogRead(5));
  delay(3000);
//  Serial.begin(115200);
  bot.attachServos();
  if (analogRead(0) > 1000) {
    algorithm = 0;
  } else if (analogRead(1) > 1000) {
    algorithm = 1;
  } else if (analogRead(2) > 1000) {
    algorithm = 2;
  }    
}

void loop() { 
  
//}
//void dumdum(void) { 

  if (algorithm == 0) {
   doWallFollowerAlgorithm();  
 } else if (algorithm == 1) {
   doPledgeAlgorithm();
 } else if (algorithm == 2) {
   doRandomMouseAlgorithm();
 }
}


void doWallFollowerAlgorithm(void) {
  RAND_getDistances();

  if (cm[FS] < WALL_COL) {                          // Collision detected. TUrning CCW90
    turnCCW(90);
    return;
  }
  
  // Main statement
  if (cm[RS] < WALL_RWALL_MIN_D) {   
    if (cm[RS] < WALL_RWALL_TOO_CLOSE) {          // Too close! Need a fast turn (faster than else below) (this has not triggered front collision, 
                                             // and hence, the angle must be CCW
     setWheelSpeed(LSLOW_FW-8, RSLOW_FW+10);
    } else {                                // Normal adjustment to go farther from the wall (slow turn)
     setWheelSpeed(LFAST_FW-16, RFAST_FW+8);
    }

  } else if (cm[RS] > WALL_RWALL_MAX_D) {      
    if (cm[RS] > WALL_TOO_FAR && (cm[LS] <= WALL_NEAR_WALL || cm[FS] < WALL_NEAR_WALL)) {    // Closer to left wall or front wall or both
      stopServos();
      delay(20);
      turnCCW(90);
      turnCCW(30);
      stopServos();
      delay(20);
    } else if (cm[RS] > WALL_TOO_FAR) {                                          // Too far from right wall or at an edge
      WALL_followTurn();
    } else {
      setWheelSpeed(LFAST_FW+8, RFAST_FW-16);                               // Left slow turn
    }

  } else {
    setWheelSpeed(LFAST_FW, RFAST_FW);
  }  
}

void doPledgeAlgorithm(void) {
  RAND_getDistances();

  if (cm[FS] < PLEDGE_COL) {                            // Collision detected. TUrning CCW90
    turnCCW(90);
    return;
  }
  
  // Main statement
  if (cm[RS] < PLEDGE_RWALL_MIN_D) {   
    if (cm[RS] < PLEDGE_RWALL_TOO_CLOSE) {              // Too close! Need a fast turn (faster than else below) (this has not triggered front collision, 
                                                 // and hence, the angle must be CCW
     setWheelSpeed(LSLOW_FW-8, RSLOW_FW+10);     // 19-8=11, 17+10=27 => 27-11 = 16. %diff = 16*100/19 = 84%
    } else {                                     // Normal adjustment to go farther from the wall (slow turn)
     setWheelSpeed(LFAST_FW-16, RFAST_FW+8);     // 49-16=33, 48+8=56 => 56-33 = 23. %diff = 23*100/49 = 47%
    }
    followingWall = true;

  } else if (cm[RS] > PLEDGE_RWALL_MAX_D) {
    if (cm[RS] > PLEDGE_TOO_FAR && followingWall == false && (cm[LS] <= PLEDGE_NEAR_WALL || cm[FS] < PLEDGE_NEAR_WALL)) {    // Closer to left wall or front wall or both
      stopServos();
      delay(20);
      turnCCW(90);
      turnCCW(30);
      stopServos();
      delay(20);
    } else if (cm[RS] > PLEDGE_TOO_FAR) {            // Too far from right wall or at an edge
        if (angleCount == 0) {
          PLEDGE_leaveWall();
        } else {
          PLEDGE_followTurn();
        }
    } else {
        setWheelSpeed(LFAST_FW+8, RFAST_FW-16); // Left slow turn
    }
    followingWall = true;

  } else {
    setWheelSpeed(LFAST_FW, RFAST_FW);
  } 
}
  
  
void doRandomMouseAlgorithm(void) {
  RAND_getDistances();
  if (RAND_detectCollision()) return;

  if ((cm[RS] < RAND_TOO_FAR) && (cm[LS] < RAND_TOO_FAR)) {
    gor = false;
    gof = true;
    gol = false;
    RAND_followWall();   
  } else {                                 // Found a junction
    int d;
    if (prevTurnSmooth && (gol || gor)) {
            // If previous turn was smooth and now making a turn again, don't go forward
    } else {
      RAND_goForward(2);
    }
    // In case both right and left directions are open but walls have slight mis-alignment
    // or if robot is not exactly parallel to walls, we may only capture one opening.
    //Going forward and taking readings again will make sure we capture both openings
    RAND_getDistances();
    prev_gor = gor;
    prev_gof = gof;
    prev_gol = gol;
  
    gor = false;
    gof = false;
    gol = false;
    switch (openings) { // openings: bit2=l, bit1=f, bit0=r
    case 0:             // No sides open. Error! Should never happen!
      break;
    case 1:             // Right side open
      gor = true;
      break;
    case 2:             // front side open
      gof = true;
      break;
    case 4:             // Left side open
      gol = true;  
      break;
    case 3:             // Right + front open
      d = RAND_getRandom(2);  
      if (d == 0) {     // Going right
        gor = true;
      } else {          // Going forward
        gof = true;
      }
      break;
    case 6:             // Left + Front sides open
      d = RAND_getRandom(2);
      if (d == 0) {     // Going left
        gol = true;
      } else {          // Going forward
        gof = true;
      }
      break;
    case 5:             // Right + Left sides open
      d = RAND_getRandom(2);
      if (d == 0) {     // Going right
        gor = true;
      } else if (d == 1) {    // Going left
        gol = true;
      }
      break;
    case 7:                   // All three sides open
      d = RAND_getRandom(3);
      if (d == 0) {           // Going right
        gor = true;
      } else if (d == 1) {    // Going left
        gol = true;
      } else {                // Going forward
        gof = true;
      }
      break;
    default:                  // Should never come here
      break;
    }
  
    if (prev_gor == false && prev_gol == false) {             // were going forward before
      RAND_goForward(4);
    } else if (cm[FS] >= RAND_GRID_D && prevTurnSmooth == false) { // e.g., after RAND_findWall+PLEDGE_turnCW
      RAND_goForward(3);
    }
    if (gof == true) {
      RAND_goForward(52, true);                                   // Need to detect the edge also
    } else {
      if (cm[FS] < RAND_GRID_D) {                                  // Sharp ninety degree turn
        RAND_findWall();
        if (gor == true) {
          turnCW(90);
          prevTurnSmooth = false;
        } else if (gol == true) {      
          turnCCW(90);
          prevTurnSmooth = false;
        }
        RAND_goForward(5);
      } else {                                               // Smooth ninety degree turn
        if (gor == true) {
          smoothturnCW();
          prevTurnSmooth = true;
        } else if (gol == true) {
          smoothturnCCW();
          prevTurnSmooth = true;
        } 
      }
    }
      
    }
}

void RAND_findWall(void) {
  int i = 0;
  boolean done = false;

  setWheelSpeed(LFAST_FW, RFAST_FW);
  // Go straight for the wall
  do {
    i = i + 1;
    RAND_getDistances();
    RAND_followWall();
    if (cm[FS] <= RAND_NEAR_WALL) {
      done = true;
    }
  } 
  while (i < 500 && done == false);
  stopServos();
}

void RAND_getDistances(void) {
  float distance;
  //  long t = micros();
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    distance = sonar[i].ping();
    distance = distance/58.0;
    if (distance == 0) {                   // no ping or out of range
      distance = MAX_DISTANCE;
    }
    prev_cm[i] = cm[i];
    cm[i] = distance;
    delay(PING_INTERVAL);
  }
  openings = 0;
  if (cm[RS] > RAND_TOO_FAR) {
    openings = openings + 1;
  }
  if (cm[FS] > RAND_GRID_D) {
    openings = openings + 2;
  }
  if (cm[LS] > RAND_TOO_FAR) {
    openings = openings + 4;
  }
}


void stopServos(void) {
  bot.servoLeft.writeMicroseconds(LCENTER);
  bot.servoRight.writeMicroseconds(RCENTER);
}

void setWheelSpeed(int ls, int rs) {
  bot.servoLeft.writeMicroseconds(LCENTER + ls);
  bot.servoRight.writeMicroseconds(RCENTER - rs);
}

void setWheelSpeedBW(int ls, int rs) {
  bot.servoLeft.writeMicroseconds(LCENTER - ls);
  bot.servoRight.writeMicroseconds(RCENTER + rs);  
}

void RAND_goForward(int cm) {
  RAND_goForward(cm, false);
}

void RAND_goForward(int cm, boolean det) {
  unsigned long stime;
  unsigned long etime;
  boolean       edge;
  unsigned long mtime;
  stime = micros();
  mtime = (unsigned long)((double)1000000.0 * (((double)cm)-0.8))/10.15;
  setWheelSpeed(LFAST_FW, RFAST_FW);

  do {
    edge = RAND_edgeDetected();                     // Consumes 110 ms
    if (det && edge) {
      return;        
    }
    RAND_followWall();                              // Consumes   50 ms
    if (RAND_detectCollision()) return;  
    etime = micros() - stime;
  } while (etime < mtime);
}

// Inside a coridor (only front side open)
void RAND_followWall(void) {
  if (cm[RS] <= cm[LS]) {
    if (cm[RS] < RAND_WALL_MIN_D) {
      setWheelSpeed(LFAST_FW-12, RFAST_FW+8);  // Normal adjustment to go away from the right wall
    } else if (cm[RS] > RAND_WALL_MAX_D) {
      setWheelSpeed(LFAST_FW+8, RFAST_FW-12);  // Normal adjustment to go closer to the right wall
    }
  } else if (cm[LS] < RAND_TOO_FAR) {
    if (cm[LS] < RAND_WALL_MIN_D) {
      setWheelSpeed(LFAST_FW+8, RFAST_FW-12);  // Normal adjustment to go away from the left wall
    } else if (cm[LS] > RAND_WALL_MAX_D) {
      setWheelSpeed(LFAST_FW-12, RFAST_FW+8);  // Normal adjustment to go closer to the left wall
    }
  }
  delay(50);
  setWheelSpeed(LFAST_FW, RFAST_FW);
}

boolean RAND_edgeDetected(void) {
  boolean e;
  RAND_getDistances();
  if ((prev_cm[LS] < RAND_TOO_FAR) && ((cm[LS] - prev_cm[LS]) > (RAND_GRID_D - 10))) {         // Left edge detected
    e = true;
  } else if ((prev_cm[RS] < RAND_TOO_FAR) && ((cm[RS] - prev_cm[RS]) > (RAND_GRID_D - 25))) {  // Right edge detected
    e = true;
  } else {
    e = false;
  }  
  return e;
}

inline int RAND_getRandom(int n) {
  return random(n);
}

boolean RAND_detectCollision(void) {
  if (cm[FS] < RAND_COL) {
    gor = false;
    gof = true;
    gol = false;
    if (openings == 0) {
      turnCCW(180);
      return true;
    }
    int d = RAND_getRandom(2);
    if (d == 1) {
      turnCCW(90);  // Collision detected. Random -90 degree turn
      gol = true;
    } else {
      turnCW(90);   // Collision detected. Random +90 degree turn
      gor = true;
    }
    return true;
  } else {
    return false;
  }
}

void WALL_followTurn() {
  boolean done = false;

  WALL_goForward(5);                      // Go forward 5 cm
  setWheelSpeed(LFAST_FW-2, RSLOW_FW-1);
  delay(390);

  while (done == false) {
    RAND_getDistances();
    if (cm[RS] <= WALL_RWALL_MAX_D) {         
      done = true;
    }
  }
}


void WALL_findWall(void) {
  float a;
  int i = 0;
  boolean done = false;

  setWheelSpeed(LFAST_FW, RFAST_FW);                                       // Go straight for the wall

  do {
    i = i + 1;
    RAND_getDistances();
    if (cm[FS] <= WALL_NEAR_WALL || cm[LS] <= WALL_NEAR_WALL || cm[RS] <= WALL_NEAR_WALL) {
      done = true;
    }
  } while (i < 500 && done == false);

  // Re-orient the robot once closer to the wall so that right side faces the wall
  i = 0;
  done = false;
  stopServos();
  do {
    i = i + 1;
    RAND_getDistances();
    if (abs(cm[LS] - cm[RS]) < 6) {    // Front is pretty much perpendicular to the wall
      turnCCW(90);
      done = true;
    }
    else if (cm[RS] > cm[LS]) {        // Robot is at an angle. Right side away from wall
      a = 90 * (cm[RS] - cm[LS])/MAX_DISTANCE;
      if (a >= 90) {
        a = 70;
      }
      // Robot is at an angle. Right side away from wall (turning CCW90) and a proportional angle 'a'
      turnCCW(90);
      turnCCW(a);
      done = true;
    }
    else if (cm[LS] > cm[RS]) {        // Robot is at an angle. Right side closer to wall (done)
      done = true;
    }
  } while (i < 100 && done == false);
}

void WALL_goForward(int cm) {
  double d;
  d = (1000.0 * (((double)cm)-0.8))/10.15;

  setWheelSpeed(LFAST_FW, RFAST_FW);
  delay((int)d);
}  

void WALL_goBackward(int cm) {
  double d;
  d = (1000.0 * (((double)cm)-0.8))/10.15;

  setWheelSpeedBW(LFAST_BK, RFAST_BK);
  delay((int)d);
}  

void WALL_goForwardSlow(int cm) {
  double d;
  d = (1000.0 * (((double)cm)-0.8))/10.15;

  setWheelSpeed(LSLOW_FW, RSLOW_FW);
  delay((int)d);
}  

void WALL_goBackwardSlow(int cm) {
  double d;
  d = (1000.0 * (((double)cm)-0.8))/10.15;

  setWheelSpeedBW(LSLOW_BK, RSLOW_BK);
  delay((int)d);
}  

void PLEDGE_leaveWall(void) {
  float a;
  int i = 0;
  boolean done = false;
  followingWall = false;


  setWheelSpeed(LFAST_FW, RFAST_FW);          // Go straight for the wall

  do {
    i = i + 1;
    RAND_getDistances();
    if (cm[FS] <= PLEDGE_COL) {
      done = true;    
    }
  } while (i < 500 && done == false);
  turnCCW(90);
}

void PLEDGE_followTurn() {
  RAND_goForward(5);                       // Go 5 cm
  setWheelSpeed(LFAST_FW, RSLOW_FW);
  delay(2650);
  stopServos();
  angleCount--;
  if (angleCount == 0) {
    return;
  }

  RAND_getDistances(); 
  if (cm[RS] > PLEDGE_TOO_FAR) {             // One more CW90 turn required
    setWheelSpeed(LFAST_FW+2, RSLOW_FW);
    delay(2650);
    stopServos();
    angleCount--;
  }
}

void turnCCW(float angle = 90) {
  float d = 276.0 * angle/9.0;
  bot.servoLeft.writeMicroseconds(LCENTER - LSLOW_BK + 1);
  bot.servoRight.writeMicroseconds(RCENTER - RSLOW_FW - 2);
  delay((int) d);
  bot.servoLeft.writeMicroseconds(LCENTER);
  bot.servoRight.writeMicroseconds(RCENTER);
  if (angle == 90) {
    angleCount++;
  }
}

void turnCW(float angle = 90) {
  float d = 245.0 * angle/9.0;
  bot.servoLeft.writeMicroseconds(LCENTER + LSLOW_FW + 2);
  bot.servoRight.writeMicroseconds(RCENTER + RSLOW_BK);
  delay((int) d);
  WALL_goForward(1);
  bot.servoLeft.writeMicroseconds(LCENTER);
  bot.servoRight.writeMicroseconds(RCENTER);
  if (angle == 90) {
    angleCount--;
  }
}

// Smooth CCW turn
void smoothturnCCW(void) {
  setWheelSpeed(LSLOW_FW, RFAST_FW - 2);
  delay(2500);
  RAND_goForward(1);
  stopServos();
}

// Smooth CW turn
void smoothturnCW(void) {
  setWheelSpeed(LFAST_FW-2, RSLOW_FW);
  delay(3000);
  RAND_goForward(1);
  stopServos();
}

