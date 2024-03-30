#include <Servo.h>
#include <math.h>
#include <FlexiTimer2.h>

// commands --------------------------------
String cmd = "";
String state = "";

// servo config constants -------------------------------------
Servo servos[4][3];
const int servo_pin[4][3] = {{2, 3, 4}, {5, 6, 7}, {8, 9, 10}, {11, 12, 13}};
const double tolerance = 0.00001; 

// Offsets are added to the input angles of a servo
const double offsets[4][3] = {
  {125+2, 0+7, 10},
  {135+2, 59+7, 23},
  {125-1, 13+7, 27},
  {135-2, 21+7, 25}
};

// multipliers are multiplied with the input angle of a servo
const double multipliers[4][3] = {
  {1, 1.2, 1.15},
  {1, 0.94, 1.05},
  {1, 0.955, 1.1},
  {1, 0.985, 1}
};

double angMap[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

double posMap[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Foot dimensions constants --------------------------------------------
// unit in cm
const double MAX_ANGLE = 260.0;

const double hipL = 6.75;
const double thighL = 10.75;
const double shinL = 10.75;

// IK constants ---------------------------------------
// unit in cm
const double X_OFFSET = 0;
const double Y_OFFSET = 6.75;
const double Z_OFFSET = 0.5;

// walking constants -------------------------------
// unit in cm
int step = 0;
double STAND_Z_F = 11.5;
double STAND_Z_B = 16;
double DEFAULT_X = 0;
double DEFAULT_Y = 0;
double WALK_X = 2; // 2.5
double WALK_Y = 0;
double WALK_Z = 3;
int WALK_INTV = 10;
int counter = 0;
int WALK_DEL = 5;

// walking cycle keypoints -------------------------------------------
// unit in cm
double walkPosF2[8][3] = {
    {DEFAULT_X, DEFAULT_Y, STAND_Z_F}, // Changed index 0 and index 1 values
    {DEFAULT_X + WALK_X/3, DEFAULT_Y + WALK_Y/3, STAND_Z_F}, // Changed index 0 and index 1 values
    {DEFAULT_X + 2*WALK_X/3, DEFAULT_Y + 2*WALK_Y/3, STAND_Z_F}, // Changed index 0 and index 1 values
    {DEFAULT_X + WALK_X, DEFAULT_Y + WALK_Y, STAND_Z_F}, // Changed index 0 and index 1 values
    {DEFAULT_X - WALK_X, DEFAULT_Y - WALK_Y/2, STAND_Z_F-WALK_Z}, // Changed index 0 and index 1 values
    {DEFAULT_X - WALK_X, DEFAULT_Y - WALK_Y, STAND_Z_F}, // Changed index 0 and index 1 values
    {DEFAULT_X - 2*WALK_X/3, DEFAULT_Y - 2*WALK_Y/3, STAND_Z_F}, // Changed index 0 and index 1 values
    {DEFAULT_X - WALK_X/3, DEFAULT_Y - WALK_Y/3, STAND_Z_F}, // Changed index 0 and index 1 values
};

double walkPosB2[8][3] = {
    {DEFAULT_X, DEFAULT_Y, STAND_Z_B}, // Changed index 0 and index 1 values
    {DEFAULT_X + WALK_X/3, DEFAULT_Y + WALK_Y/3, STAND_Z_B}, // Changed index 0 and index 1 values
    {DEFAULT_X + 2*WALK_X/3, DEFAULT_Y + 2*WALK_Y/3, STAND_Z_B}, // Changed index 0 and index 1 values
    {DEFAULT_X + WALK_X, DEFAULT_Y + WALK_Y, STAND_Z_B}, // Changed index 0 and index 1 values
    {DEFAULT_X - WALK_X, DEFAULT_Y - WALK_Y/2, STAND_Z_B-WALK_Z}, // Changed index 0 and index 1 values
    {DEFAULT_X - WALK_X, DEFAULT_Y - WALK_Y, STAND_Z_B}, // Changed index 0 and index 1 values
    {DEFAULT_X - 2*WALK_X/3, DEFAULT_Y - 2*WALK_Y/3, STAND_Z_B}, // Changed index 0 and index 1 values
    {DEFAULT_X - WALK_X/3, DEFAULT_Y - WALK_Y/3, STAND_Z_B}, // Changed index 0 and index 1 values
};


// Setup and Loop ---------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  Serial.println("<Arduino is ready>");

  // FlexiTimer2::set(10, updateAllFeet);
  // FlexiTimer2::start();

  delay(2000);
  updateAllFeet();

  attachServos(); 
  
  delay(1000);
}

void loop() {
  getCmd();

  if (cmd == "clear") {
    Serial.print("cmd cleared");
    cmd = "";
  }

  if (cmd == "walk") {
    if (step == 0) {
      setFoot(2, walkPosF2[0], walkPosF2[4], WALK_INTV);
      setFoot(3, walkPosB2[0], walkPosB2[4], WALK_INTV);
      delay(WALK_DEL);
      counter++;
      if(counter == WALK_INTV) {
        step = 1;
        counter = 0;
      }
    }
    else {
      setFoot(1, walkPosF2[step - 1], walkPosF2[step%8], WALK_INTV);
      setFoot(4, walkPosB2[step - 1], walkPosB2[step%8], WALK_INTV);

      setFoot(2, walkPosF2[(step+3)%8], walkPosF2[(step+4)%8], WALK_INTV);
      setFoot(3, walkPosB2[(step+3)%8], walkPosB2[(step+4)%8], WALK_INTV);

      delay(WALK_DEL);

      counter++;
      if(counter == WALK_INTV) {
        step %= 8;
        step++;
        counter = 0;
      }
    }
  }
  

  if (cmd == "stand") {
    setFootZ(1, 0, STAND_Z_F, 40);
    setFootZ(2, 0, STAND_Z_F, 40);
    setFootZ(3, 0, STAND_Z_B, 40);
    setFootZ(4, 0, STAND_Z_B, 40);
    delay(5);
    if (checkFootZ(2, STAND_Z_F)) {
      Serial.println("stand done");
      cmd = "";
    }
  }

  IKUpdateAllFeet();
  updateAllFeet();
}

// Attaching all servos to corresponding pin
// Not in another for loop for ease of commenting out some foot to test individual foot
void attachServos() {
  for (int i=0; i<3; i++) {
    servos[0][i].attach(servo_pin[0][i]);
    delay(100);
    servos[1][i].attach(servo_pin[0][i]);
    delay(100);
    servos[2][i].attach(servo_pin[0][i]);
    delay(100);
    servos[3][i].attach(servo_pin[0][i]);
    delay(100);
  }
}



// Foot commands -----------------------------------------------
double getFootX(int footNum) {
  return posMap[footNum][0];
}

double getFootY(int footNum) {
  return posMap[footNum][1];
}

double getFootZ(int footNum) {
  return posMap[footNum][2];
}



bool checkFootPos(int footNum, double x, double y, double z) {
  return (checkFootX(footNum, x) &&
      checkFootY(footNum, y) &&
      checkFootZ(footNum, z));
}

bool checkFootX(int footNum, double setpoint) {
  return (fabs(setpoint - getFootX(footNum)) < tolerance);
}

bool checkFootY(int footNum, double setpoint) {
  return (fabs(setpoint - getFootY(footNum)) < tolerance);
}

bool checkFootZ(int footNum, double setpoint) {
  return (fabs(setpoint - getFootZ(footNum)) < tolerance);
}

// set a foot
void setFoot(int footNum, double arr0[3], double arrf[3], int interval) {
  if (arr0[0] != arrf[0]) {
    setFootX(footNum, arr0[0], arrf[0], interval);
  }
  if (arr0[1] != arrf[1]) {
    setFootY(footNum, arr0[1], arrf[1], interval);
  }
  if (arr0[2] != arrf[2]) {
    setFootZ(footNum, arr0[2], arrf[2], interval);
  }
  
}

// interval should be numbers devisable by 2 or 5
void setFootX(int footNum, double x0, double xf, int interval) {
  double step = (xf-x0) / interval;
  posMap[footNum][0] += step;
}

void setFootY(int footNum, double y0, double yf, int interval) {
  double step = (yf-y0) / interval;
  posMap[footNum][1] += step;
}

void setFootZ(int footNum, double z0, double zf, int interval) {
  double step = (zf-z0) / interval;
  posMap[footNum][2] += step;
}


// Servo functions ---------------------------------------------------------------


// Converts from degree to Miliseconds for servos
// ang: setpoint in degrees
int angToMS(double ang) {
  return round(ang / 180 * 2000) + 500;
}

// Converts from degree to Miliseconds that correpond to real life angle for servos
// Needs to further converted with offsets and multipliers for specific servos
// ang: setpoint in degrees
int angToSer(double ang) {
  int ms = round(ang / MAX_ANGLE * 2000) + 500;
  if (ms < 500) ms = 500;
  if (ms > 2500) ms = 2500;
  return ms;
}

// Turns a servo to given position
// If servo is mounted in the opposite direction from desired, inverse the motor
// ang: setpoint in degrees
// inverse: if a servo needs to be inversed
void turnServo(Servo s, double ang, boolean inverse) {
  if (inverse) {
    ang = MAX_ANGLE - ang;
  } 
  s.writeMicroseconds(angToSer(ang));
}


// Inverse Kinematics functions -----------------------------------------
// Offsets for adjusting the 0 position of the tip of the feet to match its real world default position
void IK(int footNum, double x, double y, double z, bool inverse) {
  x += X_OFFSET;
  y += Y_OFFSET;
  z += Z_OFFSET;

  double D = sqrt(pow(z, 2) + pow(y, 2) - pow(hipL, 2));
  double G = sqrt(pow(D, 2) + pow(x, 2));
  double hyp = pow(thighL, 2) + pow(shinL, 2);

  double a1 = atan2(y, z) + atan2(D, hipL);
  double a3 = acos((pow(G, 2) - hyp) / (-2 * thighL * shinL));
  double a2 = atan2(x, D) + asin(shinL * sin(a3) / G);

  a3 = a3 * 180 / M_PI;
  a1 = 90 - a1 * 180 / M_PI;
  a2 = 90 - a2 * 180 / M_PI;

  angMap[footNum][0] = a1;
  angMap[footNum][1] = a2;
  angMap[footNum][2] = a3;
}

void IKUpdateAllFeet() {
  for (int i=0; i<4; i++) {
    IK(0, posMap[i][0], posMap[i][1], posMap[i][2], isReverse(i));
  }
}

void updateAllFeet() {
  for (int i=0; i<4; i++) {
    IK(0, posMap[i][0], posMap[i][1], posMap[i][2], isReverse(i));
    for (int j=0; j<3; i++) {
      turnServo(servos[i][j], angMap[i][j] * multipliers[i][j] + offsets[i][j], isReverse(i));
    }
  }
}

// check if a foot should be reversed
bool isReverse(int footNum) {
  return footNum % 2 != 0;
}

// void wait_reach(int leg)
// {
//   while (1)
//     if (site_now[leg][0] == site_expect[leg][0])
//       if (site_now[leg][1] == site_expect[leg][1])
//         if (site_now[leg][2] == site_expect[leg][2])
//           break;
// }


// getting command from serial moniter
void getCmd() {
  if (Serial.available() > 0 && (cmd == "" || cmd == "walk")) {
    if (Serial.peek() != '\n') {
      cmd = Serial.readStringUntil('\n');

      Serial.print("This just in: ");
      Serial.println(cmd);

    } else {
      Serial.read();
    } 
  }
}

