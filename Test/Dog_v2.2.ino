#include <Servo.h>
#include <math.h>
#include <BasicLinearAlgebra.h>

// commands --------------------------------
String cmd = "";
String state = "";

// servo config constants -------------------------------------
Servo servos[4][3];
const int servo_pin[4][3] = {{2, 3, 4}, {5, 6, 7}, {8, 9, 10}, {11, 12, 13}};

// Offsets are added to the input angles of a servo before feeding into servo
const double offsets[4][3] = {
  {(125+2), 1.5, -5}, //{125+2, 1.5, -5}, 
  {170+2+3, 58, 20},
  {(125-1), 10, 15}, //{(125-1), 10, 50},
  {135-2, 18.5, 71}
};

// multipliers are multiplied with the input angle of a servo before feeding into servo
// Used to adjust small imperfections of servos
const double multipliers[4][3] = {
  {1, 1.15, 1.12},
  {1, 1, 1},
  {1, 1.1, 1.12},
  {1, 0.985, 1}
};




// Foot dimensions constants --------------------------------------------
// unit in cm

// Although the servos are said to be 180 degree, testing shows that their actual range is about 260 degrees
const double MAX_ANGLE = 260.0;

// lengths of the physical parts of each foot, used for IK
const double hipL = 6.75;
const double thighL = 12;//10.75;
const double shinL = 12;//10.75;

// width and length of the physical body
// more percisely the rectange formed by the hip joint the 4 foot
const double W = 8;
const double L = 23.5;

double footStartPos[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

double footEndPos[4][3] = {{L/2, -W/2 - hipL, 0}, {L/2, W/2 + hipL, 0}, {-L/2, -W/2 - hipL, 0}, {-L/2, W/2 + hipL, 0}};

// the vector from the starting coordinate to ending coordinate for every feet
double footVector[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// walking cycle keypoints
// unit in cm
double walkSteps[8][3];


// holds the angles that are fed into servos
// automatically updated by IK
double angMap[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// holds the increment of each axis of a foot that it should move every cycle
// before inverse kinematics
double speedMap[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// holds the position a foot now
// before inverse kinematics
// double posNow[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
double posNow[4][3] = {{L/2, -W/2 - hipL, 0}, {L/2, W/2 + hipL, 0}, {-L/2, -W/2 - hipL, 0}, {-L/2, W/2 + hipL, 0}};

// holds the desired position of a foot
// before inverse kinematics
double posExpect[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};



// IK constants ---------------------------------------
// unit in cm
const double X_OFFSET = 0;
const double Y_OFFSET = 6.75;
const double Z_OFFSET = 0.5;

// walking constants -------------------------------
// unit in cm
double STAND_Z = 13;
double DEFAULT_X = 0;
double DEFAULT_Y = 0;
double WALK_X = 2; // 2
double WALK_Y = 0;
double WALK_Z = 3;

double KEEP = 999;

bool isStanding = false;





// Setup and Loop ---------------------------------------------------------------------------
void setup() {
  delay(1000);
  Serial.begin(9600);
  Serial.println("<Arduino is ready>");

  updateFootStartPos(0,0,0,0,0,0);
  updateFootVector();
  sit();

  // updateAllFeet();

  delay(3000);
  
  attachServos();

  // updatewalkSteps();
  Serial.println("pos Expect:");
  for (int i=0; i<4; i++) {
    for (int j=0; j<3; j++) {
      Serial.print(posExpect[i][j]);
      Serial.print(", ");
    }
    Serial.println();
  }

  Serial.println("pos Now:");
  for (int i=0; i<4; i++) {
    for (int j=0; j<3; j++) {
      Serial.print(posNow[i][j]);
      Serial.print(", ");
    }
    Serial.println();
  }

  Serial.println("ang Map:");
  for (int i=0; i<4; i++) {
    for (int j=0; j<3; j++) {
      Serial.print(angMap[i][j]);
      Serial.print(", ");
    }
    Serial.println();
  }
  

  

}

void loop() {
  getCmd();

  if (cmd == "clear") {
    Serial.print("cmd cleared");
    cmd = "";
  }

  if (cmd == "stand") {
    stand();
    cmd = "";
  }

  if (cmd == "sit") {
    sit();
    cmd = "";
  }

  if (cmd == "ik") {

    updateFootStartPos(0,0,PI/18,0,0,STAND_Z);
    updateFootVector();

    rotate();
    cmd = "";

  }

  if (cmd == "walk") {
    if (!isStanding) {
      Serial.println("Not stood up");
      cmd = "";
      return;
    }
    // Setting up the walkPos array for walking
    // STAND_Z = 13;
    DEFAULT_X = 0;
    DEFAULT_Y = 0;
    WALK_X = 2;
    WALK_Y = 0;
    WALK_Z = 3;

    updateWalkSteps();
    
    walk(1);

    // Serial.print("WalkDonw");
  }




  if (cmd == "s") {
    resetToStand();
    cmd = "";
  }
}


// void stand() {
//   setFootPos(0, DEFAULT_X, -6.75, STAND_Z, 1000);
//   setFootPos(1, DEFAULT_X, 6.75, STAND_Z, 1000);
//   setFootPos(2, DEFAULT_X, -6.75, STAND_Z, 1000);
//   setFootPos(3, DEFAULT_X, 6.75, STAND_Z, 1000);
//   waitAllReach();
//   isStanding = true;
// }
void stand() {
  updateFootStartPos(0,0,0,0,0,STAND_Z);
  updateFootVector();

  for(int i=0; i<4;i++) {
    setFootPos(i, footVector[i][0], footVector[i][1], footVector[i][2], 1000);
  }
  waitAllReach();
  isStanding = true;
}

// void sit() {
//   setFootPos(0, 0, -6.75, 0, 1000);
//   setFootPos(1, 0, 6.75, 0, 1000);
//   setFootPos(2, 0, -6.75, 0, 1000);
//   setFootPos(3, 0, 6.75, 0, 1000);
//   waitAllReach();
//   isStanding = false;
// }

void sit() {
  updateFootStartPos(0,0,0,0,0,0);
  updateFootVector();

  for(int i=0; i<4;i++) {
    setFootPos(i, footVector[i][0], footVector[i][1], footVector[i][2], 1000);
  }
  waitAllReach();
  isStanding = false;
}

void rotate() {
  setFootPos(0, footVector[0][0], footVector[0][1], footVector[0][2], 500);
  setFootPos(1, footVector[1][0], footVector[1][1], footVector[1][2], 500);
  setFootPos(2, footVector[2][0], footVector[2][1], footVector[2][2], 500);
  setFootPos(3, footVector[3][0], footVector[3][1], footVector[3][2], 500);
  waitAllReach();
}


void walk(int steps) {
    for (int i=1; i<=8 * steps; i++) {
      setFootPos(0, footVector[0][0] + walkSteps[i%8][0], footVector[0][1] + walkSteps[i%8][1], footVector[0][2] + walkSteps[i%8][2], 100);
      setFootPos(3, footVector[3][0] + walkSteps[i%8][0], footVector[3][1] + walkSteps[i%8][1], footVector[3][2] + walkSteps[i%8][2], 100);

      setFootPos(1, footVector[1][0] + walkSteps[(i+4)%8][0], footVector[1][1] + walkSteps[(i+4)%8][1], footVector[1][2] + walkSteps[(i+4)%8][2], 100);
      setFootPos(2, footVector[2][0] + walkSteps[(i+4)%8][0], footVector[2][1] + walkSteps[(i+4)%8][1], footVector[2][2] + walkSteps[(i+4)%8][2], 100);
      waitAllReach();
    } 
}



void resetToStand() {
  setFootPos(1, DEFAULT_X, 6.75, STAND_Z-WALK_Z, 50);
  setFootPos(2, DEFAULT_X, -6.75, STAND_Z-WALK_Z, 50);
  waitAllReach();
  setFootPos(1, DEFAULT_X, 6.75, STAND_Z, 100);
  setFootPos(2, DEFAULT_X, -6.75, STAND_Z, 100);
  waitAllReach();
  setFootPos(0, DEFAULT_X, -6.75, STAND_Z-WALK_Z, 50);
  setFootPos(3, DEFAULT_X, 6.75, STAND_Z-WALK_Z, 50);
  waitAllReach();
  setFootPos(0, DEFAULT_X, -6.75, STAND_Z, 100);
  setFootPos(3, DEFAULT_X, 6.75, STAND_Z, 100);
  waitAllReach();
}



// set the desired position of a Foot, in given miliseconds
// Breaks the whole length into 20 equal length and moves one time each time called
// called in the WaitAllReach
void setFootPos(int footNum, double x, double y, double z, double time) {
  double length_x = 0, length_y = 0, length_z = 0;

  if (x != KEEP)
    length_x = x - posNow[footNum][0];
  if (y != KEEP)
    length_y = y - posNow[footNum][1];
  if (z != KEEP)
    length_z = z - posNow[footNum][2];

  speedMap[footNum][0] = length_x / (time/20);
  speedMap[footNum][1] = length_y / (time/20);
  speedMap[footNum][2] = length_z / (time/20);

  if (x != KEEP)
    posExpect[footNum][0] = x;
  if (y != KEEP)
    posExpect[footNum][1] = y;
  if (z != KEEP)
    posExpect[footNum][2] = z;

}



// update and turn all the feet to commanded position
void updateAllFeet() {
  for (int i=0; i<4; i++) {
    for (int j=0; j<3; j++) {
      if (fabs(posNow[i][j] - posExpect[i][j]) > fabs(speedMap[i][j])) {
        posNow[i][j] += speedMap[i][j];
      } else {
        posNow[i][j] = posExpect[i][j];
      }
    }
    IK(i, posNow[i][0], posNow[i][1], posNow[i][2], isReverse(i));

    // always inverse the Hip motor
    turnServo(servos[i][0], angMap[i][0] * multipliers[i][0] + offsets[i][0], isReverse(i));

    for (int j=1; j<3; j++) {
      turnServo(servos[i][j], angMap[i][j] * multipliers[i][j] + offsets[i][j], isReverse(i));
    }
  }
}

// check if a foot should be reversed
// just for convenience to write stuff in loops
bool isReverse(int footNum) {
  return footNum % 2 != 0;
}

// wait for all servos on a leg to reach desired position
void waitReach(int leg) {
  while (
    posNow[leg][0] != posExpect[leg][0] ||
    posNow[leg][1] != posExpect[leg][1] ||
    posNow[leg][2] != posExpect[leg][2]) {
    updateAllFeet();
    // UNDELAYED INFINITE WHILE LOOP IS BAD
    delay(20);
  }
}

void waitAllReach() {
  for (int i = 0; i < 4; i++)
    waitReach(i);
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


// Inverse Kinematics
// Offsets for adjusting the 0 position of the tip of the feet to match its real world default position
void IK(int footNum, double x, double y, double z, bool inverse) {
  x = -x;
  if (!inverse) y = -y;

  z = -z;
  // x += X_OFFSET;
  // y += Y_OFFSET;
  // z += Z_OFFSET;

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

// Attaching all servos to corresponding pin
// Not in another for loop for ease of commenting out some foot to test individual foot
void attachServos() {
  for (int i=0; i<3; i++) {
    servos[0][i].attach(servo_pin[0][i]);
    delay(100);
    servos[1][i].attach(servo_pin[1][i]);
    delay(100);
    servos[2][i].attach(servo_pin[2][i]);
    delay(100);
    servos[3][i].attach(servo_pin[3][i]);
    delay(100);
  }
}

// getting command from serial moniter
void getCmd() {
  // if (Serial.available() > 0 && (cmd == "" || cmd == "walk")) {
  if (Serial.available() > 0) {

    if (Serial.peek() != '\n') {
      cmd = Serial.readStringUntil('\n');

      Serial.print("This just in: ");
      Serial.println(cmd);

    } else {
      Serial.read();
    } 
  }
  Serial.flush();
}


void updateFootStartPos(double xRot, double yRot, double zRot, double xm, double ym, double zm) {
  zm += 0.5;
  BLA::Matrix<4, 4> Rx =
  {1, 0, 0, 0,
  0, cos(xRot), -sin(xRot), 0, 
  0, sin(xRot), cos(xRot), 0,
  0, 0, 0, 1};

  BLA::Matrix<4, 4> Ry =
  {cos(yRot), 0, -sin(yRot), 0,
  0, 1, 0, 0, 
  sin(yRot), 0, cos(yRot), 0,
  0, 0, 0, 1};

  BLA::Matrix<4, 4> Rz =
  {cos(zRot), -sin(zRot), 0, 0,
  sin(zRot), cos(zRot), 0, 0,
  0, 0, 1, 0, 
  0, 0, 0, 1};

  BLA::Matrix<4, 4> Rxyz = Rx * Ry * Rz;

  BLA::Matrix<4, 4> T =
  {0, 0, 0, xm,
  0, 0, 0, ym,
  0, 0, 0, zm, 
  0, 0, 0, 0};

  BLA::Matrix<4, 4> Tm = T + Rxyz;

  // BLA::Matrix<4, 4> t0 = 
  // {cos(PI/2), 0, -sin(PI/2), L/2,
  // 0, 1, 0, -W/2, 
  // sin(PI/2), 0, cos(PI/2), 0,
  // 0, 0, 0, 1};

  BLA::Matrix<4, 1> t0 = {L/2, -W/2, 0, 1};
  BLA::Matrix<4, 1> T0 = Tm * t0;

  BLA::Matrix<4, 1> t1 = {L/2, W/2, 0, 1};
  BLA::Matrix<4, 1> T1 = Tm * t1;

  BLA::Matrix<4, 1> t2 = {-L/2, -W/2, 0, 1};
  BLA::Matrix<4, 1> T2 = Tm * t2;

  BLA::Matrix<4, 1> t3 = {-L/2, W/2, 0, 1};
  BLA::Matrix<4, 1> T3 = Tm * t3;


  for (int i=0; i<3; i++) {
    footStartPos[0][i] = T0(0, i);
    footStartPos[1][i] = T1(0, i);
    footStartPos[2][i] = T2(0, i);
    footStartPos[3][i] = T3(0, i);
    // for (int j=0; j<1; j++) {
    //   Serial.print(T0(i, j));
    //   Serial.print(", ");
    // }
    // Serial.println();
  }


  Serial.println("Start pos:");
  for (int i=0; i<4; i++) {

    for (int j=0; j<3; j++) {
      Serial.print(footStartPos[i][j]);
      Serial.print(", ");
    }
    Serial.println();
  }

}


void updateFootVector() {

  for (int i=0; i<4; i++) {
    for (int j=0; j<3; j++) {
      footVector[i][j] = footEndPos[i][j] - footStartPos[i][j];
    }
  }

  Serial.println("Vectors:");
  for (int i=0; i<4; i++) {
    for (int j=0; j<3; j++) {
      Serial.print(footVector[i][j]);
      Serial.print(", ");
    }
    Serial.println();
  }
}

void updateWalkSteps() {
  walkSteps[0][0] = DEFAULT_X;
  walkSteps[0][1] = DEFAULT_Y;
  walkSteps[0][2] = 0;

  walkSteps[1][0] = DEFAULT_X - WALK_X/3;
  walkSteps[1][1] = DEFAULT_Y + WALK_Y/3;
  walkSteps[1][2] = 0;

  walkSteps[2][0] = DEFAULT_X - 2*WALK_X/3;
  walkSteps[2][1] = DEFAULT_Y + 2*WALK_Y/3;
  walkSteps[2][2] = 0;

  walkSteps[3][0] = DEFAULT_X - WALK_X;
  walkSteps[3][1] = DEFAULT_Y + WALK_Y;
  walkSteps[3][2] = 0;

  walkSteps[4][0] = DEFAULT_X + WALK_X;
  walkSteps[4][1] = DEFAULT_Y - WALK_Y/2;
  walkSteps[4][2] = WALK_Z;

  walkSteps[5][0] = DEFAULT_X + WALK_X;
  walkSteps[5][1] = DEFAULT_Y - WALK_Y;
  walkSteps[5][2] = 0;

  walkSteps[6][0] = DEFAULT_X + 2*WALK_X/3;
  walkSteps[6][1] = DEFAULT_Y - 2*WALK_Y/3;
  walkSteps[6][2] = 0;

  walkSteps[7][0] = DEFAULT_X + WALK_X/3;
  walkSteps[7][1] = DEFAULT_Y - WALK_Y/3;
  walkSteps[7][2] = 0;
}


