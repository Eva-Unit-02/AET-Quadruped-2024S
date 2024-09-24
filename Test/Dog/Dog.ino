#include <Servo.h>
#include <math.h>

// commands --------------------------------
String cmd = "";
String state = "";

// servo config constants -------------------------------------
Servo servos[4][3];
const int servo_pin[4][3] = { { 2, 3, 4 }, { 5, 6, 7 }, { 8, 9, 10 }, { 11, 12, 13 } };
const double tolerance = 0.001;

// Offsets are added to the input angles of a servo before feeding into servo
const double offsets[4][3] = {
  { 125 + 2, 1.5, 0 - 15}, //4 was -20
  { 175 - 1.5, 58, 65 + 15 }, 
  { 124, 10 - 5 + 5, 40 + 10 + 5 -10},
  { 131, 25 - 5, 125 + 10 - 5 + 5}
};

// multipliers are multiplied with the input angle of a servo before feeding into servo
// Used to adjust small imperfections of servos
const double multipliers[4][3] = {
  { 1, 1.15, 1.15 }, // before: 1.12
  { 1, 1.08, 1 },
  { 1, 1.1, 1.08 },
  { 1, 0.992, 1 + 0.2 - 0.1}
};

// holds the angles that are fed into servos
// automatically updated by IK
double angMap[4][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };

// holds the increment of each axis of a foot that it should move every cycle
// before inverse kinematics
double speedMap[4][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };

// holds the position a foot now
// before inverse kinematics
double posNow[4][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };

// holds the desired position of a foot
// before inverse kinematics
double posExpect[4][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };

// Foot dimensions constants --------------------------------------------
// unit in cm

// Although the servos are said to be 180 degree, testing shows that their actual range is about 260 degrees
const double MAX_ANGLE = 260.0;

// lengths of the physical parts of each foot, used for IK
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
double STAND_Z = 13;
double DEFAULT_X = 0;
double DEFAULT_Y = 0;
double WALK_X = 0 + 3;  // 2
double WALK_Y = 0 + 2;

double WALK_Z = 3;

double KEEP = 999;

bool isStanding = false;

// walking cycle keypoints
// unit in cm
double walkPos[8][3];

void updateWalkPos() {
  walkPos[0][0] = DEFAULT_X;
  walkPos[0][1] = DEFAULT_Y;
  walkPos[0][2] = STAND_Z;

  walkPos[1][0] = DEFAULT_X + WALK_X / 3;
  walkPos[1][1] = DEFAULT_Y + WALK_Y / 3;
  walkPos[1][2] = STAND_Z;

  walkPos[2][0] = DEFAULT_X + 2 * WALK_X / 3;
  walkPos[2][1] = DEFAULT_Y + 2 * WALK_Y / 3;
  walkPos[2][2] = STAND_Z;

  walkPos[3][0] = DEFAULT_X + WALK_X;
  walkPos[3][1] = DEFAULT_Y + WALK_Y;
  walkPos[3][2] = STAND_Z;

  walkPos[4][0] = DEFAULT_X - WALK_X;
  walkPos[4][1] = DEFAULT_Y - WALK_Y / 2;
  walkPos[4][2] = STAND_Z - WALK_Z;

  walkPos[5][0] = DEFAULT_X - WALK_X;
  walkPos[5][1] = DEFAULT_Y - WALK_Y;
  walkPos[5][2] = STAND_Z;

  walkPos[6][0] = DEFAULT_X - 2 * WALK_X / 3;
  walkPos[6][1] = DEFAULT_Y - 2 * WALK_Y / 3;
  walkPos[6][2] = STAND_Z;

  walkPos[7][0] = DEFAULT_X - WALK_X / 3;
  walkPos[7][1] = DEFAULT_Y - WALK_Y / 3;
  walkPos[7][2] = STAND_Z;
}


//sidestep constants --------------------------------
double PULL_IN_X = 3;
double PULL_IN_Y = 0;
double PULL_IN_Z = 3;
double PUSH_OFF_Z = 2; 
double SSMULT = 1;

double sideStepPos[16][3];

int x = 1;
int y = 0;
void updateSideStepPos() {

  //stand straight
  sideStepPos[0][x] = DEFAULT_X;
  sideStepPos[0][y] = DEFAULT_Y;
  sideStepPos[0][2] = STAND_Z;

  //pull leg up and in a little
  sideStepPos[1][x] = DEFAULT_X - (SSMULT * PULL_IN_X/2);
  sideStepPos[1][y] = DEFAULT_Y;
  sideStepPos[1][2] = STAND_Z - (SSMULT * PULL_IN_Z/2);

  //pull leg up and in all the way
  sideStepPos[2][x] = DEFAULT_X - (SSMULT * PULL_IN_X);
  sideStepPos[2][y] = DEFAULT_Y;
  sideStepPos[2][2] = STAND_Z - ( SSMULT * PULL_IN_Z);

  //slightly push leg down while it's in
  sideStepPos[3][x] = DEFAULT_X - (SSMULT * PULL_IN_X);
  sideStepPos[3][y] = DEFAULT_Y;
  sideStepPos[3][2] = STAND_Z - (SSMULT * PULL_IN_Z/2);

  //completely push leg down while it's in
  sideStepPos[3][x] = DEFAULT_X - (SSMULT * PULL_IN_X);
  sideStepPos[3][y] = DEFAULT_Y;
  sideStepPos[3][2] = STAND_Z;

  //pick up leg but stay in inward position
  sideStepPos[4][x] = DEFAULT_X - (SSMULT * PULL_IN_X);
  sideStepPos[4][y] = DEFAULT_Y;
  sideStepPos[4][2] = STAND_Z + PUSH_OFF_Z;

  //pull leg up and out a little
  sideStepPos[5][x] = DEFAULT_X - (SSMULT * PULL_IN_X/2);
  sideStepPos[5][y] = DEFAULT_Y;
  sideStepPos[5][2] = STAND_Z - (SSMULT * PULL_IN_Z/2);

  //put leg back vertically and start lowering the leg
  sideStepPos[6][x] = DEFAULT_X;
  sideStepPos[6][y] = DEFAULT_Y;
  sideStepPos[6][2] = STAND_Z - (SSMULT * PULL_IN_Z/2);

  //lower leg and return to stand
  sideStepPos[7][x] = DEFAULT_X;
  sideStepPos[7][y] = DEFAULT_Y;
  sideStepPos[7][2] = STAND_Z;

  //stand straight again
  sideStepPos[8][x] = DEFAULT_X;
  sideStepPos[8][y] = DEFAULT_Y;
  sideStepPos[8][2] = STAND_Z;

  sideStepPos[9][x] = DEFAULT_X;
  sideStepPos[9][y] = DEFAULT_Y;
  sideStepPos[9][2] = STAND_Z;

  sideStepPos[10][x] = DEFAULT_X;
  sideStepPos[10][y] = DEFAULT_Y;
  sideStepPos[10][2] = STAND_Z;

  sideStepPos[11][x] = DEFAULT_X;
  sideStepPos[11][y] = DEFAULT_Y;
  sideStepPos[11][2] = STAND_Z;

  sideStepPos[12][x] = DEFAULT_X;
  sideStepPos[12][y] = DEFAULT_Y;
  sideStepPos[12][2] = STAND_Z;

  sideStepPos[13][x] = DEFAULT_X;
  sideStepPos[13][y] = DEFAULT_Y;
  sideStepPos[13][2] = STAND_Z;

  sideStepPos[14][x] = DEFAULT_X;
  sideStepPos[14][y] = DEFAULT_Y;
  sideStepPos[14][2] = STAND_Z;

  sideStepPos[15][x] = DEFAULT_X;
  sideStepPos[15][y] = DEFAULT_Y;
  sideStepPos[15][2] = STAND_Z;

}


double dancePos[4][3] = {
  { 0, 0, STAND_Z },
  { 0, 0, STAND_Z + 0.5 },
  { 0, 0, STAND_Z },
  { 0, 0, STAND_Z - 0.5 }
};

double dancePos2[4][3] = {
  { 0, 0, STAND_Z },
  { 0, 0.5, STAND_Z + 0.5 },
  { 0, 0, STAND_Z },
  { 0, -0.5, STAND_Z - 0.5 }
};



// Setup and Loop ---------------------------------------------------------------------------
void setup() {
  delay(1000);
  Serial.begin(9600);
  Serial.println("<Arduino is ready>");

  updateAllFeet();

  delay(3000);

  attachServos();

  updateWalkPos();
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
    WALK_X = 2 + 1;
    WALK_Y = 0;
    WALK_Z = 3 + 1;

    updateWalkPos();

    walk(1);

    // Serial.print("WalkDonw");
  }

  if (cmd == "sidestep") {
    if (!isStanding) {
      Serial.println("Not stood up");
      cmd = "";
      return;
    }
    // Setting up the walkPos array for walking
    // STAND_Z = 13;
    DEFAULT_X = 0;
    DEFAULT_Y = 0;
    PULL_IN_Y = 0;
    PULL_IN_X = 2 - 4;
    PULL_IN_Z = 3;
    PUSH_OFF_Z = 6;

    updateSideStepPos();

    sideStep(1);

    // Serial.print("WalkDonw");
  }

  if (cmd == "walk_back") {
    if (!isStanding) {
      Serial.println("Not stood up");
      cmd = "";
      return;
    }
    // STAND_Z = 13;
    DEFAULT_X = 0;
    DEFAULT_Y = 0;
    WALK_X = -1.5;
    WALK_Y = 0;
    WALK_Z = 3;

    updateWalkPos();

    walk(1);
  }

  if (cmd == "walk_fast") {
    if (!isStanding) {
      Serial.println("Not stood up");
      cmd = "";
      return;
    }
    // STAND_Z = 13;
    DEFAULT_X = 0;
    DEFAULT_Y = 0;
    WALK_X = 3.5;
    WALK_Y = 0;
    WALK_Z = 3;

    updateWalkPos();

    walk(1);
  }

  if (cmd == "turn_l") {
    if (!isStanding) {
      Serial.println("Not stood up");
      cmd = "";
      return;
    }
    // STAND_Z = 13;
    DEFAULT_X = 0;
    DEFAULT_Y = 0;
    WALK_X = 0;
    WALK_Y = 1.7;
    WALK_Z = 3;

    updateWalkPos();

    turn(1);
  }

  if (cmd == "turn_r") {
    if (!isStanding) {
      Serial.println("Not stood up");
      cmd = "";
      return;
    }
    // STAND_Z = 13;
    DEFAULT_X = 0;
    DEFAULT_Y = 0;
    WALK_X = 0;
    WALK_Y = -1.7;
    WALK_Z = 3;

    updateWalkPos();

    turn(1);
  }

  if (cmd == "walk_turn_left") {
    if (!isStanding) {
      Serial.println("Not stood up");
      cmd = "";
      return;
    }
    // STAND_Z = 13;
    DEFAULT_X = 0;
    DEFAULT_Y = 0;
    WALK_X = 1.7;
    WALK_Y = -1.7;
    WALK_Z = 3;

    updateWalkPos();

    turn(1);
  }

  if (cmd == "dance") {
    if (!isStanding) {
      Serial.println("Not stood up");
      cmd = "";
      return;
    }
    // STAND_Z = 13;
    DEFAULT_X = 0;
    DEFAULT_Y = 0;
    WALK_X = 0;
    WALK_Y = 0;
    WALK_Z = 3;

    updateWalkPos();

    dance(1);
  }

  if (cmd == "dance2") {
    if (!isStanding) {
      Serial.println("Not stood up");
      cmd = "";
      return;
    }
    // STAND_Z = 13;
    DEFAULT_X = 0;
    DEFAULT_Y = 0;
    WALK_X = 0;
    WALK_Y = 0;
    WALK_Z = 3;

    updateWalkPos();

    dance2(1);
  }


  if (cmd == "s") {
    resetToStand();
    cmd = "";
  }
}


void stand() {
  setFootPos(0, DEFAULT_X, 0, STAND_Z, 1000);
  setFootPos(1, DEFAULT_X, 0, STAND_Z, 1000);
  setFootPos(2, DEFAULT_X, 0, STAND_Z, 1000);
  setFootPos(3, DEFAULT_X, 0, STAND_Z, 1000);
  waitAllReach();
  isStanding = true;
}

void sit() {
  setFootPos(0, 0, 0, 0, 1000);
  setFootPos(1, 0, 0, 0, 1000);
  setFootPos(2, 0, 0, 0, 1000);
  setFootPos(3, 0, 0, 0, 1000);
  waitAllReach();
  isStanding = false;
}

void walk(int steps) {
  for (int i = 1; i <= 8 * steps; i++) {
    setFootPos(0, walkPos[i % 8][0], walkPos[i % 8][1], walkPos[i % 8][2], 100);
    setFootPos(3, walkPos[i % 8][0], walkPos[i % 8][1], walkPos[i % 8][2], 100);

    setFootPos(1, walkPos[(i + 4) % 8][0], walkPos[(i + 4) % 8][1], walkPos[(i + 4) % 8][2], 100);
    setFootPos(2, walkPos[(i + 4) % 8][0], walkPos[(i + 4) % 8][1], walkPos[(i + 4) % 8][2], 100);
    waitAllReach();
  }
}


int s = 16;
int stepOffset = 7; // try 8
int standnum = 12;
void sideStep(int steps) {
  for (int i = 0; i <= s * steps -1; i++) {
    switchSSMULT();
    updateSideStepPos();
    setFootPos(0, sideStepPos[i % s][0], sideStepPos[i % s][1], sideStepPos[i % s][2], 100);

    switchSSMULT();
    updateSideStepPos();
    setFootPos(3, sideStepPos[i % s][0], sideStepPos[i % s][1], sideStepPos[i % s][2], 100);

    // switchSSMULT();
    // updateSideStepPos();
    // setFootPos(1, sideStepPos[(i + stepOffset) % s][0], sideStepPos[(i + stepOffset) % s][1], sideStepPos[(i + stepOffset) % s][2], 100);
    setFootPos(1, sideStepPos[standnum][0], sideStepPos[standnum][1], sideStepPos[standnum][2], 100);

    // switchSSMULT();
    // updateSideStepPos();
    // setFootPos(2, sideStepPos[(i + stepOffset) % s][0], sideStepPos[(i + stepOffset) % s][1], sideStepPos[(i + stepOffset) % s][2], 100);
    setFootPos(2, sideStepPos[standnum][0], sideStepPos[standnum][1], sideStepPos[standnum][2], 100);

    waitAllReach();
  }
}

void switchSSMULT() {
  if (SSMULT == 1){
    SSMULT = -1;
  }
  else {
    SSMULT = 1;
  }
}

void turn(int steps) {
  for (int i = 1; i <= 8 * steps; i++) {
    setFootPos(0, walkPos[i % 8][0], -walkPos[i % 8][1], walkPos[i % 8][2], 90);
    setFootPos(3, walkPos[i % 8][0], -walkPos[i % 8][1], walkPos[i % 8][2], 90);

    setFootPos(1, walkPos[(i + 4) % 8][0], walkPos[(i + 4) % 8][1], walkPos[(i + 4) % 8][2], 90);
    setFootPos(2, walkPos[(i + 4) % 8][0], walkPos[(i + 4) % 8][1], walkPos[(i + 4) % 8][2], 90);
    waitAllReach();
  }
}

void dance(int steps) {
  for (int i = 1; i <= 4 * steps; i++) {
    setFootPos(0, dancePos[i % 4][0], dancePos[i % 4][1], dancePos[i % 4][2], 200);
    setFootPos(1, dancePos[i % 4][0], dancePos[i % 4][1], dancePos[i % 4][2], 200);

    setFootPos(2, dancePos[(i + 2) % 4][0], dancePos[(i + 2) % 4][1], dancePos[(i + 2) % 4][2], 200);
    setFootPos(3, dancePos[(i + 2) % 4][0], dancePos[(i + 2) % 4][1], dancePos[(i + 2) % 4][2], 200);
    waitAllReach();
  }
}

void dance2(int steps) {
  for (int i = 1; i <= 4 * steps; i++) {
    setFootPos(0, dancePos2[i % 4][0], dancePos2[i % 4][1], dancePos2[i % 4][2], 200);
    setFootPos(2, dancePos2[i % 4][0], dancePos2[i % 4][1], dancePos2[i % 4][2], 200);

    setFootPos(1, dancePos2[(i + 2) % 4][0], dancePos2[(i + 2) % 4][1], dancePos2[(i + 2) % 4][2], 200);
    setFootPos(3, dancePos2[(i + 2) % 4][0], dancePos2[(i + 2) % 4][1], dancePos2[(i + 2) % 4][2], 200);
    waitAllReach();
  }
}

void resetToStand() {
  setFootPos(1, DEFAULT_X, 0, STAND_Z - WALK_Z, 50);
  setFootPos(2, DEFAULT_X, 0, STAND_Z - WALK_Z, 50);
  waitAllReach();
  setFootPos(1, DEFAULT_X, 0, STAND_Z, 100);
  setFootPos(2, DEFAULT_X, 0, STAND_Z, 100);
  waitAllReach();
  setFootPos(0, DEFAULT_X, 0, STAND_Z - WALK_Z, 50);
  setFootPos(3, DEFAULT_X, 0, STAND_Z - WALK_Z, 50);
  waitAllReach();
  setFootPos(0, DEFAULT_X, 0, STAND_Z, 100);
  setFootPos(3, DEFAULT_X, 0, STAND_Z, 100);
  waitAllReach();
}





// set the desired position of a Foot, in given miliseconds
// Breaks the whole length into 20 equal length and moves one time each time called
// called in the WaitAllReach
void setFootPos(int footNum, double x, double y, double z, double time) {
  float length_x = 0, length_y = 0, length_z = 0;

  if (x != KEEP)
    length_x = x - posNow[footNum][0];
  if (y != KEEP)
    length_y = y - posNow[footNum][1];
  if (z != KEEP)
    length_z = z - posNow[footNum][2];

  speedMap[footNum][0] = length_x / (time / 20);
  speedMap[footNum][1] = length_y / (time / 20);
  speedMap[footNum][2] = length_z / (time / 20);

  if (x != KEEP)
    posExpect[footNum][0] = x;
  if (y != KEEP)
    posExpect[footNum][1] = y;
  if (z != KEEP)
    posExpect[footNum][2] = z;
}




// update and turn all the feet to commanded position
void updateAllFeet() {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      if (fabs(posNow[i][j] - posExpect[i][j]) > fabs(speedMap[i][j])) {
        posNow[i][j] += speedMap[i][j];
      } else {
        posNow[i][j] = posExpect[i][j];
      }
    }
    IK(i, posNow[i][0], posNow[i][1], posNow[i][2], isReverse(i));

    for (int j = 0; j < 3; j++) {
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
    posNow[leg][0] != posExpect[leg][0] || posNow[leg][1] != posExpect[leg][1] || posNow[leg][2] != posExpect[leg][2]) {
    updateAllFeet();
    // UNDELAYED INFINITE WHILE LOOP IS BAD
    delay(25);
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

// Attaching all servos to corresponding pin
// Not in another for loop for ease of commenting out some foot to test individual foot
void attachServos() {
  for (int i = 0; i < 3; i++) {
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
