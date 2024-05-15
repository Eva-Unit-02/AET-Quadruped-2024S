#include <Servo.h>
#include <math.h>

// commands --------------------------------
String cmd = "";
String state = "";

// servo config constants -------------------------------------
Servo servos[4][3];
const int servo_pin[4][3] = {{2, 3, 4}, {5, 6, 7}, {8, 9, 10}, {11, 12, 13}};
const double tolerance = 0.001; 

// Offsets are added to the input angles of a servo
const double offsets[4][3] = {
  {125+2, 1.5, -5}, 
  {170+2+3, 58, 20},
  {125-1, 10, 7},
  {135-2, 18.5, 45}
};


double manualAngOffsets[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

double manualPosOffsets[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};


// multipliers are multiplied with the input angle of a servo
const double multipliers[4][3] = {
  {1, 1.15, 1.12},
  {1, 1, 1},
  {1, 1.1, 1.12},
  {1, 0.985, 1.03}
};

double angMap[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

double speedMap[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

double posNow[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

double posExpect[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

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
double STAND_Z = 12;
double DEFAULT_X = 0;
double DEFAULT_Y = 0;
double WALK_X = 2; // 2
double WALK_Y = 0;
double WALK_Z = 3;
int WALK_INTV = 10;
int WALK_DEL = 5;
double KEEP = 999;

bool isStanding = false;

// walking cycle keypoints -------------------------------------------
// unit in cm
double walkPos[8][3];

void updateWalkPos() {
  walkPos[0][0] = DEFAULT_X;
  walkPos[0][1] = DEFAULT_Y;
  walkPos[0][2] = STAND_Z;

  walkPos[1][0] = DEFAULT_X + WALK_X/3;
  walkPos[1][1] = DEFAULT_Y + WALK_Y/3;
  walkPos[1][2] = STAND_Z;

  walkPos[2][0] = DEFAULT_X + 2*WALK_X/3;
  walkPos[2][1] = DEFAULT_Y + 2*WALK_Y/3;
  walkPos[2][2] = STAND_Z;

  walkPos[3][0] = DEFAULT_X + WALK_X;
  walkPos[3][1] = DEFAULT_Y + WALK_Y;
  walkPos[3][2] = STAND_Z;

  walkPos[4][0] = DEFAULT_X - WALK_X;
  walkPos[4][1] = DEFAULT_Y - WALK_Y/2;
  walkPos[4][2] = STAND_Z - WALK_Z;

  walkPos[5][0] = DEFAULT_X - WALK_X;
  walkPos[5][1] = DEFAULT_Y - WALK_Y;
  walkPos[5][2] = STAND_Z;

  walkPos[6][0] = DEFAULT_X - 2*WALK_X/3;
  walkPos[6][1] = DEFAULT_Y - 2*WALK_Y/3;
  walkPos[6][2] = STAND_Z;

  walkPos[7][0] = DEFAULT_X - WALK_X/3;
  walkPos[7][1] = DEFAULT_Y - WALK_Y/3;
  walkPos[7][2] = STAND_Z;
}


// Setup and Loop ---------------------------------------------------------------------------
void setup() {
  delay(1000);
  Serial.begin(9600);
  Serial.println("<Arduino is ready>");

  

  updateAllFeet();

  delay(2000);
  
  attachServos();

  updateWalkPos();
  
  // delay(1000);
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
    Serial.println("wlak start____________________");
    for (int i=1; i<33; i++) { // __________________________________________________?????????__MAYBE 33????????????????????????????????????????????????
      setFootPos(0, walkPos[i%8][0], walkPos[i%8][1], walkPos[i%8][2], 100);
      setFootPos(3, walkPos[i%8][0], walkPos[i%8][1], walkPos[i%8][2], 100);

      setFootPos(1, walkPos[(i+4)%8][0], walkPos[(i+4)%8][1], walkPos[(i+4)%8][2], 100);
      setFootPos(2, walkPos[(i+4)%8][0], walkPos[(i+4)%8][1], walkPos[(i+4)%8][2], 100);
      waitAllReach();


      if (i%8 == 0) {
        updateWalkPos();
      }


    }
    Serial.println("walk done____________________");

    setFootPos(1, DEFAULT_X, 0, STAND_Z-3, 100);
    setFootPos(2, DEFAULT_X, 0, STAND_Z-3, 100);
    waitAllReach();
    setFootPos(1, DEFAULT_X, 0, STAND_Z, 200);
    setFootPos(2, DEFAULT_X, 0, STAND_Z, 200);
    waitAllReach();
    setFootPos(0, 0, 0, STAND_Z-3, 100);
    setFootPos(3, 0, 0, STAND_Z-3, 100);
    waitAllReach();
    setFootPos(0, 0, 0, STAND_Z, 150);
    setFootPos(3, 0, 0, STAND_Z, 150);
    waitAllReach();
    cmd = "stand";
  }

  if (cmd == "turn") {
    Serial.println("wlak start____________________");
    for (int i=1; i<32; i++) {
      setFootPos(0, walkPos[i%8][0], -walkPos[i%8][1], walkPos[i%8][2], 100);
      setFootPos(3, walkPos[i%8][0], -walkPos[i%8][1], walkPos[i%8][2], 100);

      setFootPos(1, walkPos[(i+4)%8][0], walkPos[(i+4)%8][1], walkPos[(i+4)%8][2], 100);
      setFootPos(2, walkPos[(i+4)%8][0], walkPos[(i+4)%8][1], walkPos[(i+4)%8][2], 100);
      waitAllReach();
    }
    Serial.println("wlak done____________________");
    setFootPos(1, 0, 0, STAND_Z-3, 100);
    setFootPos(2, 0, 0, STAND_Z-3, 100);
    waitAllReach();
    setFootPos(1, 0, 0, STAND_Z, 150);
    setFootPos(2, 0, 0, STAND_Z, 150);
    waitAllReach();
    cmd = "stand";
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


void setFootPos(int footNum, double x, double y, double z, double time) {
  float length_x = 0, length_y = 0, length_z = 0;

  if (x != KEEP)
    length_x = x - posNow[footNum][0];
  if (y != KEEP)
    length_y = y - posNow[footNum][1];
  if (z != KEEP)
    length_z = z - posNow[footNum][2];

  // float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

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
    
    for (int j=0; j<3; j++) {
      turnServo(servos[i][j], angMap[i][j] * multipliers[i][j] + offsets[i][j] + manualAngOffsets[i][j], isReverse(i));
    }
  }
}

// check if a foot should be reversed
bool isReverse(int footNum) {
  return footNum % 2 != 0;
}


void waitReach(int leg) {
  while (
    posNow[leg][0] != posExpect[leg][0] ||
    posNow[leg][1] != posExpect[leg][1] ||
    posNow[leg][2] != posExpect[leg][2]) {
    updateAllFeet();
    delay(20); // THIS IS FOR IT TO WORK
    // WHEN NOTHING IN THE WHILE LOOP, THE CODE WILL GET STUCK IN THE LOOP AND DO NOT GO FURTHER DUE TO UNKNOWN REASON
    // A PRINT ALSO WORKS
    // Serial.print("bruh");
    // UNDELAYED INFINITE WHILE LOOP IS BAD PRACTICE
  }
}

void waitAllReach() {
  for (int i = 0; i < 4; i++)
    waitReach(i);
}

bool check(double u1, double u2) {
  return (fabs(u1 - u2) < tolerance);
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

