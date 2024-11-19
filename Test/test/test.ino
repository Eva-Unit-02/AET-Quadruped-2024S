

// commands --------------------------------
String cmd = "";
String state = "";

// servo config constants -------------------------------------
Servo servos[4][3];
const int servo_pin[4][3] = {{2, 3, 4}, {5, 6, 7}, {8, 9, 10}, {11, 12, 13}};
const double tolerance = 0.001; 

// Offsets are added to the input angles of a servo
const double offsets[4][3] = {
  {125+2, 0+7, 10},
  {135+2, 59+7, 23},
  {125-1, 13+7, 30},
  {135-2, 21+7, 29}
};

// const double offsets[4][3] = {
//   {125+2, 0, 10},
//   {135+2, 59, 23},
//   {125-1, 13, 30},
//   {135-2, 21, 29}
// };

// multipliers are multiplied with the input angle of a servo
const double multipliers[4][3] = {
  {1, 1.2, 1.15},
  {1, 0.94, 1.05},
  {1, 0.955, 1.1},
  {1, 0.985, 0.95}
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
int step = 1;
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
double KEEP = 999;

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

  FlexiTimer2::set(20, updateAllFeet);
  // FlexiTimer2::set(200, printE);
  FlexiTimer2::start();

  delay(2000);
  
  attachServos();
  
  // delay(1000);
}

void loop() {
  getCmd();

  if (cmd == "clear") {
    Serial.print("cmd cleared");
    cmd = "";
  }

  if (cmd == "stand") {
    setFootPos(0, KEEP, KEEP, walkPosF2[step%8][2], 1000);
    setFootPos(1, KEEP, KEEP, STAND_Z_F, 1000);
    setFootPos(2, KEEP, KEEP, STAND_Z_B, 1000);
    setFootPos(3, KEEP, KEEP, STAND_Z_B, 1000);
    waitAllReach();
    cmd = "";
  }

  if (cmd == "sit") {
    setFootPos(0, KEEP, KEEP, 0, 1000);
    setFootPos(1, KEEP, KEEP, 0, 1000);
    setFootPos(2, KEEP, KEEP, 0, 1000);
    setFootPos(3, KEEP, KEEP, 0, 1000);
    waitAllReach();
    cmd = "";
  }

  step = 1;
  if (cmd == "walk") {
    // for (int i=0; i<100; i++) {
    //   setFootPos(0, walkPosF2[step%8][0], walkPosF2[step%8][1], walkPosF2[step%8][2], 100);
    //   // setFootPos(3, walkPosB2[step%8][0], walkPosB2[step%8][1], walkPosB2[step%8][2], 200);

    //   // setFootPos(1, walkPosF2[(step+4)%8][0], walkPosF2[(step+4)%8][1], walkPosF2[(step+4)%8][2], 200);
    //   // setFootPos(2, walkPosB2[(step+4)%8][0], walkPosB2[(step+4)%8][1], walkPosB2[(step+4)%8][2], 200);
    //   waitAllReach();
    //   step++;

    // }
    
    // for (int i=2; i<8; i++) {
    //   setFootPos(0, walkPosF2[i][0], walkPosF2[i][1], walkPosF2[i][2], 100);
    //   waitAllReach();
    //   // printE();
    // }

    setFootPos(0, walkPosF2[1][0], walkPosF2[1][1], walkPosF2[1][2], 100);
    waitAllReach();

    setFootPos(0, walkPosF2[2][0], walkPosF2[2][1], walkPosF2[2][2], 100);
    waitAllReach();

    setFootPos(0, walkPosF2[3][0], walkPosF2[3][1], walkPosF2[3][2], 100);
    waitAllReach();


  }
}

// Attaching all servos to corresponding pin
// Not in another for loop for ease of commenting out some foot to test individual foot
void attachServos() {
  for (int i=0; i<3; i++) {
    servos[0][i].attach(servo_pin[0][i]);
    delay(100);
    // servos[1][i].attach(servo_pin[1][i]);
    // delay(100);
    // servos[2][i].attach(servo_pin[2][i]);
    // delay(100);
    // servos[3][i].attach(servo_pin[3][i]);
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

// void IKUpdateAllFeet() {
//   for (int i=0; i<4; i++) {
//     IK(i, posMap[i][0], posMap[i][1], posMap[i][2], isReverse(i));
//   }
// }

int rad = 0;
void updateAllFeet() {
  for (int i=0; i<4; i++) {
    for (int j=0; j<3; j++) {
      if (fabs(posNow[i][j] - posExpect[i][j]) >= fabs(speedMap[i][j])) {
        posNow[i][j] += speedMap[i][j];
      } else {
        posNow[i][j] = posExpect[i][j];
      }
    }
    IK(i, posNow[i][0], posNow[i][1], posNow[i][2], isReverse(i));
    
    for (int j=0; j<3; j++) {
      turnServo(servos[i][j], angMap[i][j] * multipliers[i][j] + offsets[i][j], isReverse(i));
    }
  }
  if (rad == 20) {
    printE();
    rad = 0;
  }
  rad++;

}

// check if a foot should be reversed
bool isReverse(int footNum) {
  return footNum % 2 != 0;
}


void waitReach(int leg) {
  while (1)
    if (check(posNow[leg][0], posExpect[leg][0]))
      if (check(posNow[leg][1], posExpect[leg][1]))
        if (check(posNow[leg][2], posExpect[leg][2]))
          break;
}

void waitAllReach(void) {
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

void printE() {
  Serial.print("Now:");
  Serial.print(posNow[0][0]);
  Serial.print(" ");
  Serial.print(posNow[0][1]);
  Serial.print(" ");
  Serial.println(posNow[0][2]);

  Serial.print("Expect:");
  Serial.print(posExpect[0][0]);
  Serial.print(" ");
  Serial.print(posExpect[0][1]);
  Serial.print(" ");
  Serial.println(posExpect[0][2]);

}


