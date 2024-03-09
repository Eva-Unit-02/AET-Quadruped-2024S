#include <Servo.h>
#include <math.h>


double MAX_ANGLE = 260.0;
double LEG_LENGTH = 11; // in cm
double Z_OFFSET = 3;

String cmd = "";

Servo FR[3];
Servo FL[3];
Servo BR[3];
Servo BL[3];


struct foot {
  int id;
  Servo hip;
  Servo leg;
  Servo knee;
};

//leg 0 is when the long side of servo is 13 degrees tilted
double offsets[12] = {130,0,10,
                    135,20,-4,
                    125,13,17.5,
                    135,21,10};

double multipliers[12] = {1, 1.048, 0.995,
                    1, 1, 1.08,
                    1, 0.975, 1.05,
                    1, 0.965, 1};

double angMap[12] = {0,0,0,
                    0,0,0,
                    0,0,0,
                    0,0,0};

double posMap[12] = {0,0,0,
                    0,0,0,
                    0,0,0,
                    0,0,0};

double prevAng[12] = {0,0,0,
                    0,0,0,
                    0,0,0,
                    0,0,0};


// Servo s;

void setup() {
  Serial.begin(9600);
  Serial.println("<Arduino is ready>");

  delay(2000);
  UpdateAllFeet();

  for (int i=0; i<3; i++) {
    FR[i].attach(i+2);
    delay(500);
    FL[i].attach(i+5);
    delay(500);
    BR[i].attach(i+8);
    delay(500);
    BL[i].attach(i+11);
    delay(500);
    
  }
 
  delay(5000);
  
}

void loop() {
  getCmd();
  // if (angMap[1] < 60) {
  //   angMap[1] += 0.5;
  //   angMap[2] += 0.9;

  //   angMap[4] += 0.5;
  //   angMap[5] += 0.9;

  //   angMap[7] += 0.5;
  //   angMap[8] += 0.9;

  //   angMap[10] += 0.5;
  //   angMap[11] += 0.9;
  //   UpdateAllFeet();
  //   delay(20);
  // }

  // IK(BL, 5, 0, 10, true);

  if (cmd=="stand") {
    if (MoveAllZ(16, 10)) {
      cmd == "";
    }
  }

  if (cmd=="sit") {
    if (MoveAllZ(3, 10)) {
      cmd == "";
    }
  }

  

  IKUpdateAllFeet();
}

void getCmd() {
  if (Serial.available() > 0 && Serial.read() != '\n' && cmd == "") {
    cmd = Serial.readString();
    
    Serial.print("This just in ... ");
    Serial.println(cmd);
  }
}



bool MoveAllZ(double setpoint, int ms) {
  double amt = -0.1 * (posMap[2] - setpoint)/abs(posMap[2] - setpoint);
  // if (abs(posMap[2] - setpoint) >= 0.1) {
  if (posMap[2] != setpoint) {
    posMap[2] += amt;
    posMap[5] += amt;
    posMap[8] += amt;
    posMap[11] += amt;
    delay(ms);
    return false;
  }
  return true;
}

void StandUp() {
  MoveAllZ(17, 10);
}

void IKUpdateAllFeet() {
  IK(FR, posMap[0], posMap[1], posMap[2], false);
  IK(FL, posMap[3], posMap[4], posMap[5], true);
  IK(BR, posMap[6], posMap[7], posMap[8], false);
  IK(BL, posMap[9], posMap[10], posMap[11], true);
}

void UpdateAllFeet() {
  for (int i=0; i<3; i++) {
    TurnServo(FR[i], angMap[i] * multipliers[i] + offsets[i], false);
    TurnServo(FL[i], angMap[i+3] * multipliers[i+3] + offsets[i+3], true);
    TurnServo(BR[i], angMap[i+6] * multipliers[i+6] + offsets[i+6], false);
    TurnServo(BL[i], angMap[i+9] * multipliers[i+9] + offsets[i+9], true);
  }
}

// Converts from degree to Miliseconds for servos
// ang: setpoint in degrees
int AngToMS(double ang) {
  return round(ang / 180 * 2000) + 500;
}

// Converts from degree to Miliseconds that correpond to real life angle for servos
// Needs to further converted with offsets and multipliers for specific servos
// ang: setpoint in degrees
int AngToSer(double ang) {
  int ms = round(ang / MAX_ANGLE * 2000) + 500;
  if (ms < 500) ms = 500;
  if (ms > 2500) ms = 2500;
  return ms;
}

// Turns a servo to given position
// If servo is mounted in the opposite direction from desired, inverse the motor
// ang: setpoint in degrees
// inverse: if a servo needs to be inversed
void TurnServo(Servo s, double ang, boolean inverse) {
  if (inverse) {
    ang = MAX_ANGLE - ang;
  } 
  s.writeMicroseconds(AngToSer(ang));
}

// void TimedTurnServo(Servo s, double ang, boolean inverse, int ms) {
//   if (abs(angMap[id-1] - ang) >= 1) {
//     TurnServo(s, ang/100, inverse);
//   }
//   delay(ms);
// }

void IK(Servo foot[3], double x, double y, double z, bool inverse) {
  z += Z_OFFSET;
  double d = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
  if (d > LEG_LENGTH * 2) return;
  double theta = acos(d / (2 * LEG_LENGTH)) * 180 / M_PI;
  double a1 = 90 - atan2(z, y) * 180 / M_PI;
  double a2 = atan2(z, x) * 180 / M_PI - theta;
  double a3 = 180 - 2 * theta;
  
  TurnServo(foot[0], a1, inverse);
  TurnServo(foot[1], a2, inverse);
  TurnServo(foot[2], a3, inverse);

}


