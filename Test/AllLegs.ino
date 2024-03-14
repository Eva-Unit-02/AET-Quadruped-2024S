#include <Servo.h>
#include <math.h>


double MAX_ANGLE = 260.0;
double LEG_LENGTH = 10.5; // in cm


String cmd = "";

Servo FR[3]; //num 1
Servo FL[3]; //num 2
Servo BR[3]; //num 3
Servo BL[3]; //num 4

int step = 1;

//leg 0 is when the long side of servo is 13 degrees tilted
double offsets[12] = {130,0,10,
                    135,20,15,
                    125,13,17.5,
                    135,21,5};

double multipliers[12] = {1, 1.048, 0.995,
                    1, 1, 0.99,
                    1, 0.975, 1.05,
                    1, 0.965, 1.02};

double angMap[12] = {0,0,0,
                    0,0,0,
                    0,0,0,
                    0,0,0};

double posMap[12] = {0,0,0,
                    0,0,0,
                    0,0,0,
                    0,0,0};

// double stepPathMap[5][3] = { {},
//                               {},
//                               {},
//                               {},
//                               {}}


void setup() {
  Serial.begin(9600);
  Serial.println("<Arduino is ready>");

  delay(2000);
  UpdateAllFeet();

  for (int i=0; i<3; i++) {
    // FR[i].attach(i+2);
    delay(100);
    FL[i].attach(i+5);
    delay(100);
    // BR[i].attach(i+8);
    delay(100);
    // BL[i].attach(i+11);
    delay(100);
    
  }
 
  delay(2000);
  
}

void loop() {
  getCmd();

  if (cmd == "clear") {
    Serial.print("cmd cleared");
    cmd = "";
  }

  // if (cmd == "step") {
  //   if (step == 1) {
  //     posMap[3] = 2;
  //     posMap[5] = 10;
  //     delay(500);
  //     step = 2;
  //   } 
  //   else if (step == 2) {
  //     posMap[3] = 2;
  //     posMap[5] = 8;
  //     delay(500);
  //     step = 3;
  //   } else if (step == 3) {
  //     posMap[3] = -2;
  //     posMap[5] = 8;
  //     delay(500);
  //     step = 4;
  //   } else if (step == 4) {
  //     posMap[3] = -2;
  //     posMap[5] = 10;
  //     delay(500);
  //     step = 1;
  //   }    
  // }

  if (cmd == "step") {
    if (step == 1) {
      posMap[3] = 2;
      posMap[5] = 10;
      delay(500);
      step = 2;
    } 
    else if (step == 2) {
      posMap[3] = 2;
      posMap[5] = 8;
      delay(500);
      step = 3;
    } else if (step == 3) {
      posMap[3] = -2;
      posMap[5] = 8;
      delay(500);
      step = 4;
    } else if (step == 4) {
      posMap[3] = -2;
      posMap[5] = 10;
      delay(500);
      step = 1;
    }    
  }

  // if (cmd == "forward") {
  //   if (step == 1 && MoveLeg(2, 0, 10, 0.1, 0, 0, 2, 8)) {
  //     step = 2;
  //   } else if (step == 2 && MoveLeg(2, 0, 8, 0, 0, -0.5, 2, 8)) {
  //     step = 3;
  //   } else if (step == 3 && MoveLeg(-2, 0, 8, -0.5, 0, 0, 2, 8)) {
  //     step = 4;
  //   } else if (step == 4 && MoveLeg(-2, 0, 10, 0, 0, 0.5, 2, 8)) {
  //     step = 1;
  //   }    
  // }

  
  

  // if (cmd == "stand") {
  //   posMap[2] += 0.5;
  //   posMap[5] += 0.5;
  //   posMap[8] += 0.5;
  //   posMap[11] += 0.5;
  //   delay(5);
  //   if (posMap[2] == 10) {
  //     Serial.println("stand done");
  //     cmd = "";
  //   }
  // }

  // if (cmd == "stand") {
  //   if (MoveLeg(0, 0, 10, 0, 0, 0.5, 2, 10)) {
  //     cmd = "";
  //   }
  // }

  // if (cmd == "sit") {
  //   if (MoveLeg(0, 0, 0, 0, 0, -0.5, 2, 10)) {
  //     cmd = "";
  //   }
  // }

  // if (cmd == "sit") {
  //   posMap[2] -= 0.5;
  //   posMap[5] -= 0.5;
  //   posMap[8] -= 0.5;
  //   posMap[11] -= 0.5;
  //   delay(5);
  //   if (posMap[2] == 0) {
  //     Serial.println("sit done");
  //     cmd = "";
  //   }
  // }


  
  IKUpdateAllFeet();
  UpdateAllFeet();
}



void getCmd() {
  if (Serial.available() > 0 && (cmd == "" || cmd == "forward")) {
    if (Serial.peek() != '\n') {
      cmd = Serial.readStringUntil('\n');

      Serial.print("This just in ... ");
      Serial.println(cmd);

    } else {
      Serial.read();
    } 
  }
}


double getFootX(int footNum) {
  return posMap[(footNum-1)*3];
}

double getFootY(int footNum) {
  return posMap[(footNum-1)*3+1];
}

double getFootZ(int footNum) {
  return posMap[(footNum-1)*3+2];
}

double tolerance = 0.00001; 
bool checkFootPos(int footNum, double x, double y, double z) {
  return (checkFootX(footNum, x) &&
      checkFootY(footNum, y) &&
      checkFootZ(footNum, z));
}

bool checkFootX(int footNum, double setpoint) {
  return (fabs(setpoint - posMap[(footNum - 1) * 3]) < tolerance);
}

bool checkFootY(int footNum, double setpoint) {
  return (fabs(setpoint - posMap[(footNum - 1) * 3 + 1]) < tolerance);
}

bool checkFootZ(int footNum, double setpoint) {
  return (fabs(setpoint - posMap[(footNum - 1) * 3 + 2]) < tolerance);
}

// interval should be numbers devisable by 2 or 5
void SetFootX(int footNum, double x0, double xf, int interval, int del) {
  int index = (footNum-1)*3;
  double step = (xf-x0) / interval;
  posMap[index] += step;
  delay(del);
}

void SetFootY(int footNum, double y0, double yf, int interval, int del) {
  int index = (footNum-1)*3+1;
  double step = (yf-yf) / interval;
  posMap[index] += step;
  delay(del);
}

void SetFootZ(int footNum, double z0, double zf, int interval, int del) {
  int index = (footNum-1)*3+2;
  double step = (zf-z0) / interval;
  posMap[index] += step;
  delay(del);
}

void IKUpdateAllFeet() {
  IK(1, posMap[0], posMap[1], posMap[2], false);
  IK(2, posMap[3], posMap[4], posMap[5], true);
  IK(3, posMap[6], posMap[7], posMap[8], false);
  IK(4, posMap[9], posMap[10], posMap[11], true);
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

double X_OFFSET = 0;
double Y_OFFSET = 6.75;
double Z_OFFSET = 0.5;
double hipL = 6.75;
double thighL = 10.75;
double shinL = 10.75;

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

  angMap[(footNum-1)*3] = a1;
  angMap[(footNum-1)*3+1] = a2;
  angMap[(footNum-1)*3+2] = a3;
}

