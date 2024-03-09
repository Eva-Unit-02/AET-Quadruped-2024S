#include <Servo.h>
#include <math.h>


double MAX_ANGLE = 260.0;

Servo FR[3]; //max 257
Servo FL[3]; //max 253
Servo BR[3]; //max
Servo BL[3]; //max


struct Foot {
  int hipPin;
  int legPin;
  int kneePin;

  Servo hip;
  Servo leg;
  Servo knee;

  Foot() {}

  Foot(int hipP, int legP, int kneeP) : hipPin(hipP+1), legPin(legP+1), kneePin(kneeP+1) {
    hip.write(90);
    leg.write(0);
    knee.write(0);


    hip.attach(hipPin);
    leg.attach(legPin);
    knee.attach(kneePin);
  }

};

Foot FR;

void setup() {
    FR = Foot(1,2,3);
}

void loop() {

}

// double angMap[12] = {130,0,10,
//                     130,0,10,
//                     130,0,10,
//                     130,0,10};


// // Servo s;

// void setup() {
//   TurnServo(FR[0], 130, false);
//   FR[0].attach(2);
//   delay(500);

//   TurnServo(FR[1], 0, false);
//   FR[1].attach(3);
//   delay(500);
  
//   TurnServo(FR[2], 8.5, false);
//   FR[2].attach(4);
//   delay(500);


//   TurnServo(BR[0], 130, false);
//   BR[0].attach(8);
//   delay(500);

//   TurnServo(BR[1], 0, false);
//   BR[1].attach(9);
//   delay(500);
  
//   TurnServo(BR[2], 8, false);
//   BR[2].attach(10);
//   delay(500);

//   TurnServo(BL[0], 130, true);
//   BL[0].attach(11);
//   delay(500);

//   TurnServo(BL[1], 0, true);
//   BL[1].attach(12);
//   delay(500);
  
//   TurnServo(BL[2], 10, true);
//   BL[2].attach(13);
//   delay(500);




//   // delay(2000);
//   // FR[1].writeMicroseconds(AngToSer(90));

//   // BR[1].writeMicroseconds(AngToSer(90));

//   // TurnServo(BL[1], 90, true);

//   // delay(2000);

//   // TurnServo(FR[2], 90, false);

//   // TurnServo(BR[2], 90, false);

//   // TurnServo(BL[2], 90, true);

//   delay(2000);
  
// }

// void loop() {
//   if (angMap[1] < 90) {
//     angMap[1] += 0.5;
//     angMap[2] += 0.5;
//     UpdateFoot();
//     delay(10);
//   }

// }

// void UpdateFoot() {
  
//   // for (int i=0; i<2; i++) {
//   //   for (int j=0; j<3; j++) {
//   //     TurnServo(FEET[i][j], angMap[(i+1)*(j+1)-1], false);
//   //   }
//   // }
//   for (int i=1; i<3; i++) {
//     TurnServo(FR[i], angMap[i], false);
//   }

// }

// int AngToMS(double ang) {
//   return round(ang / 180 * 2000) + 500;
// }

// // Max Input
// int AngToSer(double ang) {
//   if (ang < 0) ang = 0;
//   if (ang > MAX_ANGLE) ang = MAX_ANGLE;
//   return round(ang / MAX_ANGLE * 2000) + 500;

// }

// // left: if a servo is on the left side of the dog
// void TurnServo(Servo s, double ang, boolean left) {
//   if (left) {
//     ang = MAX_ANGLE - ang; // -------------------------------------------------------------------originally 180
//   } 
//   s.writeMicroseconds(AngToSer(ang));
// }



