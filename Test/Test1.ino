#include <Servo.h>

char rec;
boolean newData = false;

class Leg {
    private:
        Servo hip;
        Servo leg;
        Servo knee;
        
    public:
        Leg::Leg(int hipId, int legId, int kneeId) {
            hip.attach(hipId + 1);
            leg.attach(legId + 1);
            knee.attach(kneeId + 1);
        }

        void Leg::setHip(int ang) {hip.write(ang);}
        void Leg::setLeg(int ang) {leg.write(ang);}
        void Leg::setKnee(int ang) {knee.write(ang);}
};


Leg leg1;

void setup() {
    // Serial.begin(9600);
    // Serial.println("<Arduino is ready>");
    leg1 = leg(1,2,3);
}

void loop() {
    // recvOneChar();
    // showNewData();
    leg1.setHip(30);


}

void recvOneChar() {
    if (Serial.available() > 0) {
        rec = Serial.read();
        newData = true;
    }
}

void showNewData() {
    if (newData == true) {
        Serial.print("This just in ... ");
        Serial.println(rec);

        if (rec == 'a')
            leg1.setHip(150);
        if (rec == 's')
            // leg1.setLeg(0);
        if (rec == 'd')
            // leg1.setKnee(0);

        newData = false;
    }
}

// The write() method of servos do not set them to the desired angle in real life
// this method enables angle in code to actually correspond with real life angles
// real range is 0 to 270 degrees
int toCodeAngle(int rAng) {
    return rAng*0.8;
}





