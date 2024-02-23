#include <Servo.h>

char rec;
boolean newData = false;

Leg leg1;

void setup() {
    Serial.begin(9600);
    Serial.println("<Arduino is ready>");
    leg1 = new Leg(1,2,3);
}

void loop() {
    recvOneChar();
    showNewData();
}

void recvOneChar() {
    if (Serial.available() > 0) {
        rec = Serial.read();
        newData = true;
    }
}

void showNewData() {
    if (newData == true && (rec == 'a' || rec == 's' || rec == "d")) {
        Serial.print("This just in ... ");
        Serial.println(rec);

        if (rec == 'a')
            leg1.setHip(0);
        if (rec == 's')
            leg1.setLeg(0);
        if (rec == 'd')
            leg1.setKnee(0);

        newData = false;
    }
}

// The write() method of servos do not set them to the desired angle in real life
// this method enables angle in code to actually correspond with real life angles
// real range is 0 to 270 degrees
int toCodeAngle(int rAng) {
    return rAng*0.8;
}


class Leg {
    private:
        Servo hip;
        Servo leg;
        Servo knee;
        
    public:
        Leg(int hipId, int legId, int kneeId) {
            hip.attach(hipId+1);
            leg.attach(legId+1);
            knee.attach(kneeId+1);
        }

        setHip(int ang) {hip.write(ang);}
        setLeg(int ang) {leg.write(ang);}
        setKnee(int ang) {knee.write(ang);}
}


