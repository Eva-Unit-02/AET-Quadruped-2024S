#include <Servo.h>


class Leg {
  private:
    String name;
    Servo hip;
    Servo leg;
    Servo knee;
      
  public:
    Leg(int hipId, int legId, int kneeId);

    Leg();

    void setHip(int ang) {
      hip.write(ang);
    }
    void setLeg(int ang) {
      leg.write(ang);
    }
    void setKnee(int ang) {
      knee.write(ang);
    }
};


Leg::Leg(String n, int hipId, int legId, int kneeId) {
  name = n;
  hip.write(90);
  leg.write(0);
  knee.write(0);

  hip.attach(hipId + 1);
  leg.attach(legId + 1);
  knee.attach(kneeId + 1);
}

Leg::Leg() {
}





void setup() {
  Leg leg1;
  // Leg leg2;
  Leg leg3;
  Leg leg4;

  leg1 = Leg("one", 1, 2, 3);
  leg3 = Leg("two", 7, 8, 9);
  leg4 = Leg("three", 10, 11, 12);
  
}

void loop() {
  // leg4.setLeg(180);
  // leg4.setKnee(120);
}






