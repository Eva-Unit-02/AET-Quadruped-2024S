#include <Servo.h>

char rec;
boolean newData = false;

Servo myservo;

void setup() {
    Serial.begin(9600);
    Serial.println("<Arduino is ready>");
    myservo.attach(9);
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
            myservo.write(0);
        if (rec == 's')
            myservo.write(90);
        if (rec == 'd')
            myservo.write(180);

        newData = false;
    }
}