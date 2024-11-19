#include <Servo.h>

Servo myservo;  // create servo object to control a servo

int degree = 0;
int offset = 0;
float multiplier = 1;
String prompt = "";
int pinNum = 2;

void setup() {
  Serial.begin(9600);

  myservo.attach(pinNum);
  myservo.write(0);
  delay(100);  // attaches the servo on pin 9 to the servo object
  
  Serial.println("-------------------------------");
  Serial.println("Ready! Type d, m, or o followed by an integer to change deg, mult, or offset respectively. Ex. \"d90\" sets the angle to 90.");
  Serial.println("-------------------------------");
}

void loop() {

  if (Serial.available()>0) {
    prompt = Serial.readString();
    Serial.println();
    Serial.println("-------------------------------");
    Serial.println("Received: " + prompt);

    // d for degree or m for mult
    if (prompt.startsWith("d")) {
      prompt = prompt.substring(1);
      degree = prompt.toInt();
      Serial.println("Set deg to: " + prompt);
      myservo.write((degree * multiplier) + offset);
      delay(1500);
    }

    else if (prompt.startsWith("m")) {
      prompt = prompt.substring(1);
      multiplier = prompt.toFloat();
      Serial.println("Set mult to: " + prompt);
    }

    else if (prompt.startsWith("o")) {
      prompt = prompt.substring(1);
      offset = prompt.toInt();
      Serial.println("Set offset to: " + prompt);
    }

    else if (prompt.startsWith("s")) {
      prompt = prompt.substring(1);
      int go = prompt.toInt();
      myservo.write((0 * multiplier) + offset);
      delay(1000);
      myservo.write((go * multiplier) + offset);
      delay(1000);
    }

    else if (prompt.startsWith("p")) {
      prompt = prompt.substring(1);
      pinNum = prompt.toInt();
      myservo.attach(pinNum);
      myservo.write(0);
      delay(1000);
    }

    Serial.println();
    Serial.print("Current angle: ");
    Serial.println(degree);
    Serial.print("Current pin number: ");
    Serial.println(pinNum);
    Serial.print("Current multiplier: ");
    Serial.println(multiplier);
    Serial.print("Current offset: ");
    Serial.println(offset);
    Serial.println();

    prompt="";
  }

}