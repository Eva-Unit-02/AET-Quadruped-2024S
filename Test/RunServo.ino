#include <Servo.h>

Servo myservo;  // Create a servo object

void setup() {
  myservo.attach(9);  // Attaches the servo on pin 9
}

void loop() {
  int angle = 0; // Set the angle to 90 degrees
  myservo.write(angle);  // Set the servo to the specified angle
}