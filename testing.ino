#include <Wire.h>
#include <Servo.h>

// Servo pin definitions
#define servoBasePin 3
#define servoElbow1Pin 5
#define servoElbow2Pin 6
#define servoGripperPin 12

Servo servoBase;
Servo servoElbow1;
Servo servoElbow2;
Servo servoGripper;

// Variables to store the servo positions
int currentBasePosition = 90;
int currentElbow1Position = 90;
int currentElbow2Position = 0;
int currentGripperPosition = 180;

void setup() {
  Serial.begin(9600);  // Start serial communication
  
  // Attach the servos to their respective pins
  servoBase.attach(servoBasePin);
  servoElbow1.attach(servoElbow1Pin);
  servoElbow2.attach(servoElbow2Pin);
  servoGripper.attach(servoGripperPin);

  // Set initial positions for all servos
  servoBase.write(currentBasePosition);
  servoElbow1.write(currentElbow1Position);
  servoElbow2.write(currentElbow2Position);
  servoGripper.write(currentGripperPosition);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    // Expecting data in format: base,elbow1,elbow2,gripper
    int values[4];
    int index = 0;
    
    for (int i = 0; i < 4; i++) {
      int commaIndex = data.indexOf(',');
      if (commaIndex != -1) {
        values[i] = data.substring(0, commaIndex).toInt();
        data = data.substring(commaIndex + 1);
      } else if (i == 3) {
        values[i] = data.toInt();
      } else {
        // Error in data format
        return;
      }
    }
    
    // Update servo positions
    servoBase.write(values[0]);
    servoElbow1.write(values[1]);
    servoElbow2.write(values[2]);
    servoGripper.write(values[3]);

    // Update current positions
    currentBasePosition = values[0];
    currentElbow1Position = values[1];
    currentElbow2Position = values[2];
    currentGripperPosition = values[3];
  }
}