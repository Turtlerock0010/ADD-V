/*
Program: ADD V Code
Creation: November 18th, 2025
Contributors: Daniel Principe
Use: The code that goes into ADD V
*/


#include <Alfredo_NoU3.h>
#include <PestoLink-Receive.h>
#include <stdio.h>

class MotorPID {
  public:
    // Retrieve Given Motor
    MotorPID(NoU_Motor& givenMotor) : myMotor(givenMotor) {
      // hold
    }
    // PID constants
    float Kp;
    float Ki;
    float Kd;

    void setZieglerNicholsConstants(float Ku, float Tu) {
      // Ziegler-Nichols PID formulas
      Kp = 0.6 * Ku;
      Ki = (1.2 * Ku) / Tu;
      Kd = 0.075 * Ku * Tu;
    }

    void updateMotor() {
    if (millis() - lastPIDTime >= PIDInterval) {
      // Read the current angle from your encoder
      float currentAngle = myMotor.getPosition();

      // ** Step 1: Calculate the Error **
      float error = targetAngle - currentAngle;
      Serial.println(currentAngle);
      Serial.println(targetAngle);

      // ** Step 2: Calculate the PID terms **
      float proportionalTerm = Kp * error;

      // Integral term: sum of all past errors
      integralSum += error;
      float integralTerm = Ki * integralSum;

      // Derivative term: rate of change of the error
      float derivativeTerm = Kd * (error - lastError);

      // ** Step 3: Calculate the total PID output **
      float motorSpeedOutput = proportionalTerm + integralTerm + derivativeTerm;

      // ** Step 4: Constrain and set the motor speed **
      // Constrain the output to a valid range for your motor driver (e.g., -255 to 255)
      motorSpeedOutput = constrain(motorSpeedOutput, -1, 1);
      myMotor.set(-motorSpeedOutput);

      // ** Step 5: Update variables for the next iteration **
      lastError = error;
      lastPIDTime = millis();
    } // Time check closing bracket
  } // updateMotor(); closing bracket

  void setAngle(float givenAngle) {
    targetAngle = givenAngle + 10;
  }

  private:
    // Initalize Given Motor
    NoU_Motor& myMotor;

    // Target Angle Adjustment
    float targetAngle = 0;

    // PID variables  
    float lastError = 0.0;
    float integralSum = 0.0;

    //PID timing
    unsigned long lastPIDTime = 0;
    const unsigned long PIDInterval = 20; // time in ms
};

//drive train motors init
NoU_Motor frontleftMotor(1);
NoU_Motor frontrightMotor(2);
NoU_Motor backleftMotor(3);
NoU_Motor backrightMotor(4);

NoU_Drivetrain drivetrain(&frontleftMotor, &frontrightMotor, &backleftMotor, &backrightMotor);


// Quick variable changes
bool zeroPressed = false;

void setup() {
 Serial.begin(115200); // the bauder rate gotta be 9600 or 115200
 PestoLink.begin("Tralalero Tralala");

 frontrightMotor.setInverted(true);
 backrightMotor.setInverted(true);
}


void loop() {
 //--Joystick Controls--
 float horizontalThrottle = 0;
 float verticalThrottle = 0;
 float rotationalThrottle = 0;

 double fl_throttle = 0;
 double fr_throttle = 0;
 double bl_throttle = 0;
 double br_throttle = 0;

 //Set the throttle of the robot based on what key is pressed
 horizontalThrottle = 1 * PestoLink.getAxis(0);
 verticalThrottle =  -1 * PestoLink.getAxis(1);
 rotationalThrottle = 1 * PestoLink.getAxis(2);

 if (PestoLink.buttonHeld(0)) {
    horizontalThrottle /= 2;
    verticalThrottle /= 2;
    rotationalThrottle /= 2;
 }

  drivetrain.holonomicDrive(horizontalThrottle, verticalThrottle, rotationalThrottle);

  PestoLink.update();
}
