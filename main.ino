/*
Program: ADD V Code
Creation: November 18th, 2025
Contributors: Daniel Principe, Owen King
Use: The code that goes into ADD V
*/

#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>
#include <stdio.h>

// ---PID Class Code---
// Gang please ignore for now thx
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

// Drivetrain Init
NoU_Motor frontLeftMotor(1);
NoU_Motor frontRightMotor(4);
NoU_Motor rearLeftMotor(8);
NoU_Motor rearRightMotor(5);

NoU_Motor intakeMotor(3);
NoU_Motor ElevatorMotor(2);
NoU_Servo PivotingServo(1);
NoU_Servo IntakeServo(2);

NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

float measured_angle = 31.416;
float angular_scale = (5.0*2.0*PI) / measured_angle;

void setup() {
  PestoLink.begin("ADD V");
  Serial.begin(115200);

  NoU3.begin();
  NoU3.calibrateIMUs(); // this takes exactly one second. Do not move the robot during calibration.

  // Inversion Parts
  frontLeftMotor.setInverted(true);
  frontRightMotor.setInverted(true);
  rearLeftMotor.setInverted(true);
  rearRightMotor.setInverted(true);
}

void loop() {
  // Ima be real gangalang, I have no idea what is this
  static unsigned long lastPrintTime = 0;
  if (lastPrintTime + 100 < millis()){
      Serial.printf("gyro yaw (radians): %.3f\r\n",  NoU3.yaw * angular_scale );
      lastPrintTime = millis();
  }

  // This measures your batteries voltage and sends it to PestoLink
  float batteryVoltage = NoU3.getBatteryVoltage();
  PestoLink.printBatteryVoltage(batteryVoltage);

  if (PestoLink.isConnected()) {
    // ---Robot Functions---

    // Raise Elevator (TEMPORARY)
    if (PestoLink.buttonHeld(4) || PestoLink.buttonHeld(5)) {
      if (PestoLink.buttonHeld(4)) { // Ts so arbitrary 😭
        ElevatorMotor.setInverted(false);
      } else if (PestoLink.buttonHeld(5)) {
        ElevatorMotor.setInverted(true);
      }
      ElevatorMotor.set(1);
    } else {
      ElevatorMotor.set(0);
    }

    //Intake Servo
    if (PestoLink.buttonHeld(12) || PestoLink.buttonHeld(13)) {
      if (PestoLink.buttonHeld(12)) { // Ts so arbitrary 😭
        intakeMotor.setInverted(false);
      } else if (PestoLink.buttonHeld(13)) {
        intakeMotor.setInverted(true);
      }
      intakeMotor.set(1);
    } else {
      intakeMotor.set(0);
    }

    // Adjust Height
    if (PestoLink.buttonHeld(1)) {
      PivotingServo.write(0);
    } else {
      PivotingServo.write(135);
    }

    if (PestoLink.buttonHeld(0)) {
      IntakeServo.write(180);
    } else {
      IntakeServo.write(110);
    }


    // --- Drivetrain Code ---
    // Sets Axes
    float fieldPowerX = PestoLink.getAxis(0);
    float fieldPowerY = -1 * PestoLink.getAxis(1);
    float rotationPower = -1 * PestoLink.getAxis(2);

    // Get robot heading (in radians) from the gyro
    float heading = NoU3.yaw * angular_scale;

    // Rotate joystick vector to be robot-centric
    float cosA = cos(heading);
    float sinA = sin(heading);

    float robotPowerX = fieldPowerX * cosA + fieldPowerY * sinA;
    float robotPowerY = -fieldPowerX * sinA + fieldPowerY * cosA;

    //set motor power
    drivetrain.holonomicDrive(robotPowerX, robotPowerY, rotationPower);
    NoU3.setServiceLight(LIGHT_ENABLED);
  } else {
    drivetrain.holonomicDrive(0, 0, 0); // lmao the example def took from us
    NoU3.setServiceLight(LIGHT_DISABLED);
  }
}
