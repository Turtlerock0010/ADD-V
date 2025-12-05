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

// Motor Functions Init
NoU_Motor intakeMotor(3);
NoU_Motor elevatorMotor(6);
NoU_Servo pivotingServo(1);
NoU_Servo intakeServo(2);

// Drivetrain Init
NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

// PID Init
MotorPID elevatorMotorPID(elevatorMotor);

float measured_angle = 31.416;
float angular_scale = (5.0*2.0*PI) / measured_angle;

void setup() {
  PestoLink.begin("ADD V");
  Serial.begin(115200);

  NoU3.begin();
  NoU3.calibrateIMUs(); // this takes exactly one second. Do not move the robot during calibration.

  // Encoder Startup
  elevatorMotor.beginEncoder();

  // PID Values (IMPORTANT! Either tell Daniel or Yadhu to change the values)
  elevatorMotorPID.Kp = 0.005;
  elevatorMotorPID.Ki = 0;
  elevatorMotorPID.Kd = 0.001;

  elevatorMotorPID.setAngle(0);

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

    // Raise Elevator 
    if (PestoLink.buttonHeld(4) || PestoLink.keyHeld(Key::E)) {
      elevatorMotorPID.setAngle(-4000);
      // The range is from 0 to -4000 with the current elevator,
      // But if the elevator is fixed it can possibly go up to -6000
    } else if (PestoLink.buttonHeld(5) || PestoLink.keyHeld(Key::Q)) {
      elevatorMotorPID.setAngle(0);
    }

    //Intake Servo
    if (PestoLink.buttonHeld(12) || PestoLink.buttonHeld(13) || PestoLink.keyHeld(Key::R) || PestoLink.keyHeld(Key::F)) {
      if (PestoLink.buttonHeld(12) || PestoLink.keyHeld(Key::R)) { // Ts so arbitrary 😭
        intakeMotor.setInverted(false);
      } else if (PestoLink.buttonHeld(13) || PestoLink.keyHeld(Key::F)) {
        intakeMotor.setInverted(true);
      }
      intakeMotor.set(1);
    } else {
      intakeMotor.set(0);
    }

    // Adjust Height
    if (PestoLink.buttonHeld(1) || PestoLink.keyHeld(Key::C)) {
      pivotingServo.write(0);
    } else {
      pivotingServo.write(135); // Set it back to 135
    }

    // Adjusts Intake Arm Angle (Possibly might be removed)
    if (PestoLink.buttonHeld(0) || PestoLink.keyHeld(Key::V)) {
      intakeServo.write(100);
    } else {
      intakeServo.write(180);
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

    // Updates PID State
    elevatorMotorPID.updateMotor();

    // set motor power
    drivetrain.holonomicDrive(robotPowerX, robotPowerY, rotationPower);
    NoU3.setServiceLight(LIGHT_ENABLED);
  } else {
    drivetrain.holonomicDrive(0, 0, 0); // lmao the example def took from us
    NoU3.setServiceLight(LIGHT_DISABLED);
  }
}
