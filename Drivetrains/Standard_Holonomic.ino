/*
Program: Standard Holonomic Drive Code
Creation: November 18th, 2025
Contributors: Daniel Principe
Use: Standard holonomic program
*/


#include <Alfredo_NoU3.h>
#include <PestoLink-Receive.h>
#include <stdio.h>


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
 PestoLink.begin("Standard Holonomic Drive Test");

 // NoU3 Startup
 NoU3.begin();
 NoU3.calibrateIMUs(); // this takes exactly one second. Do not move the robot during calibration.

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
