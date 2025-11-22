/*
Program: ADD V Code
Creation: November 18th, 2025
Contributors: Daniel Principe
Use: The code that goes into ADD V
*/


#include <Alfredo_NoU2.h>
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
