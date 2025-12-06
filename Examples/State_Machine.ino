// REPLACE ALL OF "<Desired Button Here>" WITH THE CORRECT BUTTON
// All angles are a copy of the start position of the parts
if (PestoLink.buttonHeld("<Desired Button Here>")) {
  // L1
  elevatorMotorPID.setAngle(0);
  intakeServo.write(180);
  pivotingServo.write(135);
} else if (PestoLink.buttonHeld("<Desired Button Here>")) {
  // L2
  elevatorMotorPID.setAngle(0);
  intakeServo.write(180);
  pivotingServo.write(135);
} else if (PestoLink.buttonHeld("<Desired Button Here>")) {
  // L3
  elevatorMotorPID.setAngle(0);
  intakeServo.write(180);
  pivotingServo.write(135);
} else if (PestoLink.buttonHeld("<Desired Button Here>")) {
  // L4
  elevatorMotorPID.setAngle(0);
  intakeServo.write(180);
  pivotingServo.write(135);
}
