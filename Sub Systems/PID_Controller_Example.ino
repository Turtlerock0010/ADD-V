#include <Alfredo_NoU3.h>
#include <PestoLink-Receive.h>

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

// NoU Motor Intialization
NoU_Motor myMotor(1);


//  Motor PID Initalization
MotorPID myPIDMotor(myMotor);

void setup() {
  Serial.begin(115200);
  PestoLink.begin("PID Test");
  NoU3.begin();

  // Start Encoder Readings
  myMotor.beginEncoder();

  // Sets the desired PID values
  myPIDMotor.Kp = 0.004;
  myPIDMotor.Ki = 0.00000;
  myPIDMotor.Kd = 0.0;

  myPIDMotor.setAngle(90); // Sets the desired angle that you want the motor to be at
}

// The loop() function runs continuously
void loop() {
  float batteryVoltage = NoU3.getBatteryVoltage();
    PestoLink.printBatteryVoltage(batteryVoltage);

  // Allows the PID controller to constantly update
  myPIDMotor.updateMotor();
}
