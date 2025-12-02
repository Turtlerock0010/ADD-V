# ADD-V
![83](https://github.com/user-attachments/assets/974e4cbc-5940-4103-8765-78acd9d2f3a7)
> Punkrocker my beloved (This is a placeholder so sadly its gonna be replaced💔)

## About
Welcome to ADD V code respository, A repository for the programmers of Mini FRC team [83] Pink Fluffy Unicorns. Code will be updated and changes will be documented in the [discussion page](https://github.com/Turtlerock0010/ADD-V/discussions/categories/changes) for changes. For team members accessing the code to push into the NoU3, please use `main.ino` as any code in the file should have the latest and most reliable code. A guide on the innerworkings of the code will be created below.

## Guide
This is where you can find all the parts of the code that I deemed to be something that you really want an explanation on what it exactly does in the code as some of the features. The reason why I am doing this now is because from previous MiniFRC code reading and deciphering it is really hard to actually know what does what and why it does what it needs to do. So please use this area as a reference in order to find out what in the world is going on in the code.

### Layouts
<img width="1219" height="611" alt="Screenshot 2025-12-02 at 5 41 00 PM" src="https://github.com/user-attachments/assets/45de75ea-8f05-475f-ab8b-f6c7aa64850f" />

### PID Controller

#### Defining A Motor
- `MotorPID myPIDMotor(myNOUMotor);`
- Defines a motor to the PID controller.
- Requires a NoU motor be already defined before defining the MotorPID.

#### Updating A Motor
- `myPIDMotor.updateMotor();`
- Updates the motor to the correct state.
- Requires the piece of code to be within the `loop()` function of the file.

#### Adjusting Kp Value
- `myPIDMotor.Kp = 0.0;`
- Sets the Kp value to the desired variable.
- Requires a MotorPID to already be created.

#### Adjusting Ki Value
- `myPIDMotor.Ki = 0.0;`
- Sets the Ki value to the desired variable.
- Requires a MotorPID to already be created.

#### Adjusting Kd Value
- `myPIDMotor.Kd = 0.0;`
- Sets the Kd value to the desired variable.
- Requires a MotorPID to already be created.

#### Setting the desired PID angle
- `myPIDMotor.setAngle(90);`
- Sets the motor to the desired angle.
- Requires a MotorPID to already be created.

### General Robot Adjustments

#### Setting Drivetrain Motor Inversion
- `frontLeftMotor.setInverted(true);`
- `frontRightMotor.setInverted(true);`
- `rearLeftMotor.setInverted(true);`
- `rearRightMotor.setInverted(true);`
- Sets inversion or no inversion for the drivetrain motors.
- Requires drivetrain motors to already be defined.
