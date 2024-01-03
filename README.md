# Bobcat-Base-Swerve

This repository contains Bobcat Robotics FRC team 177 base swerve code. Our swerve uses SDS MK4i L3 modules with Falcon 500 motors, CANcoders, a Pigeon2 gyro, and a CANivore.

## Using this code

Before using this code, you should do the following:

- Update the gyro, CANivore, and all motors and encoders to the latest Phoenix 6 firmware
- Make sure CANivore name is correct
- Set CAN ids in Phoenix Tuner to match what is in the code
- Adjust swerve module offsets
- Tune drive feedforward values with sysid
- Ensure the drivetrain constants are correct
- Set the useFOC variable in constants depending on whether the motors have been licensed with Phoenix 6 Pro
- Tune all PID constants
