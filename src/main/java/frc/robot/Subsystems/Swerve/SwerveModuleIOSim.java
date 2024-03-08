package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private FlywheelSim driveSim;
    private FlywheelSim angleSim;

    private double angleAbsolutePosRot = Math.random();
    private double anglePercentOut = 0.0;

    private double drivePercentOut = 0.0;

    public SwerveModuleIOSim() {
        // Using flywheels to simulate motors
        driveSim = new FlywheelSim(DCMotor.getFalcon500(1), SwerveConstants.driveGearRatio, 0.025);
        angleSim = new FlywheelSim(DCMotor.getFalcon500(1), SwerveConstants.angleGearRatio, 0.004);
    }

    public void updateInputs(SwerveModuleIOInputs inputs) {
        driveSim.update(Constants.loopPeriodSecs);
        angleSim.update(Constants.loopPeriodSecs);

        double angleDiffRot = (angleSim.getAngularVelocityRPM() / 60) * Constants.loopPeriodSecs;
        angleAbsolutePosRot += angleDiffRot;
        while (angleAbsolutePosRot < 0) {
            angleAbsolutePosRot += 1;
        }
        while (angleAbsolutePosRot > 1) {
            angleAbsolutePosRot -= 1;
        }

        inputs.drivePositionRot += (driveSim.getAngularVelocityRPM() / 60) * Constants.loopPeriodSecs;
        inputs.driveVelocityRotPerSec = driveSim.getAngularVelocityRPM() / 60;

        inputs.canCoderPositionRot = angleAbsolutePosRot;

        inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
        inputs.odometryDrivePositionsRad = new double[] {Units.rotationsToRadians(inputs.drivePositionRot)};
        inputs.odometryAnglePositions = new Rotation2d[] {Rotation2d.fromRotations(inputs.canCoderPositionRot)};
    }

    /**
     * Sets the percent out of the drive motor
     * @param percent percent to set it to, from -1.0 to 1.0
     */
    public void setDrivePercentOut(double percent) {
        double volts = MathUtil.clamp(percent * 12, -12.0, 12.0);
        drivePercentOut = volts / 12;
        driveSim.setInputVoltage(volts);
    }

    /**
     * Stops the drive motor
     */
    public void stopDrive() {
        driveSim.setInputVoltage(0);
        drivePercentOut = 0;
    }

    /**
     * Sets the percent out of the angle motor
     * @param percent percent to set it to, from -1.0 to 1.0
     */
    public void setAnglePercentOut(double percent) {
        double volts = MathUtil.clamp(percent * 12, -12.0, 12.0);
        anglePercentOut = volts / 12;
        angleSim.setInputVoltage(volts);
    }

    /**
     * Stops the angle motor
     */
    public void stopAngle() {
        angleSim.setInputVoltage(0);
        anglePercentOut = 0;
    }
}
