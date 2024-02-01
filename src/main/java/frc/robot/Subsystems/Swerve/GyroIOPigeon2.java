package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants.SwerveConstants;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;

    public GyroIOPigeon2() {
        pigeon = new Pigeon2(SwerveConstants.pigeonID, SwerveConstants.canivore);
        Pigeon2Configuration config = new Pigeon2Configuration();
        pigeon.getConfigurator().apply(config);

        pigeon.reset();
        pigeon.setYaw(0);
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPositionDeg = pigeon.getYaw().getValueAsDouble();
        inputs.pitchPositionDeg = pigeon.getPitch().getValueAsDouble();
        inputs.rollPositionDeg = pigeon.getRoll().getValueAsDouble();
        inputs.zAngularVelocityDegPerSec = pigeon.getAngularVelocityZDevice().getValueAsDouble();
    }

    /**
     * Sets the current Gyro yaw
     * @param yaw value to set yaw to, in degrees
     */
    public void setYaw(double yaw) {
        pigeon.setYaw(yaw);
    }
}
