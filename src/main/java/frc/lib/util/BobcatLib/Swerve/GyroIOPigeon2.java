package frc.lib.util.BobcatLib.Swerve;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.Swerve.PhoenixOdometryThread;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;

    private final StatusSignal<Double> yaw;
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;
    private final StatusSignal<Double> yawVelocity;

    public GyroIOPigeon2() {
        pigeon = new Pigeon2(SwerveConstants.pigeonID);
        Pigeon2Configuration config = new Pigeon2Configuration();
        pigeon.getConfigurator().apply(config);
        // config.MountPose.MountPoseYaw = 180;
        // pigeon.getConfigurator().apply(config);

        pigeon.reset();
        pigeon.setYaw(0);

        yaw = pigeon.getYaw();
        yawVelocity = pigeon.getAngularVelocityZWorld();

        yaw.setUpdateFrequency(50);
        yawVelocity.setUpdateFrequency(50);
        pigeon.optimizeBusUtilization();

        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon, pigeon.getYaw());
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.zAngularVelocityDegPerSec = yawVelocity.getValueAsDouble();

        inputs.odometryYawTimestamps =
            yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions =
            yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(value))
                .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }

    /**
     * Sets the current Gyro yaw
     * @param yaw value to set yaw to, in degrees
     */
    public void setYaw(double yaw) {
        pigeon.setYaw(yaw);
    }
}
