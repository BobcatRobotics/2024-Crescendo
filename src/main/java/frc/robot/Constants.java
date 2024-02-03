package frc.robot;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.util.ModuleConstants;

public class Constants {
    public static final Mode currentMode = RobotBase.isSimulation() ? Mode.SIM : (RobotBase.isReal() ? Mode.REAL : Mode.REPLAY);

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final double loopPeriodSecs = 0.02; // 50 hz
    public static final Integer AmpConstants = 0;

    public static final class SwerveConstants {
        public static final String canivore = "CANt_open_file";

        public static final int pigeonID = 1;

        public static final double maxSpeed = 4.5; // max MODULE speed, NOT max chassis speed
        public static final double maxAccel = 3; 
        public static final double maxAngularVelocity = Math.PI;
        public static final double maxAngularAcceleration = Math.PI/2;

        public static final double stickDeadband = 0.05;

        public static final boolean useFOC = true;

        /* Drivetrain Constants */
        public static final double trackWidth = 0.521; // 20.5 in -> meters
        public static final double wheelBase = 0.521; // meters
        public static final double driveBaseRadius = Math.sqrt(2 * Math.pow(wheelBase/2, 2));
        public static final double wheelCircumference = Units.inchesToMeters(4.0)*Math.PI;
        public static final double angleGearRatio = ((150.0 / 7.0) / 1.0);
        public static final double driveGearRatio =  (6.12 / 1.0);

        /* Auto Constants */
        public static final double translationKP = 2.0087;
        public static final double translationKI = 0.0;
        public static final double translationKD = 0.0;

        public static final double rotationKP = 2;
        public static final double rotationKI = 0.0;
        public static final double rotationKD = 0.0;

        /* Teleop Constants */
        public static final double teleopRotationKP = 2;
        public static final double teleopRotationKI = 0.0;
        public static final double teleopRotationKD = 0.0;

        /* Swerve Kinematics */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Angle Motor Configs */
        public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;

        public static final double angleKP = 0.3;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;

        public static final double angleKS = 0.0;
        public static final double angleKV = 0.0;
        public static final double angleKA = 0.0;

        public static final boolean angleSupplyCurrentLimitEnable = true;
        public static final double angleSupplyCurrentLimit = 25.0;
        public static final double angleSupplyCurrentThreshold = 40.0;
        public static final double angleSupplyTimeThreshold = 0.1;

        /* Drive  Configs */
        public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        public static final double driveKP = 0.1;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;

        public static final double driveKS = (0.15565 / 12);
        public static final double driveKV = (2.0206 / 12);
        public static final double driveKA = (0.94648 / 12);

        public static final boolean driveSupplyCurrentLimitEnable = true;
        public static final double driveSupplyCurrentLimit = 35.0;
        public static final double driveSupplyCurrentThreshold = 60.0;
        public static final double driveSupplyTimeThreshold = 0.1;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* CanCoder Configs */
        public static final AbsoluteSensorRangeValue sensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        public static final SensorDirectionValue sensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        /* FRONT LEFT */
        public static final class Module0Constants {
            public static final int cancoderID = 1;
            public static final int angleMotorID = 2;
            public static final int driveMotorID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(109.1); // 109.1 353.32

            public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, cancoderID, angleOffset);
        }

        /* FRONT RIGHT */
        public static final class Module1Constants {
            public static final int cancoderID = 2;
            public static final int angleMotorID = 4;
            public static final int driveMotorID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(214.1); // 214.1 9.14

            public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, cancoderID, angleOffset);
        }

        /* BACK LEFT */
        public static final class Module2Constants {
            public static final int cancoderID = 3;
            public static final int angleMotorID = 6;
            public static final int driveMotorID = 5;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(203.1); // 203.1 234.66

            public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, cancoderID, angleOffset);
        }

        /* BACK RIGHT */
        public static final class Module3Constants {
            public static final int cancoderID = 4;
            public static final int angleMotorID = 8;
            public static final int driveMotorID = 7;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(51.9); // 51.9 285.29

            public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, cancoderID, angleOffset);
        }
    }
    public static final class FieldConstants{
        //1 is closest to AMP, 5 is closest to SOURCE
        public static final Translation2d centerlineNote1 = new Translation2d(0,0);
        public static final Translation2d centerlineNote2 = new Translation2d(0,0);
        public static final Translation2d centerlineNote3 = new Translation2d(0,0);
        public static final Translation2d centerlineNote4 = new Translation2d(0,0);
        public static final Translation2d centerlineNote5 = new Translation2d(0,0);

        public static final double noteDiameter = Units.inchesToMeters(14);
    }

    public static final class LimelightConstants{
        public static final double verticalFOV = 49.7; //degrees obviously
        public static final double horizontalFOV = 63.3;
        public static final double limelightMountHeight = Units.inchesToMeters(20.5);
        public static final int detectorPiplineIndex = 7; 
        public static final int horPixles = 1280;
        public static final double filterTimeConstant=  0.1; // in seconds, inputs occuring over a time period significantly shorter than this will be thrown out
    
    }

    public static final class climberConstants{
        public static final int MotorID = 0;
        public static final NeutralModeValue climberMotor = NeutralModeValue.Brake;    
        public static final InvertedValue climberMotorInvert =  InvertedValue.Clockwise_Positive;  
    }

    public static final class shooterConstants {
        public static final int topMotorID = 0;
        public static final InvertedValue topMotorInvert = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue topMotorBrakeMode = NeutralModeValue.Coast;

        public static final int bottomMotorID = 0;
        public static final InvertedValue bottomMotorInvert = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue bottomMotorBrakeMode = NeutralModeValue.Coast;

        public static final int angleMotorID = 0;
        public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue angleMotorBrakeMode = NeutralModeValue.Brake;

        public static final int feederMotorID = 0;
        public static final InvertedValue feederMotorInvert = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue feederMotorBrakeMode = NeutralModeValue.Coast;

    }
}
