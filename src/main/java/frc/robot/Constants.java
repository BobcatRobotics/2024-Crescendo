package frc.robot;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.util.ModuleConstants;
import frc.lib.util.limelightConstants;
import static edu.wpi.first.apriltag.AprilTagFields.k2024Crescendo;


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
    //public static final Integer AmpConstants = 0;

    public static final class SwerveConstants {
        public static final String canivore = "CANt_open_file";

        public static final int pigeonID = 0;

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
        public static final double wheelCircumference = Units.inchesToMeters(4.0)*Math.PI; // 3.8990 3.8985 3.8925 3.8995 checked 2/10/2024 9:36:45 AM
        public static final double angleGearRatio = ((150.0 / 7.0) / 1.0);
        // public static final double driveGearRatio =  (6.12 / 1.0);
        public static final double driveGearRatio =  (5.36 / 1.0);
        
        /* Auto Constants */
        public static final double translationKP = 2.0087;
        public static final double translationKI = 0.0;
        public static final double translationKD = 0.0;

        public static final double rotationKP = 2;
        public static final double rotationKI = 0.0;
        public static final double rotationKD = 0.0;

        /* Teleop Constants */
        public static final double teleopRotationKP = 1.5;
        public static final double teleopRotationKI = 0.0;
        public static final double teleopRotationKD = 0.0;

        /* Module Translations */
        public static final Translation2d[] moduleTranslations = new Translation2d[] {
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        };

        /* Swerve Kinematics */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(moduleTranslations);

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

        /* 
         * Offsets must be done with bevels facing towards spivit motor
         */
        /* FRONT LEFT */
        public static final class Module0Constants {
            public static final int cancoderID = 1;
            public static final int angleMotorID = 2;
            public static final int driveMotorID = 1;

            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(104.8535 + 180); // 109.1 353.32

            public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, cancoderID, angleOffset);
        }

        /* FRONT RIGHT */
        public static final class Module1Constants {
            public static final int cancoderID = 2;
            public static final int angleMotorID = 4;
            public static final int driveMotorID = 3;

            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(210.4102 - 180); // 214.1 9.14

            public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, cancoderID, angleOffset);
        }

        /* BACK LEFT */
        public static final class Module2Constants {
            public static final int cancoderID = 3;
            public static final int angleMotorID = 6;
            public static final int driveMotorID = 5;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(251.4551 - 180); // 203.1 234.66

            public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, cancoderID, angleOffset);
        }

        /* BACK RIGHT */
        public static final class Module3Constants {
            public static final int cancoderID = 4;
            public static final int angleMotorID = 8;
            public static final int driveMotorID = 7;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(141.3281 + 180); // 51.9 285.29

            public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, cancoderID, angleOffset);
        }

        public static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(3));

    }

    public static final class FieldConstants{
        //1 is closest to AMP, 5 is closest to SOURCE
        public static final Translation2d centerlineNote1 = new Translation2d(250.5,29.64);
        public static final Translation2d centerlineNote2 = new Translation2d(250.5,95.64);
        public static final Translation2d centerlineNote3 = new Translation2d(250.5,161.64);
        public static final Translation2d centerlineNote4 = new Translation2d(250.5,227.64);
        public static final Translation2d centerlineNote5 = new Translation2d(250.5,293.64);

        public static final double noteDiameter = Units.inchesToMeters(14);

        public static final double speakerHeight = Units.inchesToMeters(80.4375); // Center of opening
        
        public static final Translation2d blueSpeakerPose = new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42)); // Center of back of the opening
        public static final Translation2d redSpeakerPose = new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42)); // Center of back of the opening //652.73
  
    }


    
    public static final class AprilTagConstants{
    public static AprilTagFieldLayout layout;
    static{ 
        try{
            layout = AprilTagFieldLayout.loadFromResource(k2024Crescendo.m_resourceFile);
        }catch(Exception e){
            e.printStackTrace();
        }
    }
    }

    public static final class LimelightConstants{




        public static final class intake{
 
            public static final String name = "limelight-intake";
            public static final double verticalFOV = 49.7; //degrees obviously
            public static final double horizontalFOV = 63.3;
            public static final double limelightMountHeight = Units.inchesToMeters(20.5);
            public static final int detectorPiplineIndex = 2; 
            public static final int apriltagPipelineIndex = 1;
            public static final int horPixles = 1280;
            public static final double filterTimeConstant=  0.1; // in seconds, inputs occuring over a time period significantly shorter than this will be thrown out
            public static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.1,0.1, Units.degreesToRadians(10));
            public static final int movingAverageNumTaps = 20;

            public static final limelightConstants constants = new limelightConstants(name, verticalFOV, horizontalFOV, limelightMountHeight, detectorPiplineIndex, apriltagPipelineIndex,horPixles, filterTimeConstant,visionMeasurementStdDevs,movingAverageNumTaps);

            public static final String ip = "10.1.77.11";


        }

        public static final class shooterLeft{
            public static final String name="limelight-left";
            public static final double verticalFOV = 49.7; //degrees obviously
            public static final double horizontalFOV = 63.3;
            public static final double limelightMountHeight = Units.inchesToMeters(20.5);
            public static final int detectorPiplineIndex = 2; 
            public static final int apriltagPipelineIndex = 1;
            public static final int horPixles = 1280;
            public static final double filterTimeConstant=  0.1; // in seconds, inputs occuring over a time period significantly shorter than this will be thrown out
            public static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.1,0.1, Units.degreesToRadians(10));
            public static final int movingAverageNumTaps = 20;

            public static final limelightConstants constants = new limelightConstants(name, verticalFOV, horizontalFOV, limelightMountHeight, detectorPiplineIndex, apriltagPipelineIndex,horPixles, filterTimeConstant,visionMeasurementStdDevs,movingAverageNumTaps);
            public static final String ip = "10.1.77.13";

        }
        
        public static final class shooterRight{
            public static final String name="limelight-right";
            public static final double verticalFOV = 49.7; //degrees obviously
            public static final double horizontalFOV = 63.3;
            public static final double limelightMountHeight = Units.inchesToMeters(20.5);
            public static final int detectorPiplineIndex = 2; 
            public static final int apriltagPipelineIndex = 1;
            public static final int horPixles = 1280;
            public static final double filterTimeConstant=  0.1; // in seconds, inputs occuring over a time period significantly shorter than this will be thrown out
            public static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.1,0.1, Units.degreesToRadians(10));
            public static final int movingAverageNumTaps = 20;

            public static final limelightConstants constants = new limelightConstants(name, verticalFOV, horizontalFOV, limelightMountHeight, detectorPiplineIndex, apriltagPipelineIndex,horPixles, filterTimeConstant,visionMeasurementStdDevs,movingAverageNumTaps);
            public static final String ip = "10.1.77.12";

    
        }

    }


    public static final class IntakeConstants {
        public static final int switchMotorID = 9; // This one switches to feed shooter vs trap
        public static final InvertedValue switchMotorInvert = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue switchMotorBrakeMode = NeutralModeValue.Coast;
        public static final double switchCurrentLimit = 80;

        public static final int floorMotorID = 10;
        public static final InvertedValue floorMotorInvert = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue floorMotorBrakeMode = NeutralModeValue.Coast;
        public static final double floorCurrentLimit = 80;

        public static final int outsideMotorID = 11;
        public static final InvertedValue outsideMotorInvert = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue outsideMotorBrakeMode = NeutralModeValue.Coast;
        public static final double outsideCurrentLimit = 80;

        public static final int tofID = 0;
        public static final double tofTresh = 150; // millimeters
    }

    public static final class ClimberConstants{
        public static final int motorID = 18;
        public static final NeutralModeValue climberMotorBrakeMode = NeutralModeValue.Brake;    
        public static final InvertedValue climberMotorInvert =  InvertedValue.Clockwise_Positive;  
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double motionmagicCruiseVelocity = 0;
        public static final double motionmagicAcceleration = 0;
        public static final double motionmagicJerk = 0;
        public static final double rotationAmount = 0.5;

        // the next constant should be the exact number of rotations that the elevator must do to get to the top position
        public static final double rotationToTopAmount = 50.0;

    }

    public static final class ShooterConstants {

        //public static final double bottomFeedForwardBound = 0;
        //public static final double topFeedForwardBound = 0;
        public static final double feedforwardPercentValue = 0.0302;

        public static final double ampDeploySafeValue = 0;

        public static final double rpsTolerance = 100; //THIS IS IN RPS
        public static final double angleTolerance = 1; // THIS IS IN DEGREES

        //some notes on velocity pid
        // kI, kD - not used
        // kP - controls how your output changes with increased error
        // kv - similar to kP, this is essentially a feedforward to calculate the voltage needed to get to the general velocity you want to be at, then the kP takes over for fine tuning
        // kS - static friction gain, there is a small amount of energy lost to friction, this gain accounts for it. You can think of it as just the absolute minimum voltage required to get the motor spinning, this amount is added to every output
        public static final int topMotorID = 12;
        public static final InvertedValue topMotorInvert = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue topMotorBrakeMode = NeutralModeValue.Coast;
        public static final double kTopP = 0.05;//volts/rps
        public static final double kTopI = 0;
        public static final double kTopD = 0.1;
        public static final double kTopV = 0.0113; //volts/rps, feedforward, output per unit of requested velocity 
        public static final double kTopS = 0;// volts, this is added to each output to overcome static friction

        public static final int bottomMotorID = 13;
        public static final InvertedValue bottomMotorInvert = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue bottomMotorBrakeMode = NeutralModeValue.Coast;
        public static final double kBottomP = 0.25;
        public static final double kBottomI = 0;
        public static final double kBottomD = 0.11;
        public static final double kBottomV = 0.014;
        public static final double kBottomS = 0;

        public static final int angleMotorID = 14;
        public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue angleMotorBrakeMode = NeutralModeValue.Brake;
        public static final double kAngleS = 0;
        public static final double kAngleV = 0;
        public static final double kAngleA = 0;
        public static final double kAngleP = 4;
        public static final double kAngleI = 0;
        public static final double kAngleD = 0;

        public static final int cancoderID = 5;
        public static final AbsoluteSensorRangeValue sensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        public static final SensorDirectionValue sensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        public static final Rotation2d offset = new Rotation2d();
        public static final double rotorToSensorRatio = (64/14)*(58/18)*(58/11);

        public static final double bottomLimit = 250.5; // degrees, converted to rotations later on
        public static final double topLimit = 285;
        public static final double podiumShot = 267;
        public static final double wingShot = 256.5;
        public static final double stageShot = 259; //center of stage side closest to speaker
        public static final double subwooferShot = 285;


        public static final int fastShooterRPMSetpoint = 0;
        public static final int slowShooterRPMSetpoint = 0;
    }

    public static final class TrapConstants{
        public static final int rollerID = 17;
        public static final int armID = 16;

        public static final InvertedValue armInvert = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue armBrakeMode = NeutralModeValue.Brake;
        public static final InvertedValue rollerInvert = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue rollerBrakeMode = NeutralModeValue.Coast;
        // public static final double motionmagicCruiseVelocity = 0;
        // public static final double motionmagicAcceleration = 0;
        // public static final double motionmagicJerk = 0;
    }

    public static final class AmpConstants {
        public static final int canID = 15;
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double motionMagicCruiseVelocity = 0;
        public static final double motionMagicAcceleration = 0;
        public static final double motionMagicJerk = 0;
        public static final double rotationAmount = 0.5;
        public static final double softLimitThresh = 0;
		public static final double forwardSoftLimit  = 0;
        public static final double reverseSoftLimit  = 0;
        public static final InvertedValue ampInvertedValue = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue ampNeutralModeValue = NeutralModeValue.Brake;
        public static final double ampStatorCurrentLimit = 80;


        //encoder values
        public static final double deployValue = 0;
        public static final double retractValue = 0;
    }
}


