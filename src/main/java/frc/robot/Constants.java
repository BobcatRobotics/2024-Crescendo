package frc.robot;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.util.ModuleConstants;
import frc.lib.util.limelightConstants;
import static edu.wpi.first.apriltag.AprilTagFields.k2024Crescendo;


public class Constants {
    public static final Mode currentMode = RobotBase.isSimulation() ? Mode.SIM
            : (RobotBase.isReal() ? Mode.REAL : Mode.REPLAY);

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final double loopPeriodSecs = 0.02; // 50 hz
    // public static final Integer AmpConstants = 0;

    public static final boolean fein = false;

    public static final class SwerveConstants {
        public static final String canivore = "CANt_open_file";

        public static final int pigeonID = 0;

        public static final double maxSpeed = 4.5; // max MODULE speed, NOT max chassis speed
        public static final double maxModuleSpeed = 5.5;
        public static final double maxAccel = 3;
        public static final double maxAngularVelocity = 2*Math.PI;
        public static final double maxAngularAcceleration = Math.PI / 2;

        public static final double stickDeadband = 0.02;

        public static final boolean useFOC = true;

        // AUTO ALIGNMENT ONLY !!!!!!11!!!1!1!!!
        public static final double rotationToleranceAlignment = 2.5;

        /* Drivetrain Constants */
        public static final double trackWidth = 0.521; // 20.5 in -> meters
        public static final double wheelBase = 0.521; // meters
        public static final double driveBaseRadius = Math.sqrt(2 * Math.pow(wheelBase / 2, 2));
        public static final double wheelCircumference = Units.inchesToMeters(3.897375) * Math.PI; // avg of 3.8990
                                                                                                  // 3.8985 3.8925
                                                                                                  // 3.8995 checked
                                                                                                  // 2/10/2024 9:36:45
                                                                                                  // AM
        public static final double angleGearRatio = ((150.0 / 7.0) / 1.0);
        // public static final double driveGearRatio = (6.12 / 1.0);
        public static final double driveGearRatio = (5.36 / 1.0);

        /* Auto Constants */
        public static final double translationKP = 6;//7;//5 for outta the way?? idk;//7 before; //tuned for 4 m/s          //2.25; // tuned for .5 m/s
        public static final double translationKI = 0.0;
        public static final double translationKD = 1.4;

        public static final double rotationKP = 4;//4.5 //3 //4
        public static final double rotationKI = 0.0;
        public static final double rotationKD = 0.0; //0.4

        /* Teleop Constants */
        public static final double teleopRotationKP = 2;
        public static final double teleopRotationKI = 0.0;
        public static final double teleopRotationKD = 0.0;

        public static final double autoAlignRotationKP = 3.5;
        public static final double autoAlignRotationKI = 0.0;
        public static final double autoAlignRotationKD = 0.0;

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

        public static final boolean angleStatorCurrentLimitEnable = true;
        public static final double angleStatorCurrentLimit = 20;

        /* Drive Configs */
        public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        public static final double driveKP = 0.1;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;

        public static final double driveKS = 0.14267 / 12;// (0.15565 / 12);
        public static final double driveKV = 2.0718 / 12;// (2.0206 / 12);
        public static final double driveKA = 0.42622 / 12;// (0.94648 / 12);

        public static final boolean driveSupplyCurrentLimitEnable = true;
        public static final double driveSupplyCurrentLimit = 35.0;
        public static final double driveSupplyCurrentThreshold = 60.0;
        public static final double driveSupplyTimeThreshold = 0.1;

        public static final boolean driveStatorCurrentLimitEnable = true;
        public static final double driveStatorCurrentLimit = 60; //for now, maybe 50 if were feeling spicy

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

            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(284.766); //285.46875 //284.94 //285.56 // 109.1 353.32

            public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, cancoderID,
                    angleOffset);
        }

        /* FRONT RIGHT */
        public static final class Module1Constants {
            public static final int cancoderID = 2;
            public static final int angleMotorID = 4;
            public static final int driveMotorID = 3;

            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(27.334); //27.42 214.1 9.14

            public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, cancoderID,
                    angleOffset);
        }

        /* BACK LEFT */
        public static final class Module2Constants {
            public static final int cancoderID = 3;
            public static final int angleMotorID = 6;
            public static final int driveMotorID = 5;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(176.660); //176.40 //176.57 // 203.1 234.66

            public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, cancoderID,
                    angleOffset);
        }
        

        /* BACK RIGHT */
        public static final class Module3Constants {
            public static final int cancoderID = 4;
            public static final int angleMotorID = 8;
            public static final int driveMotorID = 7;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(320.186);// 320.63 // 320.71 // 51.9 285.29

            public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, cancoderID,
                    angleOffset);
        }

        public static final Vector<N3> autostateStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(1)); //formerly 0.1
        public static final Vector<N3> telestateStdDevs = VecBuilder.fill(0.15, 0.15, Units.degreesToRadians(1));

    }

    public static final class FieldConstants {
        public static final double fieldLength = 16.541; //meters
        public static final double fieldWidth = 8.211;

        public static final double centerlineX = 8.2705;

        // 1 is closest to AMP, 5 is closest to SOURCE
        // public static final Translation2d centerlineNote1 = new Translation2d(250.5, 29.64);
        // public static final Translation2d centerlineNote2 = new Translation2d(250.5, 95.64);
        // public static final Translation2d centerlineNote3 = new Translation2d(250.5, 161.64);
        // public static final Translation2d centerlineNote4 = new Translation2d(250.5, 227.64);
        // public static final Translation2d centerlineNote5 = new Translation2d(250.5, 293.64);

        public static final double noteDiameter = Units.inchesToMeters(14);

        public static final double speakerHeight = Units.inchesToMeters(80.4375); // Center of opening
        public static final double redAmpXPos = 14.721; //meters
        public static final double blueAmpXPos = 1.82;
        public static final Translation2d blueSpeakerPose = new Translation2d(Units.inchesToMeters(-1.5 + 12),
                Units.inchesToMeters(218.42)); // Center of the opening
        public static final Translation2d redSpeakerPose = new Translation2d(Units.inchesToMeters(652.73 - 12),
                Units.inchesToMeters(218.42)); // Center of the opening //652.73

        public static final Translation2d bluePassPose = new Translation2d(2.22, 6.42); //y=7.21;
        public static final Translation2d redPassPose = new Translation2d(14.321, 6.42); //y=7.21
        
        public static final Translation2d blueSpeakerPoseSpivit = new Translation2d(Units.inchesToMeters(-1.5),
                Units.inchesToMeters(218.42)); // Center of the opening
        public static final Translation2d redSpeakerPoseSpivit = new Translation2d(Units.inchesToMeters(652.73),
                Units.inchesToMeters(218.42)); // Center of the opening //652.73

        public static final Pose2d blueAmpCenter =
                new Pose2d(new Translation2d(Units.inchesToMeters(72.455), fieldWidth), Rotation2d.fromDegrees(90));

        public static final Pose2d redAmpCenter =
                new Pose2d(new Translation2d(fieldLength-Units.inchesToMeters(72.455), fieldWidth), Rotation2d.fromDegrees(-90));

        
        public static final Pose2d BluePathfindSourceSideShootPos = new Pose2d(3.10, 2.89, Rotation2d.fromDegrees(-35.88));
        public static final Pose2d RedPathfindSourceSideShootPos = new Pose2d(13.441, 2.89, Rotation2d.fromDegrees(-35.88));

        public static final Pose2d BlueLeftoversShootPos = new Pose2d(2.92, 5.53, new Rotation2d());
        public static final Pose2d RedLeftoversShootPos = new Pose2d(13.621, 5.53, new Rotation2d());


                
        
    }

    public static final class AprilTagConstants {
        public static AprilTagFieldLayout layout;
        static {
            try {
                layout = AprilTagFieldLayout.loadFromResource(k2024Crescendo.m_resourceFile);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    public static final class CANdleConstants{
        public static final int CANdleID = 0; 
        public static final int LedCount = 8; //without any led strips
    }

    public static final class LimelightConstants {

        public static final int[] filtertags = {3,4,7,8,9,10,1,2,14,13};
        // public static final int[] filtertags = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};

        public static final int blueSpeakerTag = 7;
        public static final int redSpeakerTag = 4;

        public static double autostdDev = 1.75; //1.75 // dividing distance by thisst
        public static double telestdDev = 64.5; // dividing distance by this

        public static Matrix<N3, N1> trustautostdDev = VecBuilder.fill(0.2, 0.2, 9999999);
        public static Matrix<N3, N1> regautostdDev = VecBuilder.fill(0.9, 0.9, 9999999);

        public static Matrix<N3, N1> trusttelestdDev = VecBuilder.fill(0.2, 0.2, 9999999);
        public static Matrix<N3, N1> regtelestdDev = VecBuilder.fill(0.9, 0.9, 9999999);

        public static final double throwoutDist = 5.5; // meters //It was 4.75 before 9:34:54 AM on 4/4/24, day 1 of new england district championships for the 2024 FIRST robotics competition season: Crescendo, presented by Haas
        public static final double rotationTolerance = 15; // maximum degrees that the limelight can be off from the gyro to update pose
        public static final double poseAmbiguityThreshold = 0.4; //It's what Jonah uses
        public static final double zDistThreshold = 0.5; // meters that the limelight can be off the ground

        public static final class intake {

            public static final String name = "limelight-intake"; //LL3
            public static final double verticalFOV = 49.7; // degrees obviously
            public static final double horizontalFOV = 63.3;
            public static final double limelightMountHeight = Units.inchesToMeters(20.5);
            public static final int detectorPiplineIndex = 2;
            public static final int apriltagPipelineIndex = 1;
            public static final int horPixles = 1280;
            public static final double filterTimeConstant = 0.1; // in seconds, inputs occuring over a time period
                                                                 // significantly shorter than this will be thrown out
            public static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1,
                    Units.degreesToRadians(10));
            public static final int movingAverageNumTaps = 20;

            public static final limelightConstants constants = new limelightConstants(name, verticalFOV, horizontalFOV,
                    limelightMountHeight, detectorPiplineIndex, apriltagPipelineIndex, horPixles, filterTimeConstant,
                    visionMeasurementStdDevs, movingAverageNumTaps);

            public static final String ip = "10.1.77.11";

        }

        public static final class shooterLeft {

            public static final double forward = -0.18415; // meters
            public static final double right = -0.24765;
            public static final double up = -0.34;
            public static final double pitch = 33; // degrees
            public static final double yaw = 180;

            public static final String name = "limelight-left"; //LL3G
            public static final double verticalFOV = 49.7; // degrees obviously
            public static final double horizontalFOV = 63.3;
            public static final double limelightMountHeight = Units.inchesToMeters(20.5);
            public static final int detectorPiplineIndex = 2;
            public static final int apriltagPipelineIndex = 1;
            public static final int horPixles = 1280;
            public static final double filterTimeConstant = 0.1; // in seconds, inputs occuring over a time period
                                                                 // significantly shorter than this will be thrown out
            public static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1,
                    Units.degreesToRadians(10));
            public static final int movingAverageNumTaps = 20;

            public static final limelightConstants constants = new limelightConstants(name, verticalFOV, horizontalFOV,
                    limelightMountHeight, detectorPiplineIndex, apriltagPipelineIndex, horPixles, filterTimeConstant,
                    visionMeasurementStdDevs, movingAverageNumTaps);
            public static final String ip = "10.1.77.13";

        }

        public static final class shooterRight {

            public static final double forward = -0.18415; // meters
            public static final double right = -0.24765;
            public static final double up = -0.34;
            public static final double pitch = 33; // degrees
            public static final double yaw = 180;

            public static final String name = "limelight-right"; //LL3G
            public static final double verticalFOV = 56.2; // degrees obviously
            public static final double horizontalFOV = 82; 
            public static final double limelightMountHeight = Units.inchesToMeters(20.5);
            public static final int detectorPiplineIndex = 2;
            public static final int apriltagPipelineIndex = 1;
            public static final int horPixles = 1280;
            public static final double filterTimeConstant = 0.1; // in seconds, inputs occuring over a time period
                                                                 // significantly shorter than this will be thrown out
            public static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1,
                    Units.degreesToRadians(10));
            public static final int movingAverageNumTaps = 20;

            public static final limelightConstants constants = new limelightConstants(name, verticalFOV, horizontalFOV,
                    limelightMountHeight, detectorPiplineIndex, apriltagPipelineIndex, horPixles, filterTimeConstant,
                    visionMeasurementStdDevs, movingAverageNumTaps);
            public static final String ip = "10.1.77.12";

        }

        public static final class shooterCenter {

            public static final double forward = -0.18415; // meters
            public static final double right = -0.24765;
            public static final double up = -0.34;
            public static final double pitch = 33; // degrees
            public static final double yaw = 180;

            public static final String name = "limelight-center"; //LL3G
            public static final double verticalFOV = 56.2; // degrees obviously
            public static final double horizontalFOV = 82; 
            public static final double limelightMountHeight = Units.inchesToMeters(20.5);
            public static final int detectorPiplineIndex = 2;
            public static final int apriltagPipelineIndex = 1;
            public static final int horPixles = 1280;
            public static final double filterTimeConstant = 0.1; // in seconds, inputs occuring over a time period
                                                                 // significantly shorter than this will be thrown out
            public static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1,
                    Units.degreesToRadians(10));
            public static final int movingAverageNumTaps = 20;

            public static final limelightConstants constants = new limelightConstants(name, verticalFOV, horizontalFOV,
                    limelightMountHeight, detectorPiplineIndex, apriltagPipelineIndex, horPixles, filterTimeConstant,
                    visionMeasurementStdDevs, movingAverageNumTaps);
            public static final String ip = "10.1.77.14";

            public static final int fpsPipline = 3;
            public static final int resPipline = 1;

        }

        public static final class intakeTag {

            public static final double forward = -0.18415; // meters
            public static final double right = -0.24765;
            public static final double up = -0.34;
            public static final double pitch = 33; // degrees
            public static final double yaw = 180;

            public static final String name = "limelight-front"; //LL3G
            public static final double verticalFOV = 56.2; // degrees obviously
            public static final double horizontalFOV = 82; 
            public static final double limelightMountHeight = Units.inchesToMeters(20.5);
            public static final int detectorPiplineIndex = 2;
            public static final int apriltagPipelineIndex = 1;
            public static final int horPixles = 1280;
            public static final double filterTimeConstant = 0.1; // in seconds, inputs occuring over a time period
                                                                 // significantly shorter than this will be thrown out
            public static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1,
                    Units.degreesToRadians(10));
            public static final int movingAverageNumTaps = 20;

            public static final limelightConstants constants = new limelightConstants(name, verticalFOV, horizontalFOV,
                    limelightMountHeight, detectorPiplineIndex, apriltagPipelineIndex, horPixles, filterTimeConstant,
                    visionMeasurementStdDevs, movingAverageNumTaps);
            public static final String ip = "10.1.77.15";

        }


    }

    public static final class IntakeConstants {
        public static final int switchMotorID = 9; // This one switches to feed shooter vs trap
        public static final InvertedValue switchMotorInvert = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue switchMotorBrakeMode = NeutralModeValue.Brake;
        public static final double switchCurrentLimit = 80;

        public static final int floorMotorID = 10;
        public static final InvertedValue floorMotorInvert = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue floorMotorBrakeMode = NeutralModeValue.Brake;
        public static final double floorCurrentLimit = 80;

        public static final int outsideMotorID = 11;
        public static final InvertedValue outsideMotorInvert = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue outsideMotorBrakeMode = NeutralModeValue.Brake;
        public static final double outsideCurrentLimit = 80;

        public static final int tofID = 0;
        public static final double tofTresh = 150; // millimeters
    }

    public static final class ClimberConstants {
        public static final int motorID = 18;
        public static final NeutralModeValue climberMotorBrakeMode = NeutralModeValue.Brake;
        public static final InvertedValue climberMotorInvert = InvertedValue.Clockwise_Positive;
        public static final double kP = 0.05;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double motionmagicCruiseVelocity = 0;
        public static final double motionmagicAcceleration = 0;
        public static final double motionmagicJerk = 0;
        public static final double rotationAmount = 0.5;
        public static final double currentLimit = 50;

        public static final double topLimit = -123.4; // rotations
        public static final double bottomLimit = 0;

        // the next constant should be the exact number of rotations that the elevator
        // must do to get to the top position
        public static final double rotationToTopAmount = 50.0;

    }

    public static final class ShooterConstants {

        // public static final double bottomFeedForwardBound = 0;
        // public static final double topFeedForwardBound = 0;
        public static final double feedforwardPercentValue = 0.0302;

        // position needed to deploy the amp hood
        public static final double ampDeploySafeValue = 257; // 266.5;

        public static final double encoderOffsetFromHorizontal = 230.6;
        public static final double noteIdealExitVelocityMPS = 25;

        public static final double rpsTolerance = 200 / 60; // THIS IS IN RPS
        public static final double angleTolerance = 0.5; // THIS IS IN DEGREES

        // some notes on velocity pid
        // kI, kD - not used
        // kP - controls how your output changes with increased error
        // kv - similar to kP, this is essentially a feedforward to calculate the
        // voltage needed to get to the general velocity you want to be at, then the kP
        // takes over for fine tuning
        // kS - static friction gain, there is a small amount of energy lost to
        // friction, this gain accounts for it. You can think of it as just the absolute
        // minimum voltage required to get the motor spinning, this amount is added to
        // every output
        public static final int topMotorID = 12;
        public static final InvertedValue topMotorInvert = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue topMotorBrakeMode = NeutralModeValue.Coast;
        public static final double kTopP = 9; // volts/rps 0.05
        public static final double kTopI = 0;
        public static final double kTopD = 0;
        public static final double kTopV = 0.04; // volts/rps, feedforward, output per unit of requested velocity
                                                   // 0.0113
        public static final double kTopS = 6; // volts, this is added to each output to overcome static friction
        public static final double topCurrentLimit = 100;

        public static final int bottomMotorID = 13;
        public static final InvertedValue bottomMotorInvert = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue bottomMotorBrakeMode = NeutralModeValue.Coast;
        public static final double kBottomP = 9; // .25
        public static final double kBottomI = 0;
        public static final double kBottomD = 0; // .11
        public static final double kBottomV = 0.06; // .014
        public static final double kBottomS = 6;
        public static final double bottomCurrentLimit = 100;

        public static final int angleMotorID = 14;
        public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue angleMotorBrakeMode = NeutralModeValue.Brake;
        public static final double kAngleS = 0;
        public static final double kAngleV = 0;
        public static final double kAngleA = 0;
        public static final double kAngleP = 7.5;
        public static final double kAngleI = 0;
        public static final double kAngleD = 0;
        public static final double angleCurrentLimit = 40;

        public static final int cancoderID = 5;
        public static final AbsoluteSensorRangeValue sensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        public static final SensorDirectionValue sensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        public static final Rotation2d offset = new Rotation2d();
        public static final double rotorToSensorRatio = (64 / 14) * (58 / 18) * (58 / 11);

        public static final double bottomLimit = 250.5; // degrees, converted to rotations later on
        public static final double topLimit = 285;
        public static final double podiumShot = 267;
        public static final double wingShot = 256.5;
        public static final double stageShot = 259; // center of stage side closest to speaker
        public static final double subwooferShot = 285;
        public static final double ampPosition = 274.5; // 272
        public static final double releasehookSetpoint = 285;

        public static final int fastShooterRPMSetpoint = 5000;
        public static final int slowShooterRPMSetpoint = 3000; 
        public static final int ampShootRPMSetpoint = 1500;
        public static final double slowShooterSpivitAngle = 282; //when the shooter is beyond this, use the slow shooter speed

        public static final double holonomicAprilTagThrowoutDistance = 5.5;

        public static final double stow = bottomLimit + 2;

        public static final InterpolatingDoubleTreeMap spivitAngles = new InterpolatingDoubleTreeMap();
        static {
            spivitAngles.put(1.2509, 285.0);
            spivitAngles.put(1.7589, 280.3);
            spivitAngles.put(2.2669, 271.0);
            spivitAngles.put(2.7749, 266.5);
            spivitAngles.put(3.2829, 263.4);
            spivitAngles.put(3.7909, 260.4);
            spivitAngles.put(4.2989, 258.5);
            spivitAngles.put(4.8069, 257.0);
            spivitAngles.put(5.3149, 255.7);
            spivitAngles.put(5.8229, 255.0);
        }

        // public static final double[] tagDist = {1.2509, 1.7589, 2.2669, 2.7749, 3.2829, 3.7909, 4.2989, 4.8069, 5.3149, 5.8229};
        // public static final double[] swerveAngle = {0,28.5};
        // public static final double[][] spivitAngle = {
        //     {285, 285},
        //     {280.3, 277.3},            
        //     {271,272},
        //     {266.5,266.2},
        //     {263.4,262},
        //     {260.4,259.6},
        //     {258.5, 258.9},
        //     {257, 257.4},
        //     {255.4, 255.7},
        //     {255.0,255}
        // };
        // public static final BilinearInterpolation spivitBiLinearInterpolation = new BilinearInterpolation(tagDist, swerveAngle, spivitAngle);
        public static final int outtookShooterRPMDropThresholdForShootingEarlierInAutos = 4500; //4500
    }
    public static final class TrapConstants {
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
        public static final double kP = 0.0543;
        public static final double kI = 0.005;
        public static final double kD = 0.005;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double motionMagicCruiseVelocity = 0;
        public static final double motionMagicAcceleration = 0;
        public static final double motionMagicJerk = 0;
        public static final double rotationAmount = 0.5;
        public static final double softLimitThresh = 0; 
        public static final double forwardSoftLimit = 0;
        public static final double reverseSoftLimit = 0;
        public static final InvertedValue ampInvertedValue = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue ampNeutralModeValue = NeutralModeValue.Brake;
        public static final double ampStatorCurrentLimit = 15;

        // encoder values
        public static final double deployValue = 0;
        public static final double retractValue = 131.8;
        

        public static final double deployTolerance = 1.5; // degrees of tolerance
        public static final double retractTolerance = 2.5;
    }
}
