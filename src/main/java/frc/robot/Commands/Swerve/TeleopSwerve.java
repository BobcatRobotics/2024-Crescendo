package frc.robot.Commands.Swerve;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Util.BobcatUtil;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve swerve;    
    private DoubleSupplier translation;
    private DoubleSupplier fineStrafe;
    private DoubleSupplier strafe;
    private DoubleSupplier rotation;
    private BooleanSupplier robotCentric;
    private DoubleSupplier fineTrans;
    private BooleanSupplier snapToAmp;
    private BooleanSupplier snapToSpeaker;
    private BooleanSupplier pass;
    private BooleanSupplier ampAssist;
    private double autoAlignAngle = 0.0;
    private boolean overriden = false;
    private PIDController ampAssistController = new PIDController(0.1, 0, 0); //TODO tune

    /**
     * 
     * suppliers are objects that can return a value that will change, for example a method that returns a double can be input as a doubleSupplier 
     * 
     * @param swerve the swerve subsystem
     * @param translation the value to drive the robot forward and backward
     * @param strafe  value to drive the robot left and right
     * @param rotation  value to rotate the drivetrain
     * @param robotCentric field-centric if false
     * @param fineStrafe slow speed control, cancled if translation or strafe is in use
     * @param fineTrans slow speed control, cancled if translation or strafe is in use
     * @param snapToAmp should we automatically rotate to the amp
     * @param snapToSpeaker should we automatically align to the speaker
     * @param pass align to be facing the amp for passing notes
     */
    public TeleopSwerve(Swerve swerve, DoubleSupplier translation, DoubleSupplier strafe, DoubleSupplier rotation, BooleanSupplier robotCentric, DoubleSupplier fineStrafe, DoubleSupplier fineTrans, BooleanSupplier snapToAmp, BooleanSupplier snapToSpeaker, BooleanSupplier pass, BooleanSupplier ampAssist) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.translation = translation;
        this.strafe = strafe;
        this.rotation = rotation;
        this.robotCentric = robotCentric;
        this.fineStrafe = fineStrafe;
        this.fineTrans = fineTrans;
        this.snapToAmp = snapToAmp;
        this.snapToSpeaker = snapToSpeaker;    
        this.pass = pass;   
        this.ampAssist = ampAssist; 
        ampAssistController.setSetpoint(0);
        
    }

    @Override
    public void execute() {

        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translation.getAsDouble(), SwerveConstants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafe.getAsDouble(), SwerveConstants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotation.getAsDouble(), SwerveConstants.stickDeadband); //from 0 to one
        // Rotation2d ampVal = BobcatUtil.isBlue()?Constants.FieldConstants.blueAmpCenter.getRotation() : Constants.FieldConstants.redAmpCenter.getRotation();


        if(pass.getAsBoolean() && rotationVal == 0){
            autoAlignAngle = BobcatUtil.isRed() ? swerve.getAngleToPassArea() : swerve.getAngleToPassArea() + Math.PI;
            overriden = false;
            Logger.recordOutput("Swerve/PassAngle",new Pose2d(swerve.getPose().getTranslation(), Rotation2d.fromRadians(swerve.getAngleToPassArea())));

        }else if (snapToSpeaker.getAsBoolean() && rotationVal == 0) {
            //Translation2d speaker = swerve.getTranslationToSpeaker();
            //angleToSpeaker = Math.atan(speaker.getY()/speaker.getX());
            // angleToSpeaker = swerve.getAngleToSpeakerApriltag().getRadians();
            // Logger.recordOutput("Swerve/AlignmentToSpeaker",new Pose2d(swerve.getPose().getTranslation(), swerve.getAngleToSpeakerApriltag()) );
            // angleToSpeaker = swerve.getAngleToSpeakerTagAuto().getRadians();
            autoAlignAngle = BobcatUtil.isRed() ? swerve.getShootWhileMoveBallistics(ShooterConstants.encoderOffsetFromHorizontal)[0] : swerve.getShootWhileMoveBallistics(ShooterConstants.encoderOffsetFromHorizontal)[0] + Math.PI;
            Logger.recordOutput("Swerve/AlignmentToSpeaker",new Pose2d(swerve.getPose().getTranslation(), swerve.getAngleToSpeakerTagAuto()));
            overriden = false;
            
        } else {
            overriden = true;
        }
        // overriden = true;

        // if (snapToAmp.getAsBoolean() && rotationVal == 0) {
        //     snapToAmp = () -> true;
        // } else {
        //     snapToAmp = () -> false;
        // }


        /* If joysticks not receiving any normal input, use twist values for fine adjust */
        if (strafeVal == 0.0) {
            strafeVal = fineStrafe.getAsDouble();
        } 
        if(translationVal == 0.0) {
            translationVal = fineTrans.getAsDouble();
        }

        if(ampAssist.getAsBoolean()){ //apply aim assist for amp alignment, might need to be -=
            translationVal += ampAssistController.calculate(swerve.getDistanceToAmp());
            Logger.recordOutput("AimAssist/AssistValue", ampAssistController.calculate(swerve.getDistanceToAmp()));
            Logger.recordOutput("AimAssist/distanceToAmp", swerve.getDistanceToAmp());
        }

        Logger.recordOutput("AmpAlign/snapToAmp", snapToAmp.getAsBoolean());
        /* Drive */
        // if(!snapToAmp.getAsBoolean()){
        swerve.drive(
            new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed), 
            rotationVal * SwerveConstants.maxAngularVelocity,
            !robotCentric.getAsBoolean(),
            snapToAmp.getAsBoolean(),
            snapToSpeaker.getAsBoolean() && !overriden,
            autoAlignAngle
        );
    // }else{
    //     swerve.drive(
    //         new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed), 
    //         ampVal,
    //         !robotCentric.getAsBoolean(),
    //         false,
    //         false,
    //         0
    //     );

    // }
        
    }
}