package frc.robot.Commands.Swerve;

import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.Swerve.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
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
    private double angleToSpeaker = 0.0;
    private boolean overriden = false;

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
     */
    public TeleopSwerve(Swerve swerve, DoubleSupplier translation, DoubleSupplier strafe, DoubleSupplier rotation, BooleanSupplier robotCentric, DoubleSupplier fineStrafe, DoubleSupplier fineTrans, BooleanSupplier snapToAmp, BooleanSupplier snapToSpeaker) {
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
    }

    @Override
    public void execute() {

        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translation.getAsDouble(), SwerveConstants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafe.getAsDouble(), SwerveConstants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotation.getAsDouble(), SwerveConstants.stickDeadband); //from 0 to one

        if (snapToSpeaker.getAsBoolean() && rotationVal == 0) {
            // Translation2d speaker = swerve.getTranslationToSpeaker();
            // angleToSpeaker = Math.atan(speaker.getY()/speaker.getX());
            angleToSpeaker = Rotation2d.fromDegrees(swerve.getShootWhileMoveBallistics()[0]).getRadians();
            overriden = false;
            
        } else {
            overriden = true;
        }
        // overriden = true;

        if (snapToAmp.getAsBoolean() && rotationVal == 0) {
            snapToAmp = () -> true;
        } else {
            snapToAmp = () -> false;
        }

        /* If joysticks not receiving any normal input, use twist values for fine adjust */
        if (strafeVal == 0.0) {
            strafeVal = fineStrafe.getAsDouble();
        } 
        if(translationVal == 0.0) {
            translationVal = fineTrans.getAsDouble();
        }

        /* Drive */
        swerve.drive(
            new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed), 
            rotationVal * SwerveConstants.maxAngularVelocity,
            !robotCentric.getAsBoolean(),
            snapToAmp.getAsBoolean(),
            snapToSpeaker.getAsBoolean() && !overriden,
            angleToSpeaker
        );
    }
}