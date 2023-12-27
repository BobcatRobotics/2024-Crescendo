package frc.robot.Commands;

import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.Swerve.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
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

    public TeleopSwerve(Swerve swerve, DoubleSupplier translation, DoubleSupplier strafe, DoubleSupplier rotation, BooleanSupplier robotCentric, DoubleSupplier fineStrafe, DoubleSupplier fineTrans) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.translation = translation;
        this.strafe = strafe;
        this.rotation = rotation;
        this.robotCentric = robotCentric;
        this.fineStrafe = fineStrafe;
        this.fineTrans = fineTrans;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translation.getAsDouble(), SwerveConstants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafe.getAsDouble(), SwerveConstants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotation.getAsDouble(), SwerveConstants.stickDeadband); //from 0 to one

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
            !robotCentric.getAsBoolean()
        );
    }
}