package frc.robot.Commands;

import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.Swerve.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve swerve;    
    private Double translation;
    private Double fineStrafe;
    private Double strafe;
    private Double rotation;
    private Boolean robotCentric;
    private Double fineTrans;

    public TeleopSwerve(Swerve swerve, Double translation, Double strafe, Double rotation, Boolean robotCentric, Double fineStrafe, Double fineTrans) {
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
        double translationVal = MathUtil.applyDeadband(translation, SwerveConstants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafe, SwerveConstants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotation, SwerveConstants.stickDeadband); //from 0 to one

        if (strafeVal == 0.0) {
            strafeVal = fineStrafe;
        } 
        if(translationVal == 0.0) {
            translationVal = fineTrans;
        }
        /* Drive */
        swerve.drive(
            new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed), 
            rotationVal * SwerveConstants.maxAngularVelocity,
            !robotCentric, 
            true
        );
    }
}