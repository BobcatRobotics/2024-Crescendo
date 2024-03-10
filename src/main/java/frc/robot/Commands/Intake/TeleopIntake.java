package frc.robot.Commands.Intake;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AmpConstants;
import frc.robot.Constants.TrapConstants;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Rumble.Rumble;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Trap.Trap;

public class TeleopIntake extends Command {
    private Intake intake;
    private BooleanSupplier intakeShooter;
    // private BooleanSupplier intakeTrap;
    private BooleanSupplier runOut;
    private BooleanSupplier atSpeed;
    private BooleanSupplier atAngle;
    private BooleanSupplier feed;
    // private Trap trap;
    private boolean trapping = false;
    private Swerve swerve;

    private Rumble rumble;
    

    /**
     * 
     * @param intake intake subsystem
     * @param intakeShooter should we intake to the shooter
     * @param intakeTrap should we intake to the trap
     * @param runOut should we outtake
     * @param atSpeed are we up to speed
     * @param atAngle are we properly aligned
     * @param feed should we feed the note to the shooter
     */
    public TeleopIntake(Intake intake, BooleanSupplier intakeShooter, BooleanSupplier runOut, BooleanSupplier atSpeed, BooleanSupplier atAngle, BooleanSupplier feed, Rumble rumble, Swerve swerve) {
        this.intake = intake;
        this.intakeShooter = intakeShooter;
        // this.intakeTrap = intakeTrap;
        this.runOut = runOut;
        this.atSpeed = atSpeed;
        this.atAngle = atAngle;
        this.feed = feed;
        // this.trap = trap;
        this.rumble = rumble;
        this.swerve = swerve;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (feed.getAsBoolean()) {
            intake.intakeToShooter();
            // swerve.setLimeLEDS(false);
        } else if (runOut.getAsBoolean()) {
            intake.runOut();
            // swerve.setLimeLEDS(false);
        } else if (intake.hasPiece()) {
            intake.stop();
            swerve.setLimeLEDS(true);
        } else if (intakeShooter.getAsBoolean()) {
            intake.intakeToShooter();
            // swerve.setLimeLEDS(false);
        } else {
            intake.stop();
            swerve.setLimeLEDS(false);
            if(trapping){
            //  trap.stopArm();
            //  trap.stopRoller();
             trapping = false;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        // trap.stopRoller();
    }
}
