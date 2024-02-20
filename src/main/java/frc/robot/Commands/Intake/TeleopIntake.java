package frc.robot.Commands.Intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.Intake;

public class TeleopIntake extends Command {
    private Intake intake;
    private BooleanSupplier intakeShooter;
    private BooleanSupplier intakeTrap;
    private BooleanSupplier runOut;
    private BooleanSupplier atSpeed;
    private BooleanSupplier atAngle;
    private BooleanSupplier feed;

    public TeleopIntake(Intake intake, BooleanSupplier intakeShooter, BooleanSupplier intakeTrap, BooleanSupplier runOut, BooleanSupplier atSpeed, BooleanSupplier atAngle, BooleanSupplier feed) {
        this.intake = intake;
        this.intakeShooter = intakeShooter;
        this.intakeTrap = intakeTrap;
        this.runOut = runOut;
        this.atSpeed = atSpeed;
        this.atAngle = atAngle;
        this.feed = feed;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (feed.getAsBoolean()) {
            intake.intakeToShooter();
        } else if (runOut.getAsBoolean()) {
            intake.runOut();
        } else if (intake.hasPiece()) {
            intake.stop();
        } else if (intakeShooter.getAsBoolean()) {
            intake.intakeToShooter();
        } else if (intakeTrap.getAsBoolean()) {
            intake.intakeToTrap();
        } else {
            intake.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
