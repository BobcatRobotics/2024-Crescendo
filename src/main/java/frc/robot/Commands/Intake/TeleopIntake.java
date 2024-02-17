package frc.robot.Commands.Intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.Intake;

public class TeleopIntake extends Command {
    private Intake intake;
    private BooleanSupplier intakeShooter;
    private BooleanSupplier intakeTrap;
    private BooleanSupplier runOut;
    private BooleanSupplier shoot;
    private BooleanSupplier atSpeed;
    private BooleanSupplier atAngle;

    public TeleopIntake(Intake intake, BooleanSupplier intakeShooter, BooleanSupplier intakeTrap, BooleanSupplier shoot, BooleanSupplier runOut, BooleanSupplier atSpeed, BooleanSupplier atAngle) {
        this.intake = intake;
        this.intakeShooter = intakeShooter;
        this.intakeTrap = intakeTrap;
        this.runOut = runOut;
        this.shoot = shoot;
        this.atSpeed = atSpeed;
        this.atAngle = atAngle;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (runOut.getAsBoolean()) {
            intake.runOut();
        } else if (shoot.getAsBoolean()) {
            intake.intakeToShooter();
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
