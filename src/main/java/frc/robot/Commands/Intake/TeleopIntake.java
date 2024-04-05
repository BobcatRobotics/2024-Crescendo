package frc.robot.Commands.Intake;

import java.util.function.BooleanSupplier;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CANdle.CANdle;
import frc.robot.Subsystems.CANdle.CANdleState;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Rumble.Rumble;
import frc.robot.Subsystems.Spivit.Spivit;

public class TeleopIntake extends Command {
    private Intake intake;
    private BooleanSupplier intakeShooter;
    // private BooleanSupplier intakeTrap;
    private BooleanSupplier runOut;
    private BooleanSupplier atSpeed;
    private BooleanSupplier atAngle;
    private BooleanSupplier feed;
    private BooleanSupplier aligning;
    // private Trap trap;
    private boolean trapping = false;
    private boolean ledsoff = false;

    private Rumble rumble;
    private CANdle candle;
    private boolean intook = false;
    private Spivit spivit;
    

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
    public TeleopIntake(Intake intake, BooleanSupplier intakeShooter, BooleanSupplier runOut, BooleanSupplier atSpeed, BooleanSupplier atAngle, BooleanSupplier feed, Rumble rumble, CANdle leds, Spivit spivit, BooleanSupplier aligning) {
        this.intake = intake;
        this.intakeShooter = intakeShooter;
        // this.intakeTrap = intakeTrap;
        this.runOut = runOut;
        this.atSpeed = atSpeed;
        this.atAngle = atAngle;
        this.feed = feed;
        // this.trap = trap;
        this.rumble = rumble;
        this.candle = leds;
        this.spivit = spivit;
        ledsoff = false;
        this.aligning = aligning;
        addRequirements(intake);
    }

    @Override
    public void execute() {
     
        
        if(!intake.hasPiece()){
            intook = false;
        }

        if (feed.getAsBoolean()) {
            intake.intakeToShooter();
            candle.setLEDs(CANdleState.FEED);
            ledsoff = false;
        } else if (runOut.getAsBoolean()) {
            intake.runOut();
            candle.setLEDs(CANdleState.OUTAKE);
            ledsoff = false;
        } else if (intake.hasPiece()) {
            intake.stop();
            if(!aligning.getAsBoolean()){
            candle.setLEDs(CANdleState.INTOOK); //changed from one second
            }
            intook = true;
            ledsoff = false;
        } else if (intakeShooter.getAsBoolean()) {
            intake.intakeToShooter();
            candle.setLEDs(CANdleState.INTAKING);
            ledsoff = false;
        } else {
            intake.stop();
            if(!intake.hasPiece() && !ledsoff){
                candle.setLEDs(CANdleState.OFF);
                ledsoff = true;
            }

        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        // trap.stopRoller();
    }
}
