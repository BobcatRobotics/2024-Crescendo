package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.lang.Object;
import com.ctre.phoenix6.jni.CtreJniWrapper;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;


public class ClimberSubsystem extends SubsystemBase {
  public static enum LockState {LOCKED, UNLOCKED};
  
  /**
   * Creates a new subsystem.
   */

  // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/core/CoreTalonFX.html
  // API Reference for the motors that we are using

  private final rightClimberMotor;
  private final leftClimberMotor;

  public ClimberSubsystem(int motorConst1, int motorConst2, kTalonFXCANbus) {
    leftClimberMotor = new TalonFX(motorConst1, kTalonFXCANbus);
    rightClimberMotor = new TalonFX(motorConst2, kTalonFXCANbus);
  }

  public void climb(double speed ) {
    
  }

  @Override
  public void periodic() {
  }

  public void lock() {
  }

  public void unlock() {
  }
}
