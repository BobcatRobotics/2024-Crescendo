// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Spivit;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.ShooterConstants;

public class SpivitIOFalcon implements SpivitIO {

  private TalonFX angleMotor; // this is responsbile for the angle of the shooter
  private PositionDutyCycle angleRequest;
  private DutyCycleOut percentRequest;
  private CANcoder cancoder;

  private StatusSignal<Double> position;
  private StatusSignal<Double> canPosition;
  private StatusSignal<Double> statorCurrent;

  /** Creates a new Spivit. */
  public SpivitIOFalcon() {
    cancoder = new CANcoder(ShooterConstants.cancoderID);
    // CANcoder configurations
    CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
    cancoder.getConfigurator().apply(cancoderConfigs);
    cancoderConfigs.MagnetSensor.SensorDirection = ShooterConstants.sensorDirection;
    cancoderConfigs.MagnetSensor.AbsoluteSensorRange = ShooterConstants.sensorRange;
    cancoderConfigs.MagnetSensor.MagnetOffset = ShooterConstants.offset.getRotations();
    cancoder.getConfigurator().apply(cancoderConfigs);

    angleMotor = new TalonFX(ShooterConstants.angleMotorID); // initializes TalonFX motor 3
    angleRequest = new PositionDutyCycle(0).withEnableFOC(true)
        .withFeedForward(ShooterConstants.feedforwardPercentValue);
    percentRequest = new DutyCycleOut(0).withEnableFOC(true);
    // Angle motor configurations
    TalonFXConfiguration angleConfigs = new TalonFXConfiguration();
    angleMotor.getConfigurator().apply(angleConfigs);
    angleConfigs.MotorOutput.Inverted = ShooterConstants.angleMotorInvert;
    angleConfigs.MotorOutput.NeutralMode = ShooterConstants.angleMotorBrakeMode;
    angleConfigs.Slot0.kP = ShooterConstants.kAngleP;
    angleConfigs.Slot0.kI = ShooterConstants.kAngleI;
    angleConfigs.Slot0.kD = ShooterConstants.kAngleD;
    angleConfigs.Slot0.kS = ShooterConstants.kAngleS;
    angleConfigs.Slot0.kV = ShooterConstants.kAngleV;
    angleConfigs.Slot0.kA = ShooterConstants.kAngleA;
    angleConfigs.Feedback.FeedbackRemoteSensorID = ShooterConstants.cancoderID;
    angleConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    angleConfigs.Feedback.RotorToSensorRatio = ShooterConstants.rotorToSensorRatio;
    angleConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    angleConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ShooterConstants.topLimit / 360; // need to devide by
                                                                                                  // 360 to convert
                                                                                                  // degrees to
                                                                                                  // rotations
    angleConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    angleConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ShooterConstants.bottomLimit / 360;
    angleConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    angleConfigs.CurrentLimits.StatorCurrentLimit = ShooterConstants.angleCurrentLimit;

    angleMotor.getConfigurator().apply(angleConfigs);

    position = angleMotor.getPosition();
    canPosition = cancoder.getAbsolutePosition();
    statorCurrent = angleMotor.getStatorCurrent();
    
    BaseStatusSignal.setUpdateFrequencyForAll(50, position, statorCurrent, canPosition);
    angleMotor.optimizeBusUtilization();
    cancoder.optimizeBusUtilization();
  }

  public void updateInputs(SpivitIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, statorCurrent, canPosition);
    inputs.angleMotorPosition = position.getValueAsDouble() * 360;
    inputs.angleMotorStatorCurrent = statorCurrent.getValueAsDouble();
    inputs.absPos = canPosition.getValueAsDouble()*360;

  }

  /**
   * @param deg degrees
   */
  public void setAngle(double deg) {
    angleMotor.setControl(angleRequest.withPosition(Rotation2d.fromDegrees(deg).getRotations()));
  }

  /**
   * 
   * @return returns a double in degrees
   */
  public double getAngle() {
    return angleMotor.getPosition().getValueAsDouble() * 360; // degrees (hopefully)
  }

  public void stopMotor() {
    angleMotor.stopMotor();
  }

  public void stopMotorFeedforward() {
    angleMotor.setControl(percentRequest.withOutput(ShooterConstants.feedforwardPercentValue));
  }

  /**
   * @param percent [-1, 1]
   */
  public void setPercent(double percent) {
    angleMotor.setControl(percentRequest.withOutput(percent + ShooterConstants.feedforwardPercentValue));
  }

  @Override
  public boolean aligned() {
    return Math.abs(angleMotor.getClosedLoopError().getValueAsDouble() * 360) <= ShooterConstants.angleTolerance;
  }
}
