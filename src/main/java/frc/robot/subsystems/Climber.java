// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  // list of abilities
  public enum ClimberState { STORED, MOVING, HANGING, MANUAL }
  private ClimberState state = ClimberState.STORED;

  // change to match robot wiring
  private final TalonFX climberMotor = new TalonFX(17); 
  private final TalonFX followerMotor = new TalonFX(18);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withEnableFOC(true);
  private final Debouncer targetDebouncer = new Debouncer(0.1);

  /** Creates a new Climber. */
  public Climber() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

    // pid
    config.Slot0.kP = 2.4;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.1;
    config.Slot0.kS = 0.2;
    config.Slot0.kV = 0.12;
    config.Slot0.kG = 0.4;
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    // current limits
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.5;

    // software limit
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 110.0;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.5;

    // hardware limit
    config.HardwareLimitSwitch.ReverseLimitEnable = true;
    config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;

    // motion magic
    config.MotionMagic.MotionMagicCruiseVelocity = 60;
    config.MotionMagic.MotionMagicAcceleration = 120;
    config.MotionMagic.MotionMagicJerk = 1200;

    climberMotor.getConfigurator().apply(config);
    followerMotor.getConfigurator().apply(config);
    followerMotor.setControl(new StrictFollower(climberMotor.getDeviceID()));
    climberMotor.setPosition(0);
  }
  // target position
  public void setTargetPosition(double rotations) {
    climberMotor.setControl(mmRequest.withPosition(rotations));
    state = ClimberState.MOVING;
  }

  // check target
  public boolean isAtTarget() {
    return targetDebouncer.calculate(
      Math.abs(climberMotor.getClosedLoopError().getValueAsDouble()) < 0.5
    );
  }

  public void setVoltage(double volts) {
    climberMotor.setControl(voltageRequest.withOutput(volts));
    state = ClimberState.MANUAL;
  }

  public void stop() {
    climberMotor.setControl(voltageRequest.withOutput(0));
    state = ClimberState.HANGING;
  }  

  public Command extendCommand(double volts) {
    return this.runEnd(() -> setVoltage(volts), this::stop);
  }

  public Command retractCommand(double volts) {
    return this.runEnd(() -> setVoltage(-Math.abs(volts)), this::stop);
  }

  // auto up
  public Command climbToTopCommand() {
    return this.run(() -> setTargetPosition(110.0))
        .until(this::isAtTarget)
        .withName("ClimbToTop");
  }

  // auto hang
  public Command pullUpAndHangCommand() {
    return this.run(() -> setTargetPosition(2.0))
        .until(this::isAtTarget)
        .andThen(this::stop)
        .withName("PullUpAndHang");
  }

  // zero cmd
  public Command zeroCommand() {
    return this.runOnce(() -> climberMotor.setPosition(0));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Position (Rotations)", climberMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Climber Motor Voltage", climberMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putString("Climber State", state.name());
    SmartDashboard.putNumber("Climber Error", climberMotor.getClosedLoopError().getValueAsDouble());
    SmartDashboard.putNumber("Climber Supply Current", climberMotor.getSupplyCurrent().getValueAsDouble());
  }
}