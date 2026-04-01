// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  VoltageOut voltageRequest = new VoltageOut(0);
  TalonFX shooterMotor = new TalonFX(16);
  DoubleEntry speedEntry = NetworkTableInstance.getDefault().getDoubleTopic("name").getEntry(0);
  BangBangController controller = new BangBangController();
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0.127, 0);

  /** Creates a new Shooter. */
  public Shooter() {
    MotorOutputConfigs outputConfigs = new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
        
    speedEntry.set(0);
    shooterMotor.getConfigurator().apply(outputConfigs);
    
    // calculate doesnt save state remove if just testing
    feedforward.calculate(10, 20); 
  }

    // no rightspeed needed without the srx
  public void setSpeeds(AngularVelocity setpoint) {
    shooterMotor.setVoltage(
        controller.calculate(shooterMotor.getVelocity(true).getValue().in(RotationsPerSecond), setpoint.in(RotationsPerSecond)) * 1 
        + feedforward.calculate(setpoint.in(RotationsPerSecond))
    );
    SmartDashboard.putNumber(getName(), shooterMotor.getVelocity(true).getValue().in(RotationsPerSecond));
  }

  public void stop() {
    shooterMotor.setControl(voltageRequest.withOutput(0));
  }  

  public Command shootCommand(AngularVelocity speed) {
    return this.runEnd(() -> setSpeeds(speed), this::stop);
  }

  public Command intakeCommand(AngularVelocity speed) {
    // since we removed SRX, shootCommand and intakeCommand do the same thing.
    // use negative speed to spin backwards and pull fuel in.
    // negative speed = reverse spin for intake.
    return this.runEnd(() -> setSpeeds(speed), this::stop);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}