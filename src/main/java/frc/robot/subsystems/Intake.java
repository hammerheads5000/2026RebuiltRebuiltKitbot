// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  VoltageOut voltageRequest = new VoltageOut(0);
  TalonSRX bottomMotor = new TalonSRX(1);
  TalonSRX topMotor = new TalonSRX(2);
  DoubleEntry speedEntry = NetworkTableInstance.getDefault().getDoubleTopic("name").getEntry(0);

  /** Creates a new FuelMechanism. */
  public Intake() {
    bottomMotor.setInverted(false);
    topMotor.setInverted(false);
    bottomMotor.setNeutralMode(NeutralMode.Brake);
    topMotor.setNeutralMode(NeutralMode.Brake);
    bottomMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0.5));
    topMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0.5));
    speedEntry.set(0);
}

  public void setSpeeds(AngularVelocity setpoint, double speed) {
        bottomMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, speed);
        topMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, speed);
    }

  public void stop() {
        bottomMotor.neutralOutput();
        topMotor.neutralOutput();
    } 

  public Command intakeCommand(AngularVelocity speed) {
    return this.runEnd(() -> setSpeeds(speed, 0.5), this::stop);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
