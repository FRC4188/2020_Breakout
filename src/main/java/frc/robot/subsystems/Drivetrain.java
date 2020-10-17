/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import java.util.List;
import java.util.NoSuchElementException;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  TalonSRX leftMotor = new TalonSRX(5);
  TalonSRX leftSlave = new TalonSRX(6);
  TalonSRX rightMotor = new TalonSRX(7);
  TalonSRX rightSlave = new TalonSRX(8);

  double leftPower;
  double rightPower;

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    leftSlave.follow(leftMotor);
    rightSlave.follow(rightMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double left, double right) {
    leftMotor.set(ControlMode.PercentOutput, left);
    rightMotor.set(ControlMode.PercentOutput, right);
  }

  public void arcade(double xSpeed, double zRotation, boolean fineControl) {
    if (!fineControl) {
      leftPower = xSpeed - (zRotation * 0.75);
      rightPower = xSpeed + (zRotation * 0.75);
    } else {
      leftPower = (xSpeed * 0.5) - (zRotation * 0.25);
      rightPower = (xSpeed * 0.5) + (zRotation * 0.25);
    }
    set(leftPower, rightPower);
  }
}
