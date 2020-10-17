/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.CspController;

public class ManualDrive extends CommandBase {

  private Drivetrain drivetrain;

  private CspController pilot;

  private SlewRateLimiter speedLimiter = new SlewRateLimiter(1.5);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(1.5);

  /**
   * Creates a new ManualDrive.
   */
  public ManualDrive(Drivetrain drivetrain, CspController pilot) {
      addRequirements(drivetrain);
      this.drivetrain = drivetrain;
      this.pilot = pilot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean fineControl = pilot.getBumper(Hand.kLeft);
    double zRotation = pilot.getY(Hand.kLeft);
    double xSpeed = pilot.getX(Hand.kRight);

    drivetrain.arcade(xSpeed, -zRotation, fineControl);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
