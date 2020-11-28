/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.wheel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WheelSpinner;

public class SpinWheelFour extends CommandBase {
  WheelSpinner wheelSpinner;

  /**
   * Creates a new SpinWheelFour.
   */
  public SpinWheelFour(WheelSpinner wheelSpinner) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wheelSpinner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //wheelSpinner.addColors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wheelSpinner.spinFourRevolutions();
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
