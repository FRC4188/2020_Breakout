/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.wheel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WheelSpinner;

public class ToggleWheel extends CommandBase {

  WheelSpinner wheel; 

  /**
  * Constructs new ToggleWheel command to fire wheel solenoids.
  * Raises wheel if it is currently lowered and vice versa.
  * @param wheel - Wheel subsystem to use.
  */
  public ToggleWheel(WheelSpinner wheel) {
    addRequirements(wheel);
    this.wheel = wheel; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (wheel.isRaised) wheel.lower(); 
    else wheel.raise(); 
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
