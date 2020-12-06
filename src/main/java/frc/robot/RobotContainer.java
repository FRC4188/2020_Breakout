/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.WheelSpinner;
import frc.robot.utils.CspController;
import frc.robot.utils.ButtonBox;
import frc.robot.commands.drive.ManualDrive;
import frc.robot.commands.groups.autonomous.Default;
import frc.robot.commands.wheel.SpinRevolutions;
import frc.robot.commands.wheel.SpinToColor;
import frc.robot.commands.wheel.SpinWheel;
//import frc.robot.commands.wheel.ToggleWheel;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  Drivetrain drivetrain = new Drivetrain();
  Limelight limelight = new Limelight();
  Magazine magazine = new Magazine();
  Shooter shooter = new Shooter(); 
  Turret turret = new Turret();
  WheelSpinner wheelSpinner = new WheelSpinner();

  CspController pilot = new CspController(0);
  CspController copilot = new CspController(1);
  ButtonBox buttonbox = new ButtonBox(2);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    setDefaultCommands();
    // Configure the button bindings
    configureButtonBindings();
  }

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(new ManualDrive(drivetrain, pilot));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //pilot.getRbButtonObj().whenPressed(new ToggleWheel(wheelSpinner));
    pilot.getAButtonObj().whileHeld(new SpinWheel(wheelSpinner, .3));
    pilot.getAButtonObj().whenReleased(new SpinWheel(wheelSpinner, 0));
    pilot.getDpadLeftButtonObj().whenPressed(new SpinToColor(wheelSpinner, "blue")); 
    pilot.getDpadRightButtonObj().whenPressed(new SpinToColor(wheelSpinner, "red"));
    pilot.getDpadUpButtonObj().whenPressed(new SpinToColor(wheelSpinner, "green"));
    pilot.getDpadDownButtonObj().whenPressed(new SpinToColor(wheelSpinner, "yellow"));
    pilot.getXButtonObj().whenPressed(new SpinRevolutions(wheelSpinner, 4.0));  

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new Default();
  }
}
