/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.subsystems.WheelSpinner;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;

public class WheelSpinner extends SubsystemBase {
  private Solenoid wheelSpinnerSolenoid = new Solenoid(4);
  private CANSparkMax wheelSpinnerMotor = new CANSparkMax(26, MotorType.kBrushless);
  private CANEncoder wheelSpinnerEncoder = wheelSpinnerMotor.getEncoder();
  private PIDController wheelPID = new PIDController(1.0, 0.0 ,0.0);

  private I2C.Port i2cPort = I2C.Port.kOnboard;
  private ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private ColorMatch colorMatch = new ColorMatch();

  public boolean isRaised = true;
  public boolean isOnColor = true;
  Color detectedColor = colorSensor.getColor();
  double IR = colorSensor.getIR();
  int proximity = colorSensor.getProximity();
  ColorMatchResult match = colorMatch.matchClosestColor(detectedColor);
  double halfRevolutions = 0;
  double revolutions = 0;
  
  /**
   * Runs every loop.
   */
  public void periodic() {
      SmartDashboard.putBoolean("Wheel Spinner Raised", isRaised());
  }

  /**
   * Sets wheel spinner to a certain percent output.
   */
  public void setPercentage(double percent) {
      wheelSpinnerMotor.set(percent);
  }

  /**
   * Sets wheel spinner to raised position.
   */
  public void raise() {
      wheelSpinnerSolenoid.set(false);
      isRaised = true;
  }

  /**
   * Sets wheel spinner to lowered position.
   */
  public void lower() {
      wheelSpinnerSolenoid.set(true);
      isRaised = false;
  }

  /**
   * Returns true if wheel spinner is in the raised position.
   */
  public boolean isRaised() {
      return isRaised;
  }

  public boolean isOnColor() {
    return isOnColor;
  }

  public double getRevolutions() {
    Color initColor = detectedColor;

    if (detectedColor != initColor) isOnColor = false;
    if (detectedColor == initColor) halfRevolutions++;
    return halfRevolutions / 2;
  }

  public void spinFourRevolutions() {
    revolutions = getRevolutions();

    if (revolutions != 4.0) wheelSpinnerMotor.set(1);
    else wheelSpinnerMotor.set(0);
  }

  public void spinToColor(Color toColor) {
      if ((toColor == Constants.kRED && match.color == Constants.kBLUE) ||
        (toColor == Constants.kYELLOW && match.color == Constants.kGREEN) ||
        (toColor == Constants.kBLUE && match.color == Constants.kRED) ||
        (toColor == Constants.kGREEN && match.color == Constants.kYELLOW)) {
          wheelSpinnerMotor.set(wheelPID.calculate(wheelSpinnerEncoder.getPosition(), 42));
        } else if ((toColor == Constants.kGREEN && match.color == Constants.kBLUE) ||
        (toColor == Constants.kRED && match.color == Constants.kGREEN) ||
        (toColor == Constants.kYELLOW && match.color == Constants.kRED) ||
        (toColor == Constants.kBLUE && match.color == Constants.kYELLOW)) {
          wheelSpinnerMotor.set(wheelPID.calculate(wheelSpinnerEncoder.getPosition(), 21));
        } else if ((toColor == Constants.kYELLOW && match.color == Constants.kBLUE) ||
        (toColor == Constants.kBLUE && match.color == Constants.kGREEN) ||
        (toColor == Constants.kGREEN && match.color == Constants.kRED) ||
        (toColor == Constants.kRED && match.color == Constants.kYELLOW)) {
          wheelSpinnerMotor.setInverted(true); 
          wheelSpinnerMotor.set(wheelPID.calculate(wheelSpinnerEncoder.getPosition(), 21));
        } wheelSpinnerMotor.set(0);
  }

  public void addColors() {
    colorMatch.addColorMatch(Constants.kBLUE);
    colorMatch.addColorMatch(Constants.kRED);
    colorMatch.addColorMatch(Constants.kYELLOW);
    colorMatch.addColorMatch(Constants.kGREEN);
  }
}