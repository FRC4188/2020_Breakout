/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
  //constants
  private static final Color kBLUE = ColorMatch.makeColor(0.0, 0.255, 0.255);
  private static final Color kRED = ColorMatch.makeColor(0.255, 0.0, 0.0);
  private static final Color kYELLOW = ColorMatch.makeColor(0.255, 0.255, 0.0);
  private static final Color kGREEN = ColorMatch.makeColor(0.0, 0.255, 0.0);
  
  private static final double WHEEL_GEAR_RATIO = 10.0;
  private static final double NEO_ENCODER_TICKS = 42.0;
  private static final double WHEEL_ENCODER_TO_REV = NEO_ENCODER_TICKS * WHEEL_GEAR_RATIO;

  //private Solenoid wheelSpinnerSolenoid = new Solenoid(4);
  private CANSparkMax wheelSpinnerMotor = new CANSparkMax(11, MotorType.kBrushless);
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
  double halfRevolutions = 0.0;
  double revolutions = 0.0;
  
  /**
   * Runs every loop.
   */
  public void periodic() {
      SmartDashboard.putBoolean("Wheel Spinner Raised", isRaised());
      SmartDashboard.putNumber("Red", detectedColor.red);
      SmartDashboard.putNumber("Blue", detectedColor.blue);
      SmartDashboard.putNumber("Green", detectedColor.green);
      SmartDashboard.putNumber("Proximity", proximity);
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
 /*public void raise() {
      wheelSpinnerSolenoid.set(false);
      isRaised = true;
  }

  /**
   * Sets wheel spinner to lowered position.
   */
  /*public void lower() {
      wheelSpinnerSolenoid.set(true);
      isRaised = false;
  }

  /**
   * Returns true if wheel spinner is in the raised position.
   */
  public boolean isRaised() {
      return isRaised;
  }

  public double getRevolutions() {
    Color initColor = detectedColor;

    if (match.color == initColor) halfRevolutions++;
    return halfRevolutions / 2;
  }

  public void spinFourRevolutions() {
    revolutions = getRevolutions();

    if (revolutions != 4.0) wheelSpinnerMotor.set(1);
    else wheelSpinnerMotor.set(0);
  }

  public void spinToColor(String toColor) {
      if ((toColor == "red" && match.color == kBLUE) ||
        (toColor == "yellow" && match.color == kGREEN) ||
        (toColor == "blue" && match.color == kRED) ||
        (toColor == "green" && match.color == kYELLOW)) {
          wheelSpinnerMotor.set(wheelPID.calculate(wheelSpinnerEncoder.getPosition(), 1 * WHEEL_ENCODER_TO_REV));
        } else if ((toColor == "green" && match.color == kBLUE) ||
        (toColor == "red" && match.color == kGREEN) ||
        (toColor == "yellow" && match.color == kRED) ||
        (toColor == "blue" && match.color == kYELLOW) ||
        (toColor == "green" && match.color == kBLUE)) {
          wheelSpinnerMotor.set(wheelPID.calculate(wheelSpinnerEncoder.getPosition(), 0.5 * WHEEL_ENCODER_TO_REV));
        } else if ((toColor == "yellow" && match.color == kBLUE) ||
        (toColor == "blue" && match.color == kGREEN) ||
        (toColor == "green" && match.color == kRED) ||
        (toColor == "red" && match.color == kYELLOW) || 
        (toColor == "yellow" && match.color == kBLUE)) {
          wheelSpinnerMotor.setInverted(true); 
          wheelSpinnerMotor.set(wheelPID.calculate(wheelSpinnerEncoder.getPosition(), 0.5 * WHEEL_ENCODER_TO_REV));
          wheelSpinnerMotor.setInverted(false);
        } wheelSpinnerMotor.set(0); 
  }

  public void addColors() {
    colorMatch.addColorMatch(kBLUE);
    colorMatch.addColorMatch(kRED);
    colorMatch.addColorMatch(kYELLOW);
    colorMatch.addColorMatch(kGREEN);
  }
}