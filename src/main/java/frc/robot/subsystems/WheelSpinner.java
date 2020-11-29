/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
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
import com.revrobotics.ControlType;
import com.revrobotics.ColorMatchResult;

public class WheelSpinner extends SubsystemBase {
  //constants
  private static final Color kBLUE = ColorMatch.makeColor(0.206, 0.468, 0.326); //tune color for competition; read first three digits after the decimal
  private static final Color kRED = ColorMatch.makeColor(0.441, 0.420, 0.138); //tune color for competition; read first three digits after the decimal
  private static final Color kYELLOW = ColorMatch.makeColor(0.339, 0.551, 0.110); //tune color for competition; read first three digits after the decimal
  private static final Color kGREEN = ColorMatch.makeColor(0.216, 0.578, 0.206); //tune color for competition; read first three digits after the decimal
  
  private static final double WHEEL_GEAR_RATIO = 10.0;
  private static final double NEO_ENCODER_TICKS = 42.0;
  private static final double WHEEL_ENCODER_TO_REV = NEO_ENCODER_TICKS * WHEEL_GEAR_RATIO;

  private Solenoid wheelSpinnerSolenoid = new Solenoid(4);
  private CANSparkMax wheelSpinnerMotor = new CANSparkMax(11, MotorType.kBrushless);
  private CANEncoder wheelSpinnerEncoder = wheelSpinnerMotor.getEncoder();

  private I2C.Port i2cPort = I2C.Port.kOnboard;
  private ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private ColorMatch colorMatch = new ColorMatch();

  public boolean isRaised = true;
  Color detectedColor = colorSensor.getColor();

  ColorMatchResult match = colorMatch.matchClosestColor(detectedColor);
  double halfRevolutions = 0.0;
  double revolutions = 0.0;
  String testString = "idk";
  
  /**
   * Constructs a new WheelSpinner object and configures devices. 
   */
  public WheelSpinner() {
    addColors();
    wheelSpinnerEncoder.setPositionConversionFactor(WHEEL_ENCODER_TO_REV);
  }
  
  /**
   * Runs every loop.
   */
  public void periodic() {
    getDetectedColor();
    updateShuffleBoard();
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

  /** 
   * Returns the color the sensor detects if the confidence is greater or equal to 99.
   */
  public String getDetectedColor() {
    String colorString = "unknown";
    ColorMatchResult match = colorMatch.matchClosestColor(detectedColor);
    if (match.color == kBLUE) colorString = "blue";
    else if (match.color == kRED) colorString = "red";
    else if (match.color == kGREEN) colorString = "green";
    else if (match.color == kYELLOW) colorString = "yellow";
    SmartDashboard.putNumber("Confidence", match.confidence);
      return colorString;
  }

  /**
   * Writes values to the Shuffleboard.
   */
  public void updateShuffleBoard() {
    detectedColor = colorSensor.getColor();
    SmartDashboard.putBoolean("Wheel Spinner Raised", isRaised());
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("IR", colorSensor.getIR());
    SmartDashboard.putNumber("Proximity", colorSensor.getProximity());
    SmartDashboard.putNumber("Revolutions", getRevolutions());
    SmartDashboard.putNumber("Motor Position", wheelSpinnerEncoder.getPosition());
    SmartDashboard.putString("Color String", getDetectedColor());
    SmartDashboard.putString("testString", testString);
    SmartDashboard.putNumber("Velocity", wheelSpinnerEncoder.getVelocity());
    SmartDashboard.putNumber("temperature", wheelSpinnerMotor.getMotorTemperature());
  }

  /**
   * Adds colors to ColorMatch.
   */
  public void addColors() {
    colorMatch.addColorMatch(kBLUE);
    colorMatch.addColorMatch(kRED);
    colorMatch.addColorMatch(kYELLOW);
    colorMatch.addColorMatch(kGREEN);
  }

  /**
   * Returns the number of revolutions.
   */
  public double getRevolutions() {
    Color initColor = colorSensor.getColor();
    
    if (match.color == initColor) halfRevolutions++;
    return halfRevolutions / 2;
  }

  /**
   * Spins color wheel four times.
   */
  public void spinFourRevolutions() {
    revolutions = getRevolutions();

    if (revolutions != 4.0) wheelSpinnerMotor.set(1);
    else wheelSpinnerMotor.set(0);
  }

  /**
   * Tells you if motor is over 40 degrees Celcius. (for Vincent purposes).
   */
  public boolean motorIsHot() {
    return (wheelSpinnerMotor.getMotorTemperature() > 40.0);
  }

  /**
   * Spins to a destination color based on the input color. 
   * @param toColor Desired color to spin to.
   */
  public void spinToColor(String toColor) {
    if ((toColor == "red" && getDetectedColor() == "blue") ||
      (toColor == "yellow" && getDetectedColor() == "green") ||
      (toColor == "blue" && getDetectedColor() == "red") ||
      (toColor == "green" && getDetectedColor() == "yellow")) {
        wheelSpinnerEncoder.setPosition(2.0);
        testString = "spinning forward 2";
      } else if ((toColor == "green" && getDetectedColor() == "blue") ||
      (toColor == "red" && getDetectedColor() == "green") ||
      (toColor == "yellow" && getDetectedColor() == "red") ||
      (toColor == "blue" && getDetectedColor() == "yellow")) {
        testString = "spinning forward 1";
        wheelSpinnerEncoder.setPosition(1.0);
      } else if ((toColor == "yellow" && getDetectedColor() == "blue") ||
      (toColor == "blue" && getDetectedColor() == "green") ||
      (toColor == "green" && getDetectedColor() == "red") ||
      (toColor == "red" && getDetectedColor() == "yellow")) {
        testString = "spinning backward 1";
        wheelSpinnerEncoder.setPosition(-1.0);
    } else wheelSpinnerMotor.set(0); 
    wheelSpinnerMotor.setInverted(false);
  }
}