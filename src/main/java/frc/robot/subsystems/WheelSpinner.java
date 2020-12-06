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
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import frc.robot.subsystems.WheelSpinner;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.ColorMatchResult;

public class WheelSpinner extends SubsystemBase {
  //constants
  private static final Color kBLUE = ColorMatch.makeColor(0.204, 0.468, 0.328); //tune color for competition; read first three digits after the decimal
  private static final Color kRED = ColorMatch.makeColor(0.451, 0.415, 0.132); //tune color for competition; read first three digits after the decimal
  private static final Color kYELLOW = ColorMatch.makeColor(0.339, 0.552, 0.108); //tune color for competition; read first three digits after the decimal
  private static final Color kGREEN = ColorMatch.makeColor(0.214, 0.578, 0.206); //tune color for competition; read first three digits after the decimal
  
  private static final double WHEELSPINNER_GEAR_RATIO = 10.0;
  private static final double NEO_ENCODER_TICKS = 42.0;
  private static final double WHEELSPINNER_ENCODER_TO_REV = NEO_ENCODER_TICKS * WHEELSPINNER_GEAR_RATIO;

  private Solenoid wheelSpinnerSolenoid = new Solenoid(4);
  private CANSparkMax wheelSpinnerMotor = new CANSparkMax(11, MotorType.kBrushless);
  private CANEncoder wheelSpinnerEncoder = wheelSpinnerMotor.getEncoder();

  //private ProfiledPIDController wheelSpinnerPID = new ProfiledPIDController(0,0,0, new Constraints(3 * WHEELSPINNER_ENCODER_TO_REV, 3*WHEELSPINNER_ENCODER_TO_REV));
  
  private I2C.Port i2cPort = I2C.Port.kOnboard;
  private ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private ColorMatch colorMatch = new ColorMatch();

  public boolean isRaised = true;
  Color detectedColor = colorSensor.getColor();

  ColorMatchResult match = colorMatch.matchClosestColor(detectedColor);
  double eighthRevolutions =- 1;
  double revolutions = 0.0;
  double totalRevolutions = 0.0 ;
  String lastColor;
  
  /**
   * Constructs a new WheelSpinner object and configures devices. 
   */
  public WheelSpinner() {
    addColors();
    resetEncoders();
    wheelSpinnerMotor.setOpenLoopRampRate(1);
    wheelSpinnerMotor.setIdleMode(IdleMode.kBrake);
  }
  
  /**
   * Runs every loop.
   */
  public void periodic() {
    getDetectedColor();
    updateShuffleBoard();
  }

  /**
   * Resets encoder position to 0.
   */
  public void resetEncoders() {
    wheelSpinnerEncoder.setPosition(0); 
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
   * Returns the color the sensor detects if the confidence is greater or equal to 98.
   */
  public String getDetectedColor() {
    String dColorString = "unknown";
    ColorMatchResult match = colorMatch.matchClosestColor(detectedColor);

    if (match.confidence >= 0.98) {
      if (match.color == kBLUE) dColorString = "blue";
      else if (match.color == kRED) dColorString = "red";
      else if (match.color == kGREEN) dColorString = "green";
      else if (match.color == kYELLOW) dColorString = "yellow";
    }
    SmartDashboard.putNumber("Confidence", match.confidence);
      return dColorString;
  }

  /**
   * Returns the color the control panel sensor detects. 
   */
  public String getSensorColor() {
    String sColorString = "unkown";

    if (getDetectedColor() == "blue") sColorString = "red";
    else if (getDetectedColor() == "yellow") sColorString = "green";
    else if (getDetectedColor() == "red") sColorString = "blue";
    else if (getDetectedColor() == "green") sColorString = "yellow";
    return sColorString;
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
    SmartDashboard.putNumber("Revolutions", getRevolutions());
    SmartDashboard.putNumber("Motor Position", wheelSpinnerEncoder.getPosition());
    SmartDashboard.putString("Color String", getDetectedColor());
    SmartDashboard.putNumber("temperature", wheelSpinnerMotor.getMotorTemperature());
    SmartDashboard.putBoolean("Motor Is Fine",  motorIsFine());
    SmartDashboard.putNumber("Eighth Revolutions", eighthRevolutions);
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
   * Resets the revolution number.
   */
  public void resetRevolutions() {
    eighthRevolutions = 0.0; 
    totalRevolutions = 0.0;
  }
  
  /**
   * Returns the number of revolutions.
   */
  public double getRevolutions() {
    String initColor = getDetectedColor();
    totalRevolutions = ((eighthRevolutions) / 8); 
    
    if (lastColor != initColor && lastColor != "unknown") eighthRevolutions++;
    lastColor = initColor;

    if (totalRevolutions < 0) return 0;
    return totalRevolutions;
  }

  /**
   * Spins color wheel numer of times.
   */
  public void spinRevolutions(double times) {
    revolutions = getRevolutions() + 0.127;

    if (revolutions <= times) wheelSpinnerMotor.set(.2);
    else wheelSpinnerMotor.set(0);
  }

  /**
   * Tells you if motor is over 40 degrees Celcius. (for Vincent purposes).
   */
  public boolean motorIsFine() {
    return !(wheelSpinnerMotor.getMotorTemperature() > 45.0);
  }

  /**
   * Spins to a destination color based on the input color. 
   * @param toColor Desired color to spin to as detected by the control panel sensor.
   */
  public void spinToColor(String toColor) {
    if (getSensorColor() != toColor) {
      wheelSpinnerMotor.set(.15);
    } else wheelSpinnerMotor.set(0);
  }
}