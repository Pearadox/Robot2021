// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {
  /** Creates a new hood. */
  private TalonSRX hoodMotor;
  private DigitalInput hoodSwitch;

  public Hood() {
    hoodMotor = new TalonSRX(Constants.HoodConstants.HOOD_MOTOR_ID);
    hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    hoodMotor.configSelectedFeedbackCoefficient(1.0 / 1440);
    hoodSwitch = new DigitalInput(HoodConstants.HOOD_SWITCH_PORT);
  }

  public void setHoodSpeed(double speed) {
    hoodMotor.set(ControlMode.PercentOutput, speed);
    if (!SmartDashboard.containsKey("Hood Angle")) SmartDashboard.putNumber("Hood Angle", 0);
    if (!SmartDashboard.containsKey("Set Hood Angle")) SmartDashboard.putNumber("Set Hood Angle", 0);
    if (!SmartDashboard.containsKey("Hood Switch")) SmartDashboard.putBoolean("Hood Switch", false);
  }

  public void hoodUp() {
    setHoodSpeed(1);
  }

  public void hoodDown() {
    setHoodSpeed(-1);
  }

  private double getRawHoodAngle() {
    return hoodMotor.getSelectedSensorPosition() / 8618.5;  //8616.5 (MAGIC NUMBER)
  }

  // total teeth/pinion teeth
  public double getHoodAngle() {
    return getRawHoodAngle() * 404/20;
  }

  public void stopHood() {
    setHoodSpeed(0);
  }

  public boolean getHoodSwitch() {
    return hoodSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void dashboard() {
    SmartDashboard.putNumber("Hood Angle", getHoodAngle());
    SmartDashboard.putBoolean("Hood Switch", getHoodSwitch());
  }
}
