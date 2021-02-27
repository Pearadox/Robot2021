// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Hood extends SubsystemBase {
  /** Creates a new hood. */
  private TalonSRX hoodMotor;

  public Hood() {
    hoodMotor = new TalonSRX(Constants.HoodConstants.HOOD_MOTOR_ID);
    // hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    // hoodMotor.configSelectedFeedbackCoefficient(1.0 / 1440);
  }

  public void setHoodSpeed(double speed) {
    hoodMotor.set(ControlMode.PercentOutput, speed);
  }

  public void hoodUp() {
    setHoodSpeed(0.6);
  }

  public void hoodDown() {
    setHoodSpeed(-0.6);
  }

  public void stopHood() {
    setHoodSpeed(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void dashboard() {
    
  }
}
