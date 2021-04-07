// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

// import org.graalvm.compiler.core.common.type.ArithmeticOpTable.UnaryOp.Abs;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {
  /** Creates a new hood. */
  private TalonSRX hoodMotor;
  private DigitalInput hoodSwitch;
  private double setPoint;
  private double kMaxOutput = 1;
  private double kMinOutput = -1;
  private double THRESHOLD = 3;
  private double kP = 1/ THRESHOLD;
  public double kMinError = 0.5;
  private boolean hasHoodZeroed = false;


  public Hood() {
    hoodMotor = new TalonSRX(Constants.HoodConstants.HOOD_MOTOR_ID);
    hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    hoodMotor.configSelectedFeedbackCoefficient(1);
    hoodSwitch = new DigitalInput(HoodConstants.HOOD_SWITCH_PORT);
    setPoint = 0;
  }

  public void setHoodSpeed(double speed) {
    hoodMotor.set(ControlMode.PercentOutput, speed);
    if (!SmartDashboard.containsKey("Hood Angle")) SmartDashboard.putNumber("Hood Angle", 0);
    if (!SmartDashboard.containsKey("Set Hood Angle")) SmartDashboard.putNumber("Set Hood Angle", 0);
    if (!SmartDashboard.containsKey("Hood Error")) SmartDashboard.putNumber("Hood Error", 0);
    if (!SmartDashboard.containsKey("Hood Switch")) SmartDashboard.putBoolean("Hood Switch", false);
    if (!SmartDashboard.containsKey("Hood RPM")) SmartDashboard.putNumber("Hood RPM", 0);
  }

  public void hoodUp() {
    setHoodSpeed(1);
  }
  public boolean gethasHoodZeroed() {
    return hasHoodZeroed;
  }
  public void sethasHoodZeroed(boolean value) {
    hasHoodZeroed = value;
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
    return !hoodSwitch.get();
  }

  public void setHoodPoint(double target){
    setPoint = target;
  }

  public double getHoodSetPoint() {
    return setPoint;
  }

  public void zeroHood() {
    hoodMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void dashboard() {
    if (getHoodAngle() < 1) {
      SmartDashboard.putNumber("Hood Angle", 0);
    } else {
      SmartDashboard.putNumber("Hood Angle", getHoodAngle());
    }
    SmartDashboard.putBoolean("Hood Switch", getHoodSwitch());
    SmartDashboard.putBoolean("hasHoodZeroed", gethasHoodZeroed());
    setHoodPoint(SmartDashboard.getNumber("Set Hood Angle", 0));
    SmartDashboard.putNumber("Hood RPM", hoodMotor.getMotorOutputVoltage());
  }

  public void setHoodAngle(double setPointAngle) {
    double currAngle = SmartDashboard.getNumber("Hood Angle", 0);
    double error = setPointAngle - currAngle;
    SmartDashboard.putNumber("Hood Error", error);
    double output = error * kP;
      if (output > kMaxOutput) {
        output = kMaxOutput;
      } 
      if (output < kMinOutput) {
        output = kMinOutput;
      }
      if(Math.abs(output) < 0.3)
      {
        output = Math.signum(output) * 0.3;
      }
      setHoodSpeed(output);
  }
}