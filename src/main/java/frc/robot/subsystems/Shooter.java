// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.FlywheelConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private PearadoxSparkMax rightFlywheelMotor;
  private PearadoxSparkMax leftFlywheelMotor;
  private CANEncoder rightCanEncoder;
  private CANEncoder leftCanEncoder;

  public Shooter() {
    rightFlywheelMotor = new PearadoxSparkMax(FlywheelConstants.RIGHT_FLY_MOTOR, MotorType.kBrushless, IdleMode.kBrake, 20, false);
    leftFlywheelMotor = new PearadoxSparkMax(FlywheelConstants.LEFT_FLY_MOTOR, MotorType.kBrushless, IdleMode.kBrake, 20, false);
    rightFlywheelMotor.setInverted(true);

    rightCanEncoder = rightFlywheelMotor.getEncoder();
    leftCanEncoder = leftFlywheelMotor.getEncoder();

    rightFlywheelMotor.setOpenLoopRampRate(0.25);
    leftFlywheelMotor.setOpenLoopRampRate(0.25);
    SmartDashboard.putNumber("Flywheel RPM", 0);
    SmartDashboard.putNumber("Flywheel Voltage", 0);
    SmartDashboard.putNumber("Flywheel Output", 0);
  }

  // public void setShooterSpeed(double speed) {
  //   rightFlywheelMotor.set(speed);
  //   leftFlywheelMotor.set(speed);
  // }
  public void setShooterVoltage(double voltage){
    rightFlywheelMotor.setVoltage(voltage);
    leftFlywheelMotor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void dashboard() {
    SmartDashboard.putNumber("Flywheel RPM", (rightCanEncoder.getVelocity() + leftCanEncoder.getVelocity())/2);
    SmartDashboard.putNumber("Flywheel Voltage", rightFlywheelMotor.getBusVoltage());
    SmartDashboard.putNumber("Flywheel Output", rightFlywheelMotor.getAppliedOutput()); 
  }
}
