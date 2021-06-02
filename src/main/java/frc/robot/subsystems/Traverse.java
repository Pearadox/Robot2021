// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TraverseConstants;

public class Traverse extends SubsystemBase {
  /** Creates a new Traverse. */
  
  private CANSparkMax transverseMotor;
  private CANEncoder transverseEncoder;

  public Traverse() {
    transverseMotor = new CANSparkMax(TraverseConstants.TRANSVERSE_CLIMB_MOTOR, MotorType.kBrushless);
    transverseMotor.setIdleMode(IdleMode.kBrake);
    transverseEncoder = transverseMotor.getEncoder();
    transverseEncoder.setPositionConversionFactor(42);
    transverseMotor.setOpenLoopRampRate(0.75);
  }

  public void setTransverseMotor(double setSpeed) {
    transverseMotor.set(setSpeed);
  }

  public double getTransverseRaw() { return transverseEncoder.getPosition();}

  public void stopTransverseMotor() { setTransverseMotor(0);}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
