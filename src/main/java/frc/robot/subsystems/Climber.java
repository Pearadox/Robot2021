// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private TalonSRX climbMotor;
  private CANSparkMax transverseMotor;
  private CANEncoder transverseEncoder;
  private final static Climber INSTANCE = new Climber();
  public double kServoPos = 0.5;
  private Servo climbServo;
  public double BrakeEngaged = 1.0;
  public double BrakeDisengaged = 0.5;

  /**
   * Creates a new instance of this ClimberSubsystem.
   * This constructor is private since this class is a Singleton. External classes
   * should use the {@link #getInstance()} method to get the instance.
   */
  public Climber() {
    // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
    //       in the constructor or in the robot coordination class, such as RobotContainer.
    //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
    //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    climbMotor = new TalonSRX(ClimberConstants.CLIMB_MOTOR);
    climbServo = new Servo(9);
    // climbMotor.configOpenloopRamp(.25);
    
    transverseMotor = new CANSparkMax(ClimberConstants.TRANSVERSE_CLIMB_MOTOR, MotorType.kBrushless);
    transverseMotor.setIdleMode(IdleMode.kBrake);
    transverseEncoder = transverseMotor.getEncoder();
    transverseEncoder.setPositionConversionFactor(42);
    transverseMotor.setOpenLoopRampRate(0.75);
    
    if (!SmartDashboard.containsKey("ClimbVoltage")) {
      SmartDashboard.putNumber("ClimbVoltage", 0);
    }
    if (!SmartDashboard.containsKey("ServoPos")) {
      SmartDashboard.putNumber("ServoPos", kServoPos);
    }
  }

  public void setClimbMotor(double setSpeed) {
    climbMotor.set(ControlMode.PercentOutput, setSpeed);
  }

  public void setDisengageBrake() {
    climbServo.set(BrakeDisengaged);
  }

  public void setEngageBrake() {
    climbServo.set(BrakeEngaged);
  }

  public void stopClimbMotor() { setClimbMotor(0);}

  public double getClimbCurrent() {
    return climbMotor.getSupplyCurrent();
  }

  public void setTransverseMotor(double setSpeed) {
    transverseMotor.set(setSpeed);
  }

  public double getTransverseRaw() { return transverseEncoder.getPosition();}

  public void stopTransverseMotor() { setTransverseMotor(0);}

  /**
   * Returns the Singleton instance of this ClimberSubsystem. This static method
   * should be used -- {@code ClimberSubsystem.getInstance();} -- by external
   * classes, rather than the constructor to get the instance of this class.
   */

   @Override
   public void periodic() {
     
     
   }
  public static Climber getInstance() {
    return INSTANCE;
  }
  
  public void dashboard() {
    SmartDashboard.putNumber("ClimbVoltage", climbMotor.getBusVoltage());
     SmartDashboard.putNumber("ClimbCurrent", getClimbCurrent());
     double ServoPos = SmartDashboard.getNumber("ServoPos", kServoPos);
  }

}
