// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
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
  private CANPIDController m_pidController;
  private CANEncoder rightCanEncoder;
  private CANEncoder leftCanEncoder;
  private double kP, kI, kD, kIz, kFF, ksetpoint, kMaxOutput, kMinOutput, maxRPM;
  private double lastError = 0;

  public Shooter() {
    rightFlywheelMotor = new PearadoxSparkMax(FlywheelConstants.RIGHT_FLY_MOTOR, MotorType.kBrushless, IdleMode.kCoast, 80, false);
    leftFlywheelMotor = new PearadoxSparkMax(FlywheelConstants.LEFT_FLY_MOTOR, MotorType.kBrushless, IdleMode.kCoast, 80, false);

    //follow function has a second parameter to indicate if it should be reversed in the follow
    leftFlywheelMotor.follow(rightFlywheelMotor, true);

    
    rightCanEncoder = rightFlywheelMotor.getEncoder();
    leftCanEncoder = leftFlywheelMotor.getEncoder();

    // rightFlywheelMotor.setOpenLoopRampRate(0.25);
    // leftFlywheelMotor.setOpenLoopRampRate(0.25);

    m_pidController = rightFlywheelMotor.getPIDController();
    rightCanEncoder.setVelocityConversionFactor(1.57289);
    leftCanEncoder.setVelocityConversionFactor(1.57289);
    SmartDashboard.putNumber("Flywheel RPM", 0);
    SmartDashboard.putNumber("Flywheel Voltage", 0);
    SmartDashboard.putNumber("Flywheel Output", 0);
    
    // PID coefficients
    //from testing 4/2/2021, good kp = .00027 and kf of .00013
    kP = 0.00022; //0.00075; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = .00013; //0.0215; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;
    ksetpoint = 0;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("S_P Gain", kP);
    SmartDashboard.putNumber("S_I Gain", kI);
    SmartDashboard.putNumber("S_D Gain", kD);
    SmartDashboard.putNumber("S_I Zone", kIz);
    SmartDashboard.putNumber("S_Feed Forward", kFF);
    SmartDashboard.putNumber("S_Max Output", kMaxOutput);
    SmartDashboard.putNumber("S_Min Output", kMinOutput);
    SmartDashboard.putNumber("S_SetPoint", ksetpoint);

    // this.setDefaultCommand(new ShooterVoltage(this, 4.3));
  }

  // public void setShooterSpeed(double speed) {
  //   rightFlywheelMotor.set(speed);
  //   leftFlywheelMotor.set(speed);
  // }

  public void setShooterVoltage(double voltage){
    m_pidController.setReference(ksetpoint, ControlType.kVelocity);
  }

  public void setShooterZone(double voltage) {
    m_pidController.setReference(voltage, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("S_P Gain", 0);
    double i = SmartDashboard.getNumber("S_I Gain", 0);
    double d = SmartDashboard.getNumber("S_D Gain", 0);
    double iz = SmartDashboard.getNumber("S_I Zone", 0);
    double ff = SmartDashboard.getNumber("S_Feed Forward", 0);
    double max = SmartDashboard.getNumber("S_Max Output", 0);
    double min = SmartDashboard.getNumber("S_Min Output", 0);
    double setpoint = SmartDashboard.getNumber("S_SetPoint", 0);


    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((setpoint != ksetpoint)) { ksetpoint=setpoint; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    // rightFlywheelMotor.setVoltage(4.2);
    // m_pidController.setReference(ksetpoint, ControlType.kVelocity);

    SmartDashboard.putNumber("S_RightProcessVariable", rightCanEncoder.getVelocity());
    SmartDashboard.putNumber("S_LeftProcessVariable", leftCanEncoder.getVelocity());
    
  }

  public void ShooterPID() {
    double currRPM = (rightCanEncoder.getVelocity() + leftCanEncoder.getVelocity())/2;
    double error = SmartDashboard.getNumber("S_SetPoint", 0) - currRPM;
    double errorSum = lastError + error;
    double P = kP * error;
    double I = errorSum * kI;
    double D = (lastError - error) * kD;
    lastError = error;
    double output = P + I - D;
    rightFlywheelMotor.setVoltage(output);
  }

  public double getFlySetPoint() {
    return ksetpoint;
  }

  public double getFlywheelRPM() {
    return (rightCanEncoder.getVelocity() + leftCanEncoder.getVelocity())/2;
  }

  public boolean isFlywheelInRange() {
    return getFlywheelRPM() < getFlySetPoint();
  }

  public void dashboard() {
    SmartDashboard.putNumber("Flywheel RPM", getFlywheelRPM());
    SmartDashboard.putNumber("Flywheel Voltage", rightFlywheelMotor.getBusVoltage());
    SmartDashboard.putNumber("Flywheel Output", rightFlywheelMotor.getAppliedOutput()); 
    SmartDashboard.putNumber("Right Current", rightFlywheelMotor.getOutputCurrent());
    SmartDashboard.putNumber("left Current", leftFlywheelMotor.getOutputCurrent());
  }
}


