// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private static final int deviceID = Constants.IntakeConstants.ARM_INTAKE_MOTOR;
  private CANSparkMax m_Intakemotor;
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, minVel, maxAcc, allowedErr;
  // public double maxRPM; //not used anywhere
  
  /** Creates a new Intake. */
  public Intake() {
    
    m_Intakemotor = new CANSparkMax(deviceID, MotorType.kBrushless);
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_Intakemotor = new CANSparkMax(deviceID, MotorType.kBrushless);
    m_Intakemotor.restoreFactoryDefaults();

    // initialze PID controller and encoder objects
    m_pidController = m_Intakemotor.getPIDController();
    m_encoder = m_Intakemotor.getEncoder();
    m_Intakemotor.setSmartCurrentLimit(30, 35);

    // PID coefficients
    kP = 5e-5; //0.002469(original 5e-5)
    kI = 1e-6; //0 (original 1e-6)
    kD = 0; 
    kIz = 0; 
    kFF = 0.000156; //0.096957 (original 0.000156)
    kMaxOutput = .4;  //approximate speeds used previously for manual control on 2020 robot
    kMinOutput = -.4; //approximate speeds used previously for manual control on 2020 robot
    // maxRPM = 5700; not used anywhere

    // Smart Motion Coefficients
    maxVel = 405; // rpm
    maxAcc = 202;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    /**
     * Smart Motion coefficients are set on a CANPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("Arm P Gain", kP);
    SmartDashboard.putNumber("Arm I Gain", kI);
    SmartDashboard.putNumber("Arm D Gain", kD);
    SmartDashboard.putNumber("Arm I Zone", kIz);
    SmartDashboard.putNumber("Arm Feed Forward", kFF);
    SmartDashboard.putNumber("Arm Max Output", kMaxOutput);
    SmartDashboard.putNumber("Arm Min Output", kMinOutput);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Arm Max Velocity", maxVel);
    SmartDashboard.putNumber("Arm Min Velocity", minVel);
    SmartDashboard.putNumber("Arm Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Arm Allowed Closed Loop Error", allowedErr);
    SmartDashboard.putNumber("Arm Set Position", 0);
    SmartDashboard.putNumber("Arm Set Velocity", 0);

    // button to toggle between velocity and smart motion modes
    SmartDashboard.putBoolean("Arm Mode", true);
  }

  public void resetEncoder() {
    m_encoder.setPosition(0.0);
  }
  public void setSpeed(double speed) {
    m_Intakemotor.set(speed);
  }

  @Override
  public void periodic() {

    double setPoint, processVariable;
    boolean mode = SmartDashboard.getBoolean("Arm Mode", false);
    if(mode) {
      setPoint = SmartDashboard.getNumber("Set Velocity", 0);
      m_pidController.setReference(setPoint, ControlType.kVelocity);
      processVariable = m_encoder.getVelocity();
    } else {
      setPoint = SmartDashboard.getNumber("Arm Set Position", 0);
      /**
       * As with other PID modes, Smart Motion is set by calling the
       * setReference method on an existing pid object and setting
       * the control type to kSmartMotion
       */
      m_pidController.setReference(setPoint, ControlType.kSmartMotion);
      processVariable = m_encoder.getPosition();
    }
    
    SmartDashboard.putNumber("Arm SetPoint", setPoint);
    SmartDashboard.putNumber("Arm Process Variable", processVariable);
  }
  public void dashboard() {
    // This method will be called once per scheduler run
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("Arm P Gain", 0);
    double i = SmartDashboard.getNumber("Arm I Gain", 0);
    double d = SmartDashboard.getNumber("Arm D Gain", 0);
    double iz = SmartDashboard.getNumber("Arm I Zone", 0);
    double ff = SmartDashboard.getNumber("Arm Feed Forward", 0);
    double max = SmartDashboard.getNumber("Arm Max Output", 0);
    double min = SmartDashboard.getNumber("Arm Min Output", 0);
    double maxV = SmartDashboard.getNumber("Arm Max Velocity", 0);
    double minV = SmartDashboard.getNumber("Arm Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("Arm Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Arm Allowed Closed Loop Error", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel)) { m_pidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { m_pidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { m_pidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { m_pidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }
    
    SmartDashboard.putNumber("Arm Output", m_Intakemotor.getAppliedOutput());
    SmartDashboard.putNumber("Arm Encoder",  m_encoder.getPosition());
    
  }
}
