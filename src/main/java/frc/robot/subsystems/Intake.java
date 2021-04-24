// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private static final int kArmIntakeMotor = Constants.IntakeConstants.ARM_INTAKE_MOTOR;
  private static final int kTopRollerMotor = Constants.IntakeConstants.TOP_ROLLER_MOTOR;
  private static final int kBotRollerMotor = Constants.IntakeConstants.BOT_ROLLER_MOTOR;

  private CANSparkMax ArmIntakeMotor;
  private PearadoxSparkMax TopRollerMotor;
  private PearadoxSparkMax BotRollerMotor;

  private CANEncoder TopRollerEncoder;

  public CANPIDController ArmPidController;
  private CANEncoder ArmIntakeEncoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, minVel, maxAcc, allowedErr;

  public double in_speed = 1;
  public double out_speed = -0.3;
  public double IntakeDownEncoderValue = -30;
  // public double maxRPM; //not used anywhere

  /** Creates a new Intake. */
  public Intake() {
    ArmIntakeMotor = new CANSparkMax(kArmIntakeMotor, MotorType.kBrushless);
    TopRollerMotor = new PearadoxSparkMax(kTopRollerMotor);
    BotRollerMotor = new PearadoxSparkMax(kBotRollerMotor);
    // BotRollerMotor.setInverted(true);

    TopRollerEncoder = TopRollerMotor.getEncoder();
    resetRollerIntakeEncoder();

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    ArmIntakeMotor.restoreFactoryDefaults();

    // initialze PID controller and encoder objects
    ArmPidController = ArmIntakeMotor.getPIDController();
    ArmIntakeEncoder = ArmIntakeMotor.getEncoder();
    ArmIntakeMotor.setSmartCurrentLimit(20, 30);
    ArmIntakeMotor.setInverted(true);

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
    ArmPidController.setP(kP);
    ArmPidController.setI(kI);
    ArmPidController.setD(kD);
    ArmPidController.setIZone(kIz);
    ArmPidController.setFF(kFF);
    ArmPidController.setOutputRange(kMinOutput, kMaxOutput);

    
    //3this.setDefaultCommand(new RunCommand( () -> { this.setRollerSpeed(.7); }, this));

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
    ArmPidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    ArmPidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    ArmPidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    ArmPidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // display PID coefficients on SmartDashboard
    // SmartDashboard.putNumber("Arm P Gain", kP);
    // SmartDashboard.putNumber("Arm I Gain", kI);
    // SmartDashboard.putNumber("Arm D Gain", kD);
    // SmartDashboard.putNumber("Arm I Zone", kIz);
    // SmartDashboard.putNumber("Arm Feed Forward", kFF);
    // SmartDashboard.putNumber("Arm Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Arm Min Output", kMinOutput);

    // // display Smart Motion coefficients
    // SmartDashboard.putNumber("Arm Max Velocity", maxVel);
    // SmartDashboard.putNumber("Arm Min Velocity", minVel);
    // SmartDashboard.putNumber("Arm Max Acceleration", maxAcc);
    // SmartDashboard.putNumber("Arm Allowed Closed Loop Error", allowedErr);
    // SmartDashboard.putNumber("Arm Set Position", 0);
    // SmartDashboard.putNumber("Arm Set Velocity", 0);

    // // button to toggle between velocity and smart motion modes
    // SmartDashboard.putBoolean("Arm Mode", true);

    // SmartDashboard.putNumber("Roller in Speed", in_speed);
    // SmartDashboard.putNumber("Roller out Speed", out_speed);
    // SmartDashboard.putNumber("Roller RPM", 0);
    // SmartDashboard.putNumber("Arm Current", 0);
    // SmartDashboard.putNumber("Arm Voltage", 0);
    // SmartDashboard.putNumber("IntakeDown value", IntakeDownEncoderValue);
  }

  public void resetArmIntakeEncoder() {
     ArmIntakeEncoder.setPosition(0.0);
  }

  public void resetRollerIntakeEncoder() {
    TopRollerEncoder.setPosition(0.0);
  }
  
  public void setArmIntakeSpeed(double speed) {
    ArmIntakeMotor.set(speed);
  }

  public void setRollerSpeed(double speed) {
    TopRollerMotor.set(speed);
    BotRollerMotor.set(-speed);
  }

  public void RollerIn() {
    setRollerSpeed(in_speed);
  }

  public void RollerOut() {
    setRollerSpeed(out_speed);
  }

  public void setIntakeDownPos(double value) {
    IntakeDownEncoderValue = value;
  }

  public double getIntakeDownPos() {
    return IntakeDownEncoderValue;
  }

  public CANEncoder getArmIntakeEncoder() {
    return ArmIntakeEncoder;
  }

  public double getArmIntakePosition(){
    return ArmIntakeEncoder.getPosition();
  }
  public double getIntakeCurrent(){
    return ArmIntakeMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
  }

  public void dashboard() {
    // This method will be called once per scheduler run
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("Arm P Gain", kP);
    double i = SmartDashboard.getNumber("Arm I Gain", kI);
    double d = SmartDashboard.getNumber("Arm D Gain", kD);
    double iz = SmartDashboard.getNumber("Arm I Zone", kIz);
    double ff = SmartDashboard.getNumber("Arm Feed Forward", kFF);
    double max = SmartDashboard.getNumber("Arm Max Output", kMaxOutput);
    double min = SmartDashboard.getNumber("Arm Min Output", kMinOutput);
    double maxV = SmartDashboard.getNumber("Arm Max Velocity", maxVel);
    double minV = SmartDashboard.getNumber("Arm Min Velocity", minVel);
    double maxA = SmartDashboard.getNumber("Arm Max Acceleration", maxVel);
    double allE = SmartDashboard.getNumber("Arm Allowed Closed Loop Error", allowedErr);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { ArmPidController.setP(p); kP = p; }
    if((i != kI)) { ArmPidController.setI(i); kI = i; }
    if((d != kD)) { ArmPidController.setD(d); kD = d; }
    if((iz != kIz)) { ArmPidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { ArmPidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      ArmPidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel)) { ArmPidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { ArmPidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { ArmPidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { ArmPidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }
    
    // SmartDashboard.putNumber("Arm Output", ArmIntakeMotor.getAppliedOutput());
    // SmartDashboard.putNumber("Arm Encoder",  ArmIntakeEncoder.getPosition());
    // SmartDashboard.putNumber("Arm Current", ArmIntakeMotor.getOutputCurrent());
    // SmartDashboard.putNumber("Arm Voltage", ArmIntakeMotor.getBusVoltage());
    // SmartDashboard.putNumber("IntakeDown value", IntakeDownEncoderValue);
    // SmartDashboard.putNumber("Intake Temp", ArmIntakeMotor.getMotorTemperature());
    //change roller speeds based on SmartDashboard values
    final double in = SmartDashboard.getNumber("Roller in Speed", in_speed);
    final double out = SmartDashboard.getNumber("Roller out Speed", out_speed);

    if((in_speed != in)) in_speed = in;
    if((out_speed != out)) out_speed = out;

    // SmartDashboard.putNumber("Roller RPM", TopRollerEncoder.getVelocity());
  }
}