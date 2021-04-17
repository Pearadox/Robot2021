// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.BACK_LEFT_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.BACK_RIGHT_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.FRONT_LEFT_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.FRONT_RIGHT_MOTOR;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
// import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Robot.RobotState;
import frc.robot.commands.HelixDrive;

public class Drivetrain extends SubsystemBase {

  // Real Robot stuff
  // These represent our regular encoder objects, which we would
  // create to use on a real robot.
  // The motors on the left side of the drive.

  //Need to change these to be CANSParkMax for real robot
  
  private final CANSparkMax frontLeftMotor = new CANSparkMax(FRONT_LEFT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax backLeftMotor = new CANSparkMax(BACK_LEFT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax frontRightMotor = new CANSparkMax(FRONT_RIGHT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax backRightMotor = new CANSparkMax(BACK_RIGHT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
  // public PowerDistributionPanel pdp = new PowerDistributionPanel();

  private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
  private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(frontRightMotor, backRightMotor);

  private final DifferentialDrive m_drive = new DifferentialDrive(leftMotors, rightMotors);

  public final CANEncoder leftDriveEncoder = frontLeftMotor.getEncoder();  
  public final CANEncoder rightDriveEncoder = frontRightMotor.getEncoder();

  public static AHRS navx;
  private final DifferentialDriveOdometry odometry;
  private final Field2d m_field = new Field2d();

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    frontRightMotor.restoreFactoryDefaults();
    backRightMotor.restoreFactoryDefaults();
    frontLeftMotor.restoreFactoryDefaults();
    backLeftMotor.restoreFactoryDefaults();

    frontRightMotor.setIdleMode(IdleMode.kBrake);
    frontLeftMotor.setIdleMode(IdleMode.kBrake);
    backRightMotor.setIdleMode(IdleMode.kCoast);
    backLeftMotor.setIdleMode(IdleMode.kCoast);

    backLeftMotor.follow(frontLeftMotor);
    backRightMotor.follow(frontRightMotor);
    frontRightMotor.setInverted(true);

    navx = new AHRS(SPI.Port.kMXP);
    navx.reset();
  
    leftDriveEncoder.setPositionConversionFactor(Constants.DrivetrainConstants.DISTANCE_PER_REVOLUTION / Constants.DrivetrainConstants.GEAR_REDUCTION);
    rightDriveEncoder.setPositionConversionFactor(Constants.DrivetrainConstants.DISTANCE_PER_REVOLUTION / Constants.DrivetrainConstants.GEAR_REDUCTION);
    encoderReset();

    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    SmartDashboard.putData("Field", m_field);
  
    this.setDefaultCommand(new HelixDrive(this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if(Robot.getState() == RobotState.AUTONOMOUS)
      odometry.update(Rotation2d.fromDegrees(getGyroAngle()), getLeftDrivePosition(), getRightEncoderPosition());
    
    dashboard();
  }

  public void dashboard() {
    m_field.setRobotPose(getPose());
    // SmartDashboard.putNumber("RightCurrent1", frontRightMotor.getOutputCurrent());
    // SmartDashboard.putNumber("LeftCurrent1", frontLeftMotor.getOutputCurrent());
    // SmartDashboard.putNumber("RightCurrent2", backRightMotor.getOutputCurrent());
    // SmartDashboard.putNumber("LeftCurrent2", backLeftMotor.getOutputCurrent());
    // SmartDashboard.putNumber("Gyro Heading", navx.getAngle());
    // SmartDashboard.putNumber("RightDriveEncoderPostion", rightDriveEncoder.getPosition());
    // SmartDashboard.putNumber("LeftDriveEncoderPostion", leftDriveEncoder.getPosition());
  }

  //Drivetrain methods

  public void arcadeDrive(double fwd, double rot) {
    
    if(RobotContainer.driverJoystick.getRawButton(11))
    {
      frontLeftMotor.set(0.2 * (fwd + rot));
      frontRightMotor.set(fwd - rot);
    }
    else if(RobotContainer.driverJoystick.getRawButton(12))
    {
      frontLeftMotor.set(fwd + rot);
      frontRightMotor.set(0.2 * (fwd - rot));
    }
    else {
      frontLeftMotor.set(fwd + rot);
      frontRightMotor.set(fwd - rot);
    }
    m_drive.feed();
  }

  public void HelixDrive() {
    double throttle = RobotContainer.driverJoystick.getY();
    double twist = RobotContainer.driverJoystick.getZ() * -0.65;

    double saturatedInput;
    double greaterInput = Math.max(Math.abs(twist), Math.abs(throttle));
    double lesserInput = Math.min(Math.abs(twist), Math.abs(throttle));

    if (greaterInput > 0.0) 
      saturatedInput = (lesserInput/greaterInput) + 1.0;
    else 
      saturatedInput = 1.0;

    throttle = throttle / saturatedInput;
    twist = twist/saturatedInput;
    if(Math.abs(throttle) < 0.1) 
      throttle = 0;
    if(Math.abs(twist) < 0.1) 
      twist = 0;
    
    arcadeDrive(throttle, twist);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    frontLeftMotor.setVoltage(-leftVolts);
    frontRightMotor.setVoltage(-rightVolts);
    m_drive.feed();
    // backRightMotor.setVoltage(-rightVolts);
    // backLeftMotor.setVoltage(leftVolts);
  }

  public void stop() {
    frontLeftMotor.set(0);
    frontRightMotor.set(0);
  }

  //Gyro/Odometry methods

  public void zeroHeading() {
    navx.reset();   
  }

  public void resetOdometry(Pose2d pose) {
    encoderReset();
    zeroHeading();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(-navx.getAngle()));
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(-leftDriveEncoder.getVelocity()/60, -rightDriveEncoder.getVelocity() / 60);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public double getHeading() {
    return navx.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -navx.getRate();
  }

  public double getGyroAngle() {
    return -navx.getAngle();
  }

  //Encoder Methods

  public void encoderReset() {
    rightDriveEncoder.setPosition(0.0);
    leftDriveEncoder.setPosition(0.0);
  }

  public double getLeftDrivePosition() {
    return -leftDriveEncoder.getPosition();
  }

  public double getRightEncoderPosition() {
    return -rightDriveEncoder.getPosition();
  }
}