// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.BACK_LEFT_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.BACK_RIGHT_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.FRONT_LEFT_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.FRONT_RIGHT_MOTOR;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
// import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
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

  public final CANEncoder m_leftEncoder = frontLeftMotor.getEncoder();  
  public final CANEncoder m_rightEncoder = frontRightMotor.getEncoder();

  public static AHRS navx;
  private final DifferentialDriveOdometry m_odometry;
  private Pose2d m_pose;
   
  // public static final double ksVolts = 0.23;
  // public static final double kvVoltSecondsPerMeter = 3.61;
  // public static final double kaVoltSecondsSquaredPerMeter = 0.529;

  // public static final double kpDriveVel = 2.04;

  // public static final double kMaxSpeedMetersPerSecond = 3;
  // public static final double kMaxAccelerationMetersPerSecondSquared = 3;

  // public static final double kRamseteB = 2;
  // public static final double kRamseteZeta = 0.7;

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
  
    m_leftEncoder.setPositionConversionFactor(Constants.DrivetrainConstants.DISTANCE_PER_REVOLUTION / Constants.DrivetrainConstants.GEAR_REDUCTION);
    m_rightEncoder.setPositionConversionFactor(Constants.DrivetrainConstants.DISTANCE_PER_REVOLUTION / Constants.DrivetrainConstants.GEAR_REDUCTION);
    encoderReset();

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-navx.getAngle()));
  
    this.setDefaultCommand(new HelixDrive(this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(Rotation2d.fromDegrees(-navx.getAngle()), -m_leftEncoder.getPosition(), -m_rightEncoder.getPosition());
    dashboard();
  }

  @Override
  public void simulationPeriodic() {
    
  }
  public void dashboard() {
    // SmartDashboard.putNumber("RightCurrent1", frontRightMotor.getOutputCurrent());
    // SmartDashboard.putNumber("LeftCurrent1", frontLeftMotor.getOutputCurrent());
    // SmartDashboard.putNumber("RightCurrent2", backRightMotor.getOutputCurrent());
    // SmartDashboard.putNumber("LeftCurrent2", backLeftMotor.getOutputCurrent());
    // SmartDashboard.putNumber("Gyro Heading", navx.getAngle());
    // SmartDashboard.putNumber("RightEncoderPostion", m_rightEncoder.getPosition());
    // SmartDashboard.putNumber("LeftEncoderPostion", m_leftEncoder.getPosition());

  }

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
  }

  public void stop() {
    frontLeftMotor.set(0);
    frontRightMotor.set(0);
  }

  public void zeroHeading() {
    navx.reset();   
  }
  public void encoderReset() {
    m_rightEncoder.setPosition(0.0);
    m_leftEncoder.setPosition(0.0);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(-m_leftEncoder.getVelocity()/60, -m_rightEncoder.getVelocity() / 60);
  }

  public void resetOdometry(Pose2d pose) {
    encoderReset();
    zeroHeading();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(-navx.getAngle()));
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    frontLeftMotor.setVoltage(-leftVolts);
    frontRightMotor.setVoltage(-rightVolts);
    m_drive.feed();
    // backRightMotor.setVoltage(-rightVolts);
    // backLeftMotor.setVoltage(leftVolts);

  }
  // public double getAverageEncoderDistance() {

  // }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  
  public double getHeading() {
    return navx.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -navx.getRate();
  }
}