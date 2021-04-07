// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

// import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;
import frc.robot.commands.HelixDrive;
import edu.wpi.first.wpilibj.SPI;

import static frc.robot.Constants.DrivetrainConstants.*;
import com.kauailabs.navx.frc.AHRS;

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

  // private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
  // private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(frontRightMotor, backRightMotor);

  private final AHRS gyro;
   
  public static final double ksVolts = 0.235;
  public static final double kvVoltSecondsPerMeter = 3.6;
  public static final double kaVoltSecondsSquaredPerMeter = 0.521;

  public static final double kpDriveVel = 2.5;

  public static final double kMaxSpeedMetersPerSecond = 3;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;

  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    backLeftMotor.follow(frontLeftMotor);
    backRightMotor.follow(frontRightMotor);
    frontRightMotor.setInverted(true);
    gyro = new AHRS(SPI.Port.kMXP);
    gyro.reset();
  
  
    this.setDefaultCommand(new HelixDrive(this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    dashboard();
  }

  @Override
  public void simulationPeriodic() {
    
  }
  public void dashboard() {
    SmartDashboard.putNumber("RightCurrent1", frontRightMotor.getOutputCurrent());
    SmartDashboard.putNumber("LeftCurrent1", frontLeftMotor.getOutputCurrent());
    SmartDashboard.putNumber("RightCurrent2", backRightMotor.getOutputCurrent());
    SmartDashboard.putNumber("LeftCurrent2", backLeftMotor.getOutputCurrent());
    SmartDashboard.putNumber("Gyro Heading", gyro.getAngle());

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
}