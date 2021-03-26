// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import org.opencv.video.BackgroundSubtractor;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.commands.HelixDrive;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import static frc.robot.Constants.DrivetrainConstants.*;

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

  private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
  private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(frontRightMotor, backRightMotor);

  private Encoder m_leftEncoder = new Encoder(15, 14,
            kLeftEncoderReversed);
  private Encoder m_rightEncoder = new Encoder(13, 12,
            kRightEncoderReversed);
  
  // The robot's drive
  // private final DifferentialDrive m_drive = new DifferentialDrive(leftMotors, rightMotors);

  // Create our gyro object like we would on a real robot.
  // private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
  
  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;
   
  
  // These are our EncoderSim objects, which we will only use in
  // simulation. However, you do not need to comment out these
  // declarations when you are deploying code to the roboRIO.
  private EncoderSim m_leftEncoderSim;
  private EncoderSim m_rightEncoderSim; 
  
  // The Field2d class shows the field in the sim GUI
  private Field2d m_fieldSim; 
  
  // Create the simulated gyro object, used for setting the gyro
  // angle. Like EncoderSim, this does not need to be commented out
  // when deploying code to the roboRIO.
  private ADXRS450_GyroSim m_gyroSim;

  // Create the simulation model of our drivetrain.
  public DifferentialDrivetrainSim m_drivetrainSimulator;

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    
  backLeftMotor.follow(frontLeftMotor);
  backRightMotor.follow(frontRightMotor);
  frontRightMotor.setInverted(true);
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    // m_leftEncoder.setDistancePerPulse(2 * Math.PI * Constants.DrivetrainConstants.WHEEL_DIAMETER / 2.0 / Constants.DrivetrainConstants.PULSES_PER_REVOLUTION);
    // m_rightEncoder.setDistancePerPulse(2 * Math.PI * Constants.DrivetrainConstants.WHEEL_DIAMETER / 2.0 / Constants.DrivetrainConstants.PULSES_PER_REVOLUTION);
    if (RobotBase.isSimulation()) {
    //Do the following for simulating a robot  
      m_drivetrainSimulator = new DifferentialDrivetrainSim(
        DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
        12.4,                    // 10:62 1st stage and 16:32 second stage = 12.4:1
        4.25,                     // (should be between 3-8 kg * m^2) mass = 60 kg, COM is x = .068, y = -.02. MOI = 60.0 * sqrt(.068^2 + .02^2) = 4.25?
        60.0,                    // The mass of the robot is 60 kg.
        Units.inchesToMeters(6), // The robot uses 6" radius wheels.
        Units.inchesToMeters(27),                  // The track width is 0.7112 meters. 24.937 in to inside wheel faces, 29.063 to outside. Average = 27
      
        // The standard deviations for measurement noise:
        // x and y:          0.001 m
        // heading:          0.001 rad
        // l and r velocity: 0.1   m/s
        // l and r position: 0.005 m
        VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
      }

      // The encoder and gyro angle sims let us set simulated sensor readings
      m_leftEncoderSim = new EncoderSim(m_leftEncoder);
      m_rightEncoderSim = new EncoderSim(m_rightEncoder);
      // m_gyroSim = new ADXRS450_GyroSim(m_gyro);

      // the Field2d class lets us visualize our robot in the simulation GUI.
      m_fieldSim = new Field2d();
      SmartDashboard.putData("Field", m_fieldSim);

      this.setDefaultCommand(new HelixDrive(this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Update odometry in the periodic block
    m_odometry.update(
      Rotation2d.fromDegrees(getHeading()),
      m_leftEncoder.getDistance(),
      m_rightEncoder.getDistance());
    m_fieldSim.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic() {
    
    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and gyro.
    // We negate the right side so that positive voltages make the right side
    // move forward.
    m_drivetrainSimulator.setInputs(
        leftMotors.get() * RobotController.getBatteryVoltage(),
        -rightMotors.get() * RobotController.getBatteryVoltage());
    m_drivetrainSimulator.update(0.020);

    m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
  }

  /**
   * Returns the current being drawn by the drivetrain. This works in SIMULATION ONLY! If you want
   * it to work elsewhere, use the code in {@link DifferentialDrivetrainSim#getCurrentDrawAmps()}
   *
   * @return The drawn current in Amps.
   */
  public double getDrawnCurrentAmps() {
    return m_drivetrainSimulator.getCurrentDrawAmps();
  }

  
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  
  public void dashboard() {
    SmartDashboard.putNumber("RightCurrent1", frontRightMotor.getOutputCurrent());
    SmartDashboard.putNumber("LeftCurrent1", frontLeftMotor.getOutputCurrent());
    SmartDashboard.putNumber("RightCurrent2", backRightMotor.getOutputCurrent());
    SmartDashboard.putNumber("LeftCurrent2", backLeftMotor.getOutputCurrent());
    
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    frontLeftMotor.set(fwd + rot);
    frontRightMotor.set(fwd - rot);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return 0;
    // return Math.IEEEremainder(m_gyro.getAngle(), 360) * (kGyroReversed ? -1.0 : 1.0);
  }
}
