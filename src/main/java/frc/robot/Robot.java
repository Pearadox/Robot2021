// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.team2363.logger.HelixLogger;
import frc.robot.commands.HopperInCmd;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.commands.ResetArmandEncoder;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public static int brownOutCtn = 0;

  // Add Debug flags
  // You can have a flag for each subsystem, etc
  public static final String _controls = "CONTROL";
  public static final String _general = "GENERAL";
  public static final String _auton = "AUTON";
  public static final String _drive = "DRIVE";
  public static final String _transport = "TRANSPORT";
  public static final String _intake = "INTAKE";
  public static final String _shooter = "SHOOTER";
  public static final String _tower = "TOWER";
  public static final String _climber = "CLIMBER";
  public static final String _visionLL = "LIMELIGHT";
  public static final String _traverse = "TRAVERSE";
  
  public enum RobotState {
    DISABLED, AUTONOMOUS, TELEOP, TEST
  }

  public static RobotState s_robot_state = RobotState.DISABLED;

  public static RobotState getState() {
    return s_robot_state;
  }

  public static void setState(final RobotState state) {
    s_robot_state = state;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    CameraServer.getInstance().startAutomaticCapture();
    m_robotContainer = new RobotContainer();
    
    
  }
  
  @Override
  public void simulationPeriodic() {
    // Here we calculate the battery voltage based on drawn current.
    // As our robot draws more power from the battery its voltage drops.
    // The estimated voltage is highly dependent on the battery's internal
    // resistance.
    // double drawCurrent = RobotContainer.m_Drivetrain.getDrawnCurrentAmps();
    // double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(drawCurrent);
    // RoboRioSim.setVInVoltage(loadedVoltage);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    RobotContainer.visionLL.setLimeLightLED(true);
    setState(RobotState.DISABLED);
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    CommandScheduler.getInstance().cancelAll();
    RobotContainer.m_Drivetrain.resetOdometry(RobotContainer.m_Drivetrain.getPose());
    RobotContainer.visionLL.setLimeLightLED(true);
    RobotContainer.m_Transport.setBallCounter(3);
    
    setState(RobotState.AUTONOMOUS);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // HelixLogger.getInstance().saveLogs();}
  }
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // RobotContainer.m_Drivetrain.zeroHeading();
    CommandScheduler.getInstance().cancelAll();
    setState(RobotState.TELEOP);
    if (!RobotContainer.visionLL.getLEDState())
      RobotContainer.visionLL.setLimeLightLED(true);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    RobotContainer.m_Transport.setDefaultCommand(new HopperInCmd(RobotContainer.m_Transport));
    // new ResetArmandEncoder().withTimeout(1.0).schedule();
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    // HelixLogger.getInstance().saveLogs();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
