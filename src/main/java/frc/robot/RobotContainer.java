// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.lib.drivers.EForwardableConnections;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RamseteConstants;
import frc.robot.Robot.RobotState;
import frc.robot.commands.*;
import frc.robot.commands.autocommands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.VisionLL.OperatorSettings;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.lib.util.TrajectoryCache;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Joystick driverJoystick = new Joystick(0);
  public static final Joystick operatorJoystick = new Joystick(1);
  public static final Drivetrain m_Drivetrain = new Drivetrain();
  public static final Hood m_Hood = new Hood();
  public static final Intake m_Intake = new Intake();
  public static final Shooter m_Shooter = new Shooter();
  public static final TransportSystem m_Transport = new TransportSystem();
  public static final VisionLL visionLL = new VisionLL();
  public static final Climber m_Climber = new Climber();
  

  public static DriverStation DS;
  public SendableChooser<String> pathSelector;

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

  public static final InstantCommand OPTRIANGLE = new InstantCommand(
      () -> {
        visionLL.setOperatorSettings(OperatorSettings.TRIANGLE);
      }, visionLL);

  public static final InstantCommand OPINITIATION = new InstantCommand(
      () -> {
        visionLL.setOperatorSettings(OperatorSettings.INITIATION);
      }, visionLL);
    
  public static final InstantCommand OPTRENCH = new InstantCommand(
      () -> {
        visionLL.setOperatorSettings(OperatorSettings.TRENCH);
      }, visionLL);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    DS = DriverStation.getInstance();
    pathSelector = new SendableChooser<>();
    portForwarding();
    Dashboard.intializeDashboard();

    // Configure the button bindings
    configureButtonBindings();
    loadTrajectoryPaths();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  JoystickButton btn1 = new JoystickButton(driverJoystick, 1);
  JoystickButton btn2 = new JoystickButton(driverJoystick, 2);
  JoystickButton btn3 = new JoystickButton(driverJoystick, 3);
  JoystickButton btn4 = new JoystickButton(driverJoystick, 4);
  JoystickButton btn5 = new JoystickButton(driverJoystick, 5);
  JoystickButton btn6 = new JoystickButton(driverJoystick, 6);
  JoystickButton btn7 = new JoystickButton(driverJoystick, 7);
  JoystickButton btn8 = new JoystickButton(driverJoystick, 8);
  JoystickButton btn9 = new JoystickButton(driverJoystick, 9);
  JoystickButton btn10 = new JoystickButton(driverJoystick, 10);
  JoystickButton btn11 = new JoystickButton(driverJoystick, 11);
  JoystickButton btn12 = new JoystickButton(driverJoystick, 12);

  JoystickButton opbtn1 = new JoystickButton(operatorJoystick, 1);
  JoystickButton opbtn2 = new JoystickButton(operatorJoystick, 2);
  JoystickButton opbtn3 = new JoystickButton(operatorJoystick, 3);
  JoystickButton opbtn4 = new JoystickButton(operatorJoystick, 4);
  JoystickButton opbtn5 = new JoystickButton(operatorJoystick, 5);
  JoystickButton opbtn6 = new JoystickButton(operatorJoystick, 6);
  JoystickButton opbtn7 = new JoystickButton(operatorJoystick, 7);
  JoystickButton opbtn8 = new JoystickButton(operatorJoystick, 8);
  JoystickButton opbtn9 = new JoystickButton(operatorJoystick, 9);
  JoystickButton opbtn10 = new JoystickButton(operatorJoystick, 10);
  JoystickButton opbtn11 = new JoystickButton(operatorJoystick, 11);
  JoystickButton opbtn12 = new JoystickButton(operatorJoystick, 12);

//  Transport command - hopper in
//  Transport command - hopper out
//  Transport command  - TowerUp'
//  Transport command  - TowerDown
//  Transport command  - LoadTransport
//  setZone wall (green)
//  setZone initiation (yellow)
//  setZone trench (blue)
//  setZone maybe controll panel (red)
//  Intake ArmUp
//  Intake ArmDown
//  Intake RollerIn
//  Intake RollerOut
//  Intake ArmUp + RollerIn
//  Intake ArmDown + RollerIn
//  Hood HoodAngleZero
//  Hood setAngle
//  Hood HoodUp
//  Hood HoodDown
//  DriveTrain 90 degree turn left
//  DriveTrain 90 degree turn right
//  Climber Release
//  Climber Down '
//  Climber Up
//  Climber TraverseUp
//  Climber TraverseDown
  private void configureButtonBindings() {
    // BUTTONS 11 and 12 ARE USED FOR HELIX DRIVE TURNS
    //Shooter items
    btn1.whileHeld(new HopperInTowerUpCmd())
        .whenReleased(new RunCommand(m_Transport::HopperInOnly, m_Transport));
    
    btn2.whileHeld(new VisionDriveToTarget(m_Drivetrain, visionLL));
    //btn3.whenPressed(new SetHood(m_Hood));
    // btn4.whenPressed(new SetZoneFlywheel_Hood(m_Shooter, visionLL, m_Hood));
    btn5.whenPressed(new SetZeroHood(m_Hood).withTimeout(4));
    // btn6.whileHeld(new RunCommand(m_Transport::LoadTransport, m_Transport))
    //     .whenReleased(new RunCommand(m_Transport::StopTransportSystem, m_Transport));
     //Tower Down/Outake
    btn3.whileHeld(new Outake_balls());
    btn4.whenPressed(new RunCommand(m_Transport::resetBallCounter));
    btn7.whenPressed(new ArmSmartMotionUp());
    btn9.whenPressed(new ArmSmartMotionLoading());
    //btn11.whenPressed(new ArmSmartMotionDown());
    btn11.whenPressed(new ResetArmandEncoder());
    btn8.whenPressed(OPTRIANGLE
        .andThen(new SetOpFlywheel_Hood(m_Shooter, m_Hood)));
    btn10.whenPressed(OPINITIATION
        .andThen(new SetOpFlywheel_Hood(m_Shooter, m_Hood)));
    btn12.whenPressed(OPTRENCH
        .andThen(new SetOpFlywheel_Hood(m_Shooter, m_Hood)));
    //intake items
    // btn9.whileHeld(new RunCommand(  //ArmIntake Up
    //   () -> {
    //     m_Intake.setArmIntakeSpeed(1);
    //     m_Intake.setRollerSpeed(0);
    // }, m_Intake))
    //     .whenReleased(new InstantCommand(m_Intake::stopArmIntake, m_Intake));
   

    // btn10.whenPressed(new IntakeDown(m_Intake));

    //Operator Buttons
    
    opbtn3.whileHeld(new ClimbUpProcess(m_Climber));
    //opbtn4.whileHeld(new ClimbRelease(m_Climber).withTimeout(2));
    opbtn2.whileHeld(new HangClimb(m_Climber));
    opbtn5.whileHeld(new TraverseLeft(m_Climber));
    opbtn6.whileHeld(new TraverseRight(m_Climber));
    
    opbtn7.whenPressed(new ArmSmartMotionUp());
    opbtn9.whenPressed(new ArmSmartMotionLoading());
    //btn11.whenPressed(new ArmSmartMotionDown());
    opbtn11.whenPressed(new ResetArmandEncoder());
    // opbtn8.whenPressed(OPTRIANGLE
    //            .andThen(new SetOpFlywheel_Hood(m_Shooter, m_Hood)));
    // opbtn10.whenPressed(OPINITIATION
    //               .andThen(new SetOpFlywheel_Hood(m_Shooter, m_Hood)));
    // opbtn12.whenPressed(OPTRENCH
    //             .andThen(new SetOpFlywheel_Hood(m_Shooter, m_Hood)));

    // new Trigger(
    //   () -> {
    //     return !(m_Hood.gethasHoodZeroed()) && Robot.getState() == Robot.RobotState.TELEOP && !(m_Shooter.isFlywheelInRange());
    //   })
    //   .whileActiveContinuous(
    //     (new SetZeroHood(m_Hood))
    //     .andThen(OPINITIATION
    //       .andThen(new SetOpFlywheel_Hood(m_Shooter, m_Hood))), false);
    new Trigger(
      () -> {
        return !(m_Hood.gethasHoodZeroed()) && Robot.getState() == Robot.RobotState.TELEOP && !(m_Shooter.isFlywheelInRange());
      })
      .whileActiveContinuous(
        (new SetZeroHood(m_Hood))
        .andThen(new InstantCommand(
          () -> {
            visionLL.setOperatorSettings(OperatorSettings.INITIATION);
          }, visionLL)
          .andThen(new SetOpFlywheel_Hood(m_Shooter, m_Hood))), false);

    //testing out trigger for ballTower with Robot state
    new Trigger(
      () -> {
        return RobotContainer.m_Transport.getLow() && (Robot.getState() == RobotState.TELEOP);
      })
      .whenActive(
              (new liftTowerOne(RobotContainer.m_Transport)).withTimeout(1)
              .andThen(new InstantCommand(m_Transport::incrementBallCounter, m_Transport)), false);
              // (new HopperInCmd(RobotContainer.m_Transport)).withTimeout(0.17)
              // .andThen(new TowerUp(RobotContainer.m_Transport).withTimeout(1.3))
              // .andThen(new InstantCommand(m_Transport::incrementBallCounter, m_Transport)), false);
  }

  public void loadTrajectoryPaths() {
    TrajectoryCache.clear();
    // sendCacheTrajectory("Slalom", "output/SlalomPath");
    // sendCacheTrajectory("Straight2m", "output/Straight2m");
    sendCacheTrajectory("Turn", "output/Turn");
    sendCacheTrajectory("Bounce0", "output/Bounce0");
    sendCacheTrajectory("Bounce1", "output/Bounce1");
    // sendCacheTrajectory("Bounce2", "output/Bounce2");
    // sendCacheTrajectory("Bounce3", "output/Bounce3");
    // sendCacheTrajectory("BarrelRacing", "output/BarrelRacing");
    // sendCacheTrajectory("Straight", "output/Straight");

    sendCacheTrajectory("SixBallFrontBackwards", "output/SixBallFrontBackwards");
    sendCacheTrajectory("SixBallBackwards", "output/SixBallBackwards");
    sendCacheTrajectory("SixBallForwards", "output/SixBallForwards");
    sendCacheTrajectory("FiveBallBackwards", "output/FiveBallBackwards");
    sendCacheTrajectory("FiveBallForward", "output/FiveBallForward");

    SmartDashboard.putData("Path Selection", pathSelector);
  }

  private void sendCacheTrajectory(String key, String jsonPath) {
    pathSelector.addOption(key, key);
    TrajectoryCache.add(key, jsonPath);
  }

  private void portForwarding() {
    EForwardableConnections.addPortForwarding(EForwardableConnections.LIMELIGHT_CAMERA_FEED);
    EForwardableConnections.addPortForwarding(EForwardableConnections.LIMELIGHT_WEB_VIEW);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return new PrintCommand("Auton Command");
    // return printMessage("Auton Command");
    if(pathSelector.getSelected().equals("Bounce0"))
      return new BouncePath(m_Drivetrain);
    else if (pathSelector.getSelected().equals("FiveBallBackwards"))
      return new AutonDriveFiveBallBack(m_Drivetrain);
    else if (pathSelector.getSelected().equals("SixBallBackwards"))
      return new SixBallBack();
    else if (pathSelector.getSelected().equals("SixBallFrontBackwards"))
      return new AutonDriveSixBallFront(m_Drivetrain);
    Trajectory pathTrajectory = TrajectoryCache.get(pathSelector.getSelected());
    RamseteCommand ramseteCommand = createRamseteCommand(pathTrajectory);
    // Reset odometry to the starting pose of the trajectory.
    m_Drivetrain.resetOdometry(pathTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_Drivetrain.tankDriveVolts(0, 0));
  }

  public RamseteCommand createRamseteCommand(Trajectory pathTrajectory) {
    return new RamseteCommand(
      pathTrajectory,
      m_Drivetrain::getPose,
      new RamseteController(RamseteConstants.B, RamseteConstants.ZETA),
      new SimpleMotorFeedforward(RamseteConstants.ksVolts,
                                 RamseteConstants.kvVoltSecondsPerMeter,
                                 RamseteConstants.kaVoltSecondsSquaredPerMeter),
      DrivetrainConstants.KINEMATICS,
      m_Drivetrain::getWheelSpeeds,
      new PIDController(RamseteConstants.kPDriveVel, 0, 0),
      new PIDController(RamseteConstants.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      m_Drivetrain::tankDriveVolts,
      m_Drivetrain
    );

  }

  public static Joystick getDriverJoystick() {
    return driverJoystick;
  }
}
