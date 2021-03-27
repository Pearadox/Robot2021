// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.EForwardableConnections;
import frc.lib.util.Debugger;
import frc.robot.Robot.RobotState;
import frc.robot.commands.*;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TransportSystem;
import frc.robot.subsystems.VisionLL;
import frc.robot.subsystems.Hood;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Joystick driverJoystick = new Joystick(0);
  public static final Climber m_Climber = new Climber();
  public static final Drivetrain m_Drivetrain = new Drivetrain();
  public static final Hood m_Hood = new Hood();
  public static final Intake m_Intake = new Intake();
  public static final Shooter m_Shooter = new Shooter();
  public static final TransportSystem m_Transport = new TransportSystem();
  public static final VisionLL visionLL = new VisionLL();
  

  public static DriverStation DS;
  // public static PowerDistributionPanel pdp = new PowerDistributionPanel();

  // public static PearadoPreferences prefs = PearadoxPreferences.getInstance();


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

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    DS = DriverStation.getInstance();
    portForwarding();
    initDebugger(); // Init Debugger
    //HelixEvents.getInstance().startLogging();
    printInfo("Start robotInit()");
    Dashboard.intializeDashboard();

    // Configure the button bindings
    configureButtonBindings();

    printInfo("End robotInit()");
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

  private void configureButtonBindings() {

    //Shooter items
   
    //btn1.whileHeld(new HopperInTowerUpCmd()); //Tower Up
    btn1.whenReleased(new RunCommand(m_Transport::HopperInOnly, m_Transport));
    btn3.whenPressed(new SetHood(m_Hood));
    btn5.whenPressed(new SetZeroHood(m_Hood));
    btn8.whenPressed(new ShooterVoltage(m_Shooter, SmartDashboard.getNumber("S_Setpoint", 0)));
    btn2.whileHeld(new VisionDriveToTarget(m_Drivetrain, visionLL));
    btn9.whileHeld(new VisionTurnToTarget(m_Drivetrain, visionLL));  

    //intake items
    // btn9.whileHeld(new RunCommand(  //ArmIntake Up
    //   () -> {
    //     m_Intake.setArmIntakeSpeed(1);
    //     m_Intake.setRollerSpeed(0);
    // }, m_Intake));

    btn10.whenPressed(new IntakeDown(m_Intake));
    btn4.whenPressed(new InstantCommand(m_Intake::resetArmIntakeEncoder, m_Intake)); //Reset ArmIntake
    //btn5.whileHeld(new RunCommand(m_Intake::RollerIn, m_Intake)); //Roller In
    // btn6.whileHeld(new RunCommand(m_Intake::RollerOut, m_Intake)); // Roller Out
    //btn3.whileHeld(new DriveForward(m_Drivetrain));


    //outtake 
    btn7.whileHeld(new Outake_balls()); //Tower Down


    //testing out trigger for ballTower with Robot state
    new Trigger(
      () -> {
        return RobotContainer.m_Transport.getLow() && (Robot.getState() == RobotState.TELEOP);
      })
      .whenActive(
              (new HopperInCmd(RobotContainer.m_Transport)).withTimeout(0.17)
              .andThen(new TowerUp(RobotContainer.m_Transport).withTimeout(0.9))
              .andThen(new InstantCommand(m_Transport::incrementBallCounter, m_Transport)), false);
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
  
    return new ThreeBallAuton();
  }

  public static Joystick getDriverJoystick() {
    return driverJoystick;
  }
  private static void initDebugger(){
    if(DS.isFMSAttached()) {
      Debugger.setLevel(Debugger.warning4);
    } else {
      Debugger.setLevel(Debugger.info3);
    }
    // Debugger.flagOn(_general); //Set all the flags on, comment out ones you want off
    // Debugger.flagOn(_auton);
    // Debugger.flagOn(_drive);
    // Debugger.flagOn(_transport);
    // Debugger.flagOn(_intake);
    // Debugger.flagOn(_shooter);
    // Debugger.flagOn(_tower);
    // Debugger.flagOn(_climber);
    // Debugger.flagOn(_visionLL);
  }

  public static void printDebug(String msg){
    Debugger.println(msg, _general, Debugger.debug2);
  }
  
  public static void printInfo(String msg){
    Debugger.println(msg, _general, Debugger.info3);
  }
  
  public static void printWarning(String msg) {
    Debugger.println(msg, _general, Debugger.warning4);
  }
}
