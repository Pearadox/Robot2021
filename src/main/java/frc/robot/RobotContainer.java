// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
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
  public static PowerDistributionPanel pdp = new PowerDistributionPanel();

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
    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    // m_Drivetrain.setDefaultCommand(
    //     // A split-stick arcade command, with forward/backward controlled by the left
    //     // hand, and turning controlled by the right.
    //     new RunCommand(
    //         () -> {
    //           double throttle = driverJoystick.getRawAxis(1);
    //           double twist = -driverJoystick.getRawAxis(2);
    //           if (Math.abs(throttle) <= 0.3) {
    //             throttle = 0;
    //           }
    //           if (Math.abs(twist) <= 0.3) {
    //             twist = 0;
    //           }
    //           m_Drivetrain.arcadeDrive(twist, throttle);
    //         }, m_Drivetrain));
            
    // m_Intake.setDefaultCommand(
    //   new RunCommand(
    //     () -> {
    //       m_Intake.RollerIn();
    //     }, m_Intake)
    // );
    // m_Shooter.setDefaultCommand(
    //   new (ShooterVoltage(m_Shooter, 4.3), m_Shooter);

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
   
    btn1.whileHeld(new RunCommand(m_Transport::LoadTransport, m_Transport)); //Tower Up
    btn1.whenReleased(new RunCommand(m_Transport::HopperIn, m_Transport));
    btn9.whileHeld(new RunCommand(  //ArmIntake Up
      () -> {
        m_Intake.setArmIntakeSpeed(0.27);
    }, m_Intake));
    /*btn10.whileHeld(new RunCommand( //ArmIntake Down
      () -> {
        m_Intake.setArmIntakeSpeed(-.1);
    }, m_Intake));
    */
    btn10.whenPressed(new IntakeDown(m_Intake));
     btn4.whenPressed(new InstantCommand(m_Intake::resetArmIntakeEncoder, m_Intake)); //Reset ArmIntake
     //btn5.whileHeld(new RunCommand(m_Intake::RollerIn, m_Intake)); //Roller In
    // btn6.whileHeld(new RunCommand(m_Intake::RollerOut, m_Intake)); // Roller Out
    //btn3.whileHeld(new DriveForward(m_Drivetrain));



    btn7.whileHeld(new RunCommand(m_Transport::TowerDown, m_Transport)); //Tower Down

   //btn5.whenPressed(new InstantCommand(m_Transport::resetBallCounter, m_Transport));
    
    //btn9.whileHeld(new RunCommand(m_Transport::TowerDown, m_Transport).withTimeout(0.4).andThen(new RunCommand(m_Transport::HopperOut, m_Transport)));
    //btn10.whileHeld(new HoodUp(m_Hood));
   // btn11.whileHeld(new RunCommand(m_Hood::hoodDown, m_Hood)).whenReleased(new RunCommand(m_Hood::stopHood, m_Hood));
    btn12.whenPressed(new SetZeroHood(m_Hood));
    btn3.whenPressed(new SetHood(m_Hood));

    //testing out trigger for ballTower with Robot state
    if (Robot.getState() == RobotState.TELEOP) {      
      new Trigger(
        () -> {
          return RobotContainer.m_Transport.getLow();
        })
        .whenActive(
                (new HopperInCmd(RobotContainer.m_Transport)).withTimeout(0.17)
                .andThen(new TowerUp(RobotContainer.m_Transport).withTimeout(.9)));
    }  
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

  public Joystick getDriverJoystick() {
    return driverJoystick;
  }
  private static void initDebugger(){
    if(DS.isFMSAttached()) {
      Debugger.setLevel(Debugger.warning4);
    } else {
      Debugger.setLevel(Debugger.info3);
    }
    Debugger.flagOn(_general); //Set all the flags on, comment out ones you want off
    Debugger.flagOn(_auton);
    Debugger.flagOn(_drive);
    Debugger.flagOn(_transport);
    Debugger.flagOn(_intake);
    Debugger.flagOn(_shooter);
    Debugger.flagOn(_tower);
    Debugger.flagOn(_climber);
    Debugger.flagOn(_visionLL);
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
