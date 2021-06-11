// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Robot.RobotState;
import frc.robot.commands.*;
import frc.robot.subsystems.VisionLL.OperatorSettings;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SixBallBack extends SequentialCommandGroup {
  /** Creates a new SixBallBehindLine. */
  public SixBallBack() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
              new InstantCommand(
      () -> {
        RobotContainer.visionLL.setOperatorSettings(OperatorSettings.INITIATION);
      }, RobotContainer.visionLL),
              new ConfirmShotVision(RobotContainer.m_Drivetrain, RobotContainer.m_Hood, RobotContainer.m_Shooter, RobotContainer.visionLL )
                    .withTimeout(1.25),
              new HopperInTowerUpCmd()
                    .withTimeout(1),
              new InstantCommand(RobotContainer.m_Transport::HopperStop),
              //  new InstantCommand(RobotContainer.m_Transport::toggleAutoLoad),
              new InstantCommand(RobotContainer.m_Transport::resetBallCounter),
              new InstantCommand(
      () -> {
        RobotContainer.visionLL.setOperatorSettings(OperatorSettings.TRENCH);
      }, RobotContainer.visionLL),
              new AutonDriveSixBallBack(RobotContainer.m_Drivetrain)
                .alongWith( new SetOpFlywheel_Hood(RobotContainer.m_Shooter, RobotContainer.m_Hood)),
              new VisionTurnToTarget(RobotContainer.m_Drivetrain, RobotContainer.visionLL)
              .alongWith( new WaitCommand(1.25).andThen(
              new HopperInTowerUpCmd())));
  }
}
