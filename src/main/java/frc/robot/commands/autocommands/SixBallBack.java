// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SixBallBack extends SequentialCommandGroup {
  /** Creates a new SixBallBehindLine. */
  public SixBallBack() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
              new SetZoneFlywheel_Hood(RobotContainer.m_Shooter, RobotContainer.visionLL, RobotContainer.m_Hood).withTimeout(2.5),
               ((new HopperInTowerUpCmd()).withTimeout(5.0)));
              //  ((new InstantCommand( () -> { RobotContainer.m_Intake.setRollerSpeed(.7); }, RobotContainer.m_Intake))),
              //  (new AutonDriveSixBallBack(RobotContainer.m_Drivetrain)),
              //  (new ConfirmShotVision(RobotContainer.m_Drivetrain, RobotContainer.m_Hood, RobotContainer.m_Shooter, RobotContainer.visionLL)),
              //  (new HopperInTowerUpCmd()));
  }
}
