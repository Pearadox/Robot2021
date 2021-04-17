// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.VisionLL;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConfirmShot extends ParallelCommandGroup {
  /** Creates a new ConfirmShot. */
  public ConfirmShot(Shooter shooter,Hood hood,VisionLL visionLL, Drivetrain driveTrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new SetFlywheel_Hood(shooter, visionLL, hood)//, new VisionTurnToTarget(driveTrain, visionLL)
    );
  }
}
