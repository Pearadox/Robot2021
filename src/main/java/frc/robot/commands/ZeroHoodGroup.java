// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Hood;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZeroHoodGroup extends SequentialCommandGroup {
  /** Creates a new ZeroHoodGroup. */
  public ZeroHoodGroup(Hood m_hood) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // addCommands(new );
    addCommands(
      new SetZeroHood(m_hood, 1.0, false),
      new ReleaseHoodSwitch(),
      new SetZeroHood(m_hood, 0.2, true));
  }
}
