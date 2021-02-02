// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class HopperWiggleCG {

    public static Command wiggle = new SequentialCommandGroup(
        new RunCommand(() -> RobotContainer.m_Transport.HopperIn(), RobotContainer.m_Transport).withTimeout(5.0),
        new RunCommand(() -> RobotContainer.m_Transport.HopperOut(), RobotContainer.m_Transport).withTimeout(0.5)
    );

    public static Command sendHopperCommand() {
        return wiggle;
    }

	public class wiggle {
	}
}
