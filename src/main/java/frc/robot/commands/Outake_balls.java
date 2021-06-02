// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Outake_balls extends CommandBase {
  /** Creates a new Outake_balls. */
  public Outake_balls() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Intake, RobotContainer.m_Transport);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_Intake.RollerOut();
    //
    RobotContainer.m_Transport.TowerDown();
    RobotContainer.m_Transport.HopperOut();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Intake.setRollerSpeed(0);
    RobotContainer.m_Transport.TowerStop();
    RobotContainer.m_Transport.HopperIn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
