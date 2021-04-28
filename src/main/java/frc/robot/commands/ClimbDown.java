// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.RobotContainer;

public class ClimbDown extends CommandBase {

  double climberCurrent;
  
  /** Creates a new DriveBackward. */
  public ClimbDown(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    RobotContainer.m_Climber.setDisengageBrake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climberCurrent = RobotContainer.m_Climber.getClimbCurrent();
    RobotContainer.m_Climber.setClimbMotor(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Climber.setClimbMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (climberCurrent >= 1) {
    //   return true;
    // }
    // else {
    //   return false;
    // }
    return false;
  }
}
