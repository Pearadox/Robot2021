// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class ResetArmandEncoder extends CommandBase {
  /** Creates a new ResetArmandEncoder. */
  public ResetArmandEncoder() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_Intake.setArmIntakeSpeed(-0.4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Intake.setArmIntakeSpeed(0);
    RobotContainer.m_Intake.resetArmIntakeEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(RobotContainer.m_Intake.getOutputCurrent() > 15)
    // {
    //   return true;
    // }
    return false;
  }
}
