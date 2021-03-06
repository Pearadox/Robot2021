// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;

public class SetZeroHood extends CommandBase {
  /** Creates a new SetZeroHood. */
  double ZeroingSpeed = 0;
  boolean ZeroEncoder = false;
  public SetZeroHood(Hood m_hood, double speed, boolean zero) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_hood);
    ZeroingSpeed = speed;
    ZeroEncoder = zero;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_Hood.hoodDown(ZeroingSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Hood.stopHood();
    RobotContainer.m_Hood.zeroHood();
    RobotContainer.m_Hood.sethasHoodZeroed(ZeroEncoder);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.m_Hood.getHoodSwitch();
  }
}
