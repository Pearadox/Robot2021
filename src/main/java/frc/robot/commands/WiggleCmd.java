// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TransportSystem;

public class WiggleCmd extends CommandBase {
  private Timer timer;
  private final static double TotalTime = 5.0;
  private final static double ReverseTime = 0.75;
  TransportSystem tower;
  /** Creates a new WiggleCmd. */
  public WiggleCmd(TransportSystem tower) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tower);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // RobotContainer.m_Transport.HopperIn();
    if(timer.get() < (TotalTime - ReverseTime)) {
      RobotContainer.m_Transport.HopperIn();
    }
    else if (timer.get() < TotalTime) {
      RobotContainer.m_Transport.HopperOut();
    }
    else {      
      timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // RobotContainer.m_Transport.HopperStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
