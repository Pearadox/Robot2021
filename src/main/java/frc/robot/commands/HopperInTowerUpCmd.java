// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class HopperInTowerUpCmd extends CommandBase {
  /** Creates a new HopperInTowerUpCmd. */
  boolean clearedBottomOnce = false;
  Timer timer;
  private final double TotalTime = 0.25;
  private final double RunTime = 0.25;
  public HopperInTowerUpCmd() {
    // Use addRequirements() here to declare subsystem dependencies.
    timer = new Timer();
    addRequirements(RobotContainer.m_Transport);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_Transport.resetBallCounter();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(RobotContainer.m_Shooter.isFlywheelInRange()) {
      if (timer.get() < RunTime) {
        RobotContainer.m_Transport.TowerUp(1);
        if(!RobotContainer.m_Transport.getLow())
        {
          clearedBottomOnce = true;
        }
        if(clearedBottomOnce)
        {
          RobotContainer.m_Transport.HopperInSet(1.0);
        }
      } else if(timer.get() < TotalTime) {
        RobotContainer.m_Transport.TowerUp(0.8);
      } else {
        timer.reset();
      }
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Transport.HopperInOnly();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
