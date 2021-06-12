// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TransportSystem;

public class liftTowerOne extends CommandBase {
  private TransportSystem ballTower;
  /** Creates a new liftTowerOne. */
  public liftTowerOne(TransportSystem tower) {
    // Use addRequirements() here to declare subsystem dependencies.
    ballTower = tower;
    addRequirements(ballTower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(ballTower.getBallCounter() < (ballTower.getMaxBallCounter())) {
      if(ballTower.getBallCounter() == 1)
      {
        ballTower.TowerUp(0.32);
      }
      else 
      {
        ballTower.TowerUp(0.32);
      }
    }
    else {
      ballTower.TowerStop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ballTower.TowerStop();
    if(!ballTower.isBallLow()){
      ballTower.incrementBallCounter();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (ballTower.getHigh()) {
      ballTower.setBallCounter(ballTower.getMaxBallCounter());
      return true;
    }

    if(ballTower.getBallCounter() == 0)
    {
      return (ballTower.getMedium() && !ballTower.isBallLow());
    }
    else if(ballTower.getBallCounter() == 1)
    {
      return (ballTower.getHigh() && ballTower.getMedium() && !ballTower.isBallLow());
    }
    else {
      return true;
    }
  }
}
