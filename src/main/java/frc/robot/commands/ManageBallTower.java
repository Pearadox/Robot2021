// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.sql.rowset.spi.TransactionalWriter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.TransportSystem;
import frc.robot.subsystems.TransportSystem.TowerState;

public class ManageBallTower extends CommandBase {
  /** Creates a new ManageBallTower. */
  private TransportSystem tower;
  private TowerState curr_state;
  public ManageBallTower() {
    // Use addRequirements() here to declare subsystem dependencies.
    tower = RobotContainer.m_Transport;
    addRequirements(tower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Determine current state
    // low = tower.getLow();
    // mid = tower.getMedium();
    // high = tower.getHigh();
    // tower.determineState();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    curr_state = tower.getState();
    switch (curr_state) {
      case EMPTY:
      case READY_2:
      case READY_3:
      case LOADED_3:
        tower.stop();
        break;
      case LOADED_1:
      case LOADED_2:
        tower.up();
        break;
      case UNKNOWN:
        tower.down();
        break;
      default:
        tower.stop();
        break;
    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tower.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
