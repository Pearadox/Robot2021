// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;

public class Climb extends CommandBase {

  /**
   * Creates a new ClimbUp.
   */
  double climbCurrent;
  public Climb(Climber climber, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber,intake);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_Climber.setEngageBrake();
    // RobotContainer.m_Hood.hoodDown(0.3);
    // RobotContainer.m_Hood.setHoodAngle(5);
    RobotContainer.m_Intake.setArmPosition(RobotContainer.m_Intake.IntakeUpEncoderValue);
    RobotContainer.m_Intake.setRollerSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbCurrent = RobotContainer.m_Climber.getClimbCurrent();
    RobotContainer.m_Climber.setClimbMotor(1.0);
    // if(RobotContainer.m_Hood.getHoodSwitch())
    //   RobotContainer.m_Hood.stopHood();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Climber.setClimbMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (climbCurrent >= 40) {
      RobotContainer.m_Climber.setClimbMotor(0);
      return true;
    }
    else { 
      return false;
    }
  }
}
