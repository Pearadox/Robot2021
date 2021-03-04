// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;

public class SetHood extends CommandBase {
  private double kP, kFF, kMaxOutput, kMinOutput, currAngle, setPointAngle, error, THRESHOLD; 

  /** Creates a new SetHood. */
  public SetHood(Hood m_hood) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_hood);

    kMaxOutput = 1;
    kMinOutput = -1;
    THRESHOLD = 5;
    kP = 1/ THRESHOLD;
  }
 


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currAngle = SmartDashboard.getNumber("Hood Angle", 0);
    setPointAngle = SmartDashboard.getNumber("Set Hood Angle", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = setPointAngle - currAngle;
    double output = error * kP;
    if (output > kMaxOutput) {
      output = kMaxOutput;
    } 
    if (output < kMinOutput) {
      output = kMinOutput;
    }
    RobotContainer.m_Hood.setHoodSpeed(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
