// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionLL;
import frc.robot.subsystems.VisionLL.HoodShooterSettings;

public class SetFlywheel_Hood extends CommandBase {
  /** Creates a new SetFlywheel_Hood. */
  public SetFlywheel_Hood(Shooter shooter, VisionLL limelight, Hood hood) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, limelight, hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    HoodShooterSettings currZone = RobotContainer.visionLL.getZone();
    RobotContainer.m_Hood.setHoodAngle(currZone.getTargetHoodAngle());
    RobotContainer.m_Shooter.setShooterVoltage(currZone.getTargetShooterVoltage());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Hood.stopHood();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(SmartDashboard.getNumber("Hood Error", 0))  < RobotContainer.m_Hood.kMinError) {
      if(Math.abs(RobotContainer.m_Shooter.getFlywheelRPM()-RobotContainer.m_Shooter.getShooterReference())<200 ){
        return true;
      }        
    }
    return false;
  }
}
