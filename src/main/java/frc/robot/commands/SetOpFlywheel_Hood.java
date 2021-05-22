// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionLL.HoodShooterSettings;
import frc.robot.subsystems.VisionLL.OperatorSettings;

public class SetOpFlywheel_Hood extends CommandBase {
  /** Creates a new SetFlywheel_Hood. */
  HoodShooterSettings opSettings;

  public SetOpFlywheel_Hood(Shooter shooter, Hood hood) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    opSettings = RobotContainer.visionLL.getOperatorHoodShooterSettings();
    RobotContainer.m_Hood.setHoodAngle(opSettings.getTargetHoodAngle());
    RobotContainer.m_Shooter.setShooterVoltage(opSettings.getTargetShooterVoltage());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    opSettings = RobotContainer.visionLL.getOperatorHoodShooterSettings();
    RobotContainer.m_Hood.setHoodAngle(opSettings.getTargetHoodAngle());
    RobotContainer.m_Shooter.setShooterVoltage(opSettings.getTargetShooterVoltage());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Hood.stopHood();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(RobotContainer.m_Hood.getHoodError()) < RobotContainer.m_Hood.kMinError
     && Math.abs(RobotContainer.m_Shooter.getFlywheelRPM() - RobotContainer.m_Shooter.getShooterReference()) < 200)
        return true;
    return false;
  }
}
