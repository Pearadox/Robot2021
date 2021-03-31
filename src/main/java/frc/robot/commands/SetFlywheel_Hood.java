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

public class SetFlywheel_Hood extends CommandBase {
  /** Creates a new SetFlywheel_Hood. */
  public SetFlywheel_Hood(Shooter shooter, VisionLL limelight, Hood hood) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, limelight, hood);
  }
  private double shooterVoltage;
  private double hoodAngle;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (RobotContainer.visionLL.enteredZone) {
      case 0:
        shooterVoltage = 0;
        hoodAngle = 0;
        break;
      case 1:
        shooterVoltage = 0;
        hoodAngle = 0;
        break;
      case 2:
        shooterVoltage = 0;
        hoodAngle = 0;
        break;
      case 3:
        shooterVoltage = 0;
        hoodAngle = 0;
        break;
      default:
        shooterVoltage = 0;
        hoodAngle = 0;
        break;
    }
    if (Math.abs(SmartDashboard.getNumber("Hood Error", 0))  < RobotContainer.m_Hood.kMinError) {
      RobotContainer.m_Hood.setHoodAngle(hoodAngle);;
    } else {
      RobotContainer.m_Hood.stopHood();
    }
    RobotContainer.m_Shooter.setShooterVoltage(shooterVoltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Hood.stopHood();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
