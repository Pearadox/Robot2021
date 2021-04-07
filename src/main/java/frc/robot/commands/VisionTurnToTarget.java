// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionLL;

public class VisionTurnToTarget extends CommandBase {
  /** Creates a new VisionTurnToTarget. */

  private double kp = RobotContainer.visionLL.turnKp;
  private double kd = RobotContainer.visionLL.turnKd;
  private double ki = RobotContainer.visionLL.turnKi;
  private double tx;
  private boolean reachedTarget, foundTarget;
  private double changeInError, errorSum = 0; 
  private double lastError;
  public VisionTurnToTarget(Drivetrain driveTrain, VisionLL visionLL) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain, visionLL);
    if (!SmartDashboard.containsKey("Vision Output")) SmartDashboard.putNumber("Vision Output", 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.visionLL.limeLightLEDOn();
    
    tx = RobotContainer.visionLL.getLLDegToTarget();
    kp = SmartDashboard.getNumber("Vision Turn kp", kp);
    ki = SmartDashboard.getNumber("Vision Turn ki", ki);
    kd = SmartDashboard.getNumber("Vision Turn kd", kd);
    foundTarget = RobotContainer.visionLL.getLLIsTargetFound();
    errorSum = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!foundTarget) {
      RobotContainer.m_Drivetrain.arcadeDrive(Math.min(RobotContainer.getDriverJoystick().getRawAxis(0), 0.6),
                                              Math.min(RobotContainer.getDriverJoystick().getRawAxis(0), 0.4));
    }
    else {
      tx = RobotContainer.visionLL.getLLDegToTarget();
      changeInError = lastError - tx;
      errorSum += tx;
      double P = kp * tx;
      double I = ki * errorSum;
      double D = kd * changeInError;
      double output = -1*(P + I - D);
      lastError = tx;
      RobotContainer.m_Drivetrain.arcadeDrive(0, output);
      if (Math.abs(tx) < 0.5) {
        reachedTarget = true;
      }
      SmartDashboard.putNumber("Vision Output", output);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Drivetrain.stop();
    reachedTarget = false;
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return reachedTarget;
  }
}
