// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionLL;

public class VisionDriveToTarget extends CommandBase {
  /** Creates a new VisionTurnToTarget. */

  private double kp = RobotContainer.visionLL.turnKp;
  private double kd = RobotContainer.visionLL.turnKd;
  private double ki = RobotContainer.visionLL.turnKi;
  private double tx;
  // private boolean reachedTarget, foundTarget;
  private double changeInError, errorSum = 0; 
  private double OFFSET = 0.215;
  private double lastError;
  public VisionDriveToTarget(Drivetrain driveTrain, VisionLL visionLL) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain, visionLL);
    if (!SmartDashboard.containsKey("Vision Output")) SmartDashboard.putNumber("Vision Output", 0);
    if (!SmartDashboard.containsKey("Vision Turn kp")) SmartDashboard.putNumber("Vision Turn kp", kp);
    if (!SmartDashboard.containsKey("Vision Turn ki")) SmartDashboard.putNumber("Vision Turn ki", ki);
    if (!SmartDashboard.containsKey("Vision Turn kd")) SmartDashboard.putNumber("Vision Turn kd", kd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.visionLL.limeLightLEDOn();
    
    tx = RobotContainer.visionLL.getLLDegToTarget();
    kp = SmartDashboard.getNumber("Vision Turn kp", kp);
    ki = SmartDashboard.getNumber("Vision Turn ki", ki);
    kd = SmartDashboard.getNumber("Vision Turn kd", kd);
    // foundTarget = RobotContainer.visionLL.getLLIsTargetFound();
    errorSum = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      OFFSET = SmartDashboard.getString("Entered Zone", "Unknown").equals("Red Zone") ? 0.0 : 0.215;
      double throttle = RobotContainer.getDriverJoystick().getY();
      double twist = RobotContainer.getDriverJoystick().getZ();
      if (RobotContainer.visionLL.getLLIsTargetFound()) {
        tx = RobotContainer.visionLL.getLLDegToTarget() + OFFSET;
        changeInError = lastError - tx;
        errorSum += tx;
        double P = kp * tx;
        double I = ki * errorSum;
        double D = kd * changeInError;
        double output = -1*(P + I - D);
        lastError = tx;
        if (Math.abs(tx) < 0.5) 
          output = 0; 
        twist = Math.copySign(Math.min(Math.abs(Math.pow(twist, 2)), 0.2), twist);
        RobotContainer.m_Drivetrain.arcadeDrive(throttle * 0.75, output - twist);
        SmartDashboard.putNumber("Vision Output", output);
      } else {
        RobotContainer.m_Drivetrain.HelixDrive();
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
