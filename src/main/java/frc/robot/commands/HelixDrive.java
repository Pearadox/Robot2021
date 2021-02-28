// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class HelixDrive extends CommandBase {
  private final Drivetrain drivetrain;
  /** Creates a new HelixDrive. */
  public HelixDrive(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle = RobotContainer.driverJoystick.getY();
    double twist = RobotContainer.driverJoystick.getZ() * -0.8;

    double saturatedInput;
    double greaterInput = Math.max(Math.abs(twist), Math.abs(throttle));
    double lesserInput = Math.min(Math.abs(twist), Math.abs(throttle));

    if (greaterInput > 0.0) {
      saturatedInput = (lesserInput/greaterInput) + 1.0;
    }
    else {
      saturatedInput = 1.0;
    }

    throttle = throttle / saturatedInput;
    twist = twist/saturatedInput;
    if(Math.abs(throttle) < 0.1) {throttle = 0;}
    if(Math.abs(twist) < 0.1) {twist = 0;}

    drivetrain.arcadeDrive(throttle, twist);


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
