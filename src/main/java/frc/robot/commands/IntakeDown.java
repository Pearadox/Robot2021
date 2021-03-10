// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeDown extends CommandBase {
  /** Creates a new IntakeDown. */
  private double encoderPosition = RobotContainer.m_Intake.getArmIntakePosition();
  
  public IntakeDown(Intake m_Intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    encoderPosition = RobotContainer.m_Intake.getArmIntakePosition();

    /* psuedo code
    * move arm down until current exceeds some value
    * turn off motor
    * set an "arm down pos" variable to the arm encoder
    * use a default command that trys to maintin current encoder value +/- epsilon
    * do similar for up except do arm up pos

    */
    if(encoderPosition > -17){
      if(RobotContainer.m_Intake.getIntakeCurrent() < 10) {
        RobotContainer.m_Intake.setArmIntakeSpeed(-0.1);
      }
      else{
        RobotContainer.m_Intake.setArmIntakeSpeed(0);

      }
    }
    else {
      
      RobotContainer.m_Intake.setArmIntakeSpeed(0);
      
    }
    RobotContainer.m_Intake.RollerIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
   {
    return false;
  }
}
