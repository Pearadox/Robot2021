// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANEncoder;
import com.revrobotics.ControlType;

import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeDown extends CommandBase {
  /** Creates a new IntakeDown. */
  private double encoderPosition = RobotContainer.m_Intake.getArmIntakePosition();
  boolean currentDownSpike = false;
  double targetSpike;
  
  public IntakeDown(Intake m_Intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Intake);
  }
  
// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_Intake.setArmIntakeSpeed(-0.2);
    RobotContainer.m_Intake.setRollerSpeed(RobotContainer.m_Intake.in_speed);
      currentDownSpike = false;
    
  }
  
// Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_Intake.setRollerSpeed(RobotContainer.m_Intake.in_speed);
    RobotContainer.m_Intake.setArmIntakeSpeed(-0.2);
    encoderPosition = RobotContainer.m_Intake.getArmIntakePosition();
    
    /* psuedo code
    * move arm down until current exceeds some value
    * turn off motor
    * set an "arm down pos" variable to the arm encoder
    * use a default command that trys to maintin current encoder value +/- epsilon
    * do similar for up except do arm up pos
​
    */
    // if(RobotContainer.m_Intake.getArmIntakePosition() > -17)
    // {
    // if(RobotContainer.m_Intake.getIntakeCurrent() < 25.0)
    // {
    //   if(currentDownSpike) {
    //     //previously hit current spike from going down, Trying to just maintain current position?
    //     if(RobotContainer.m_Intake.getArmIntakePosition() > RobotContainer.m_Intake.getIntakeDownPos())
    //     {
    //       //ball is lifting the intake, push it back down
    //       RobotContainer.m_Intake.setArmIntakeSpeed(-0.2);
    //     }
    //     else
    //     {
    //       //arm should be resting on ground, let it maintain position
    //       RobotContainer.m_Intake.setArmIntakeSpeed(0);

    //     }
    //   }
    //   else {
    //     //havent yet hit the current spike, go down at reasonable speed
    //     RobotContainer.m_Intake.setArmIntakeSpeed(-0.2);
    //   }

    // }
    // else
    // {
    //   currentDownSpike = true;
    //   RobotContainer.m_Intake.setIntakeDownPos(RobotContainer.m_Intake.getArmIntakePosition()); 
    // }

    // }
    // else {
    //   RobotContainer.m_Intake.setArmIntakeSpeed(0);
    // }
    // if (currentDownSpike){
    //   RobotContainer.m_Intake.ArmPidController.setReference(SmartDashboard.getNumber("Arm Set Position", 0), ControlType.kPosition);  
    //   RobotContainer.m_Intake.RollerIn();
    // }
    //  else{
    //   if(SmartDashboard.getNumber("Arm Current",0) < 10) {
    //     RobotContainer.m_Intake.setArmIntakeSpeed(-0.1);
    //   }
    //   else{
    //     RobotContainer.m_Intake.setArmIntakeSpeed(0);
    //     currentDownSpike=true;
    //     targetSpike = RobotContainer.m_Intake.getArmIntakePosition();
    //     }
    //  }
  }
    
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   RobotContainer.m_Intake.setRollerSpeed(0);
   currentDownSpike = false;
   RobotContainer.m_Intake.setArmIntakeSpeed(0); 
  }
  
// Returns true when the command should end.
  @Override
  public boolean isFinished()
   {
    return false;
  }
}