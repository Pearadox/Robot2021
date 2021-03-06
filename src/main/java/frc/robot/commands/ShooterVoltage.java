package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;


public class ShooterVoltage extends CommandBase {
    private final Shooter shooter;
    private double voltage;

    public ShooterVoltage(Shooter shooter, double voltage) {
        this.shooter = shooter;
        this.voltage = voltage;
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        RobotContainer.m_Shooter.setShooterVoltage(voltage);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
