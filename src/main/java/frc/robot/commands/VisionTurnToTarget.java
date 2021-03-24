package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionLL;


public class VisionTurnToTarget extends CommandBase {
    private final Drivetrain drivetrain;
    private final VisionLL visionLL;
    private double lastError = 0;
    private double error_sum = 0;
    private double kp = 0;
    private double ki = 0;
    private double kd = 0;

    boolean inTeleop;

    public VisionTurnToTarget(Drivetrain drivetrain, VisionLL visionLL) {
        this.drivetrain = drivetrain;
        this.visionLL = visionLL;
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.drivetrain, this.visionLL);
    }

    @Override
    public void initialize() {
        error_sum = 0;
        RobotContainer.visionLL.limeLightLEDOn();
        inTeleop = true;
    }

    @Override
    public void execute() {
        if(RobotContainer.visionLL.getLLIsTargetFound()) inTeleop = false;
        if(inTeleop){

        }else{
            double changeInError = lastError - Robot.l.getX();
            error_sum += RobotContainer.visionLL.getLLRobotToTargetDistance();

            double P = kp * RobotContainer.visionLL.getLLRobotToTargetDistance();
            double I = ki * error_sum;
            double D = kd * changeInError;
            lastError = RobotContainer.visionLL.getLLRobotToTargetDistance();
            double output = P + I - D;

            if(output > 0) output += 0.1;
            else output -= 0.1;

            RobotContainer.m_Drivetrain.arcadeDrive(output,-output);
        }

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
