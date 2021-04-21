package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionLL;
import frc.robot.subsystems.VisionLL.HoodShooterSettings;


public class ConfirmShotVision extends CommandBase {
    private final Drivetrain drivetrain;
    private final Hood hood;
    private final Shooter shooter;
    private final VisionLL visionLL;
    private double kp = RobotContainer.visionLL.turnKp;
    private double kd = RobotContainer.visionLL.turnKd;
    private double ki = RobotContainer.visionLL.turnKi;
    private double tx;
    private boolean reachedTarget, foundTarget;
    private double changeInError, errorSum = 0;
    private double lastError;

    public ConfirmShotVision(Drivetrain drivetrain, Hood hood, Shooter shooter, VisionLL visionLL) {
        this.drivetrain = drivetrain;
        this.hood = hood;
        this.shooter = shooter;
        this.visionLL = visionLL;
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.drivetrain, this.hood, this.shooter, this.visionLL);
        if (!SmartDashboard.containsKey("Vision Output")) SmartDashboard.putNumber("Vision Output", 0);
    }

    @Override
    public void initialize() {
        HoodShooterSettings currZone = RobotContainer.visionLL.getZone();
        RobotContainer.m_Hood.setHoodAngle(currZone.getTargetHoodAngle());
        RobotContainer.m_Shooter.setShooterVoltage(currZone.getTargetShooterVoltage());
        RobotContainer.visionLL.limeLightLEDOn();

        tx = RobotContainer.visionLL.getLLDegToTarget();
        kp = SmartDashboard.getNumber("Vision Turn kp", kp);
        ki = SmartDashboard.getNumber("Vision Turn ki", ki);
        kd = SmartDashboard.getNumber("Vision Turn kd", kd);
        foundTarget = RobotContainer.visionLL.getLLIsTargetFound();
        errorSum = 0;
    }

    @Override
    public void execute() {
        HoodShooterSettings currZone = RobotContainer.visionLL.getZone();
        RobotContainer.m_Hood.setHoodAngle(currZone.getTargetHoodAngle());
        RobotContainer.m_Shooter.setShooterVoltage(currZone.getTargetShooterVoltage());
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
    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_Hood.stopHood();
        RobotContainer.m_Drivetrain.stop();
        reachedTarget = false;
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(RobotContainer.m_Hood.getHoodError()) < RobotContainer.m_Hood.kMinError) {
            if(Math.abs(RobotContainer.m_Shooter.getFlywheelRPM()-RobotContainer.m_Shooter.getShooterReference())<200 ) {
                if(reachedTarget) {
                    return true;
                }
            }
        }
        return false;
    }

}
