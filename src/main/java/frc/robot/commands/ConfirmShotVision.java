package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionLL;
import frc.robot.subsystems.VisionLL.OperatorSettings;


public class ConfirmShotVision extends CommandBase {
    private double kp = RobotContainer.visionLL.turnKp;
    private double kd = RobotContainer.visionLL.turnKd;
    private double ki = RobotContainer.visionLL.turnKi;
    private double tx;
    private boolean reachedTarget, foundTarget;
    private double changeInError, errorSum = 0;
    private double lastError;
    private VisionLL.HoodShooterSettings currZone; // Currently Running on Operator Settings

    public ConfirmShotVision(Drivetrain drivetrain, Hood hood, Shooter shooter, VisionLL visionLL) {
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(drivetrain, hood, shooter, visionLL, RobotContainer.m_Intake);
        if (!SmartDashboard.containsKey("Vision Output")) SmartDashboard.putNumber("Vision Output", 0);
    }

    @Override
    public void initialize() {
        currZone = RobotContainer.visionLL.getOperatorHoodShooterSettings();

        tx = RobotContainer.visionLL.getLLDegToTarget();
        kp = SmartDashboard.getNumber("Vision Turn kp", kp);
        ki = SmartDashboard.getNumber("Vision Turn ki", ki);
        kd = SmartDashboard.getNumber("Vision Turn kd", kd);
        foundTarget = RobotContainer.visionLL.getLLIsTargetFound();
        errorSum = 0;
        RobotContainer.m_Intake.StopRollers();

    }

    @Override
    public void execute() {
        // currZone = RobotContainer.visionLL.getZone();
        currZone = RobotContainer.visionLL.getOperatorHoodShooterSettings();
        if(!(Math.abs(RobotContainer.m_Hood.getHoodError()) < RobotContainer.m_Hood.kMinError))
            RobotContainer.m_Hood.setHoodAngle(currZone.getTargetHoodAngle());
        RobotContainer.m_Shooter.setShooterVoltage(currZone.getTargetShooterVoltage());
        if (!foundTarget && RobotContainer.visionLL.getOperatorSettings() == OperatorSettings.TRIANGLE) {
            RobotContainer.m_Drivetrain.arcadeDrive(Math.min(RobotContainer.getDriverJoystick().getRawAxis(0), 0.6) * 0.5,
                    Math.min(RobotContainer.getDriverJoystick().getRawAxis(0), 0.4)* 0.5);
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
            if (Math.abs(tx) < RobotContainer.visionLL.MIN && RobotContainer.visionLL.getOperatorSettings() != OperatorSettings.TRENCH)
                reachedTarget = true;
            else if (Math.abs(tx) < 0.25 && RobotContainer.visionLL.getOperatorSettings() == OperatorSettings.TRENCH)
                reachedTarget = true;
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
        if (Math.abs(RobotContainer.m_Hood.getHoodError()) < RobotContainer.m_Hood.kMinError
        &&  Math.abs(RobotContainer.m_Shooter.getFlywheelRPM()-RobotContainer.m_Shooter.getShooterReference())<200
        &&  reachedTarget)
            return true;
        return false;
    }

}
