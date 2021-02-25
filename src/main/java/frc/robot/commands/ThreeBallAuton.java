package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TransportSystem;
import frc.robot.subsystems.TransportSystem.;

public class ThreeBallAuton extends ParallelCommandGroup {
    public ThreeBallAuton() {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new FooCommand(), new BarCommand());

        //syntax still being worked on :)
        super(

                new RunCommand(() -> RobotContainer.m_Transport.HopperIn(), RobotContainer.m_Transport).withTimeout(5.0),
                new RunCommand(() -> RobotContainer.m_Transport.TowerUp(), RobotContainer.m_Transport).withTimeout(0.5)
                        .andThen(new Drivetrain().arcadeDrive(0.2,0), RobotContainer.m_Drivetrain).withTimeout(0.5)
        );
    }
}