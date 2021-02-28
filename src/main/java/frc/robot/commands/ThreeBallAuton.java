package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
;

public class ThreeBallAuton extends SequentialCommandGroup {

    public ThreeBallAuton() {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new FooCommand(), new BarCommand());


        super(

                 new ShooterVoltage(RobotContainer.m_Shooter, 4.25).withTimeout(4),
                 new AutonTowerUp(RobotContainer.m_Transport).withTimeout(4)
                        .alongWith(new ShooterVoltage(RobotContainer.m_Shooter, 4.3).withTimeout(4)),
                 new DriveForward(RobotContainer.m_Drivetrain).withTimeout(1),
                 new DriveBackward(RobotContainer.m_Drivetrain).withTimeout(2)
        )
        ;
    }
}