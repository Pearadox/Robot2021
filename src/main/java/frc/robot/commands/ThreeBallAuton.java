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

                 new ShooterVoltage(RobotContainer.m_Shooter).withTimeout(2.5),
                 new AutonTowerUp(RobotContainer.m_Transport).withTimeout(7)
                        .alongWith(new ShooterVoltage(RobotContainer.m_Shooter).withTimeout(7)),
                 new DriveForward(RobotContainer.m_Drivetrain).withTimeout(5)
        )
        ;
    }
}