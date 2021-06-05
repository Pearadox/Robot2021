package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.commands.AutonDriveStraight;
import frc.robot.commands.ConfirmShotVision;
import frc.robot.commands.HopperInTowerUpCmd;
import frc.robot.subsystems.VisionLL.OperatorSettings;

public class ThreeBallAuton extends SequentialCommandGroup {

    public ThreeBallAuton() {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new FooCommand(), new BarCommand());


        super(

            new InstantCommand(
                () -> {
                  RobotContainer.visionLL.setOperatorSettings(OperatorSettings.INITIATION);
                }, RobotContainer.visionLL),
            new ConfirmShotVision(RobotContainer.m_Drivetrain, RobotContainer.m_Hood, RobotContainer.m_Shooter, RobotContainer.visionLL ).withTimeout(1.5),
            new HopperInTowerUpCmd().withTimeout(1.5),
            new AutonDriveStraight(RobotContainer.m_Drivetrain)
        )
        ;
    }
}