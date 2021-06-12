// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.TrajectoryCache;
import frc.robot.RobotContainer;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RamseteConstants;
import frc.robot.commands.AutonResetArmandEncoder;
import frc.robot.commands.AutonTransportLoading;
import frc.robot.commands.HopperInTowerUpCmd;
import frc.robot.commands.SetOpFlywheel_Hood;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionLL.OperatorSettings;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TriangleThreeBallAuton extends SequentialCommandGroup {
  /** Creates a new TriangleThreeBallAuton. */
  public TriangleThreeBallAuton(Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    RamseteController ramseteController = new RamseteController(RamseteConstants.B, RamseteConstants.ZETA);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(RamseteConstants.ksVolts, RamseteConstants.kvVoltSecondsPerMeter, RamseteConstants.kaVoltSecondsSquaredPerMeter);
    PIDController pidLeft = new PIDController(RamseteConstants.kPDriveVel, 0, 0);
    PIDController pidRight = new PIDController(RamseteConstants.kPDriveVel, 0, 0);

    Trajectory trajectory0 = TrajectoryCache.get("TriangleFirstTurn");
    RamseteCommand ramsete0 = new RamseteCommand(trajectory0, drivetrain::getPose, ramseteController, feedforward, DrivetrainConstants.KINEMATICS, drivetrain::getWheelSpeeds, pidLeft, pidRight, drivetrain::tankDriveVolts, drivetrain);
    Trajectory trajectory1 = TrajectoryCache.get("TriangleBackwardsStraight");
    RamseteCommand ramsete1 = new RamseteCommand(trajectory1, drivetrain::getPose, ramseteController, feedforward, DrivetrainConstants.KINEMATICS, drivetrain::getWheelSpeeds, pidLeft, pidRight, drivetrain::tankDriveVolts, drivetrain);
    
    addCommands(
      new InstantCommand(
      () -> {
        RobotContainer.visionLL.setOperatorSettings(OperatorSettings.TRIANGLE);
      }, RobotContainer.visionLL),
      new SetOpFlywheel_Hood(RobotContainer.m_Shooter, RobotContainer.m_Hood).withTimeout(1)
        .alongWith(new WaitCommand(0.5)).andThen(ramsete0.beforeStarting(() -> drivetrain.resetOdometry(trajectory0.getInitialPose()), drivetrain))
          .andThen(() -> drivetrain.tankDriveVolts(0, 0)),
      new HopperInTowerUpCmd().withTimeout(1.0),
      ramsete1.beforeStarting(() -> drivetrain.resetOdometry(trajectory1.getInitialPose()), drivetrain)
        .alongWith(new WaitCommand(3).andThen(new AutonResetArmandEncoder()).andThen(new AutonTransportLoading()))
    );
  }
}
