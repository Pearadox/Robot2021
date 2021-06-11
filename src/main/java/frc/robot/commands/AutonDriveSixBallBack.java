// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.RamseteConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.lib.util.TrajectoryCache;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonDriveSixBallBack extends SequentialCommandGroup {
  /** Creates a new BounceSequence. */
  public AutonDriveSixBallBack(Drivetrain drivetrain) {
    RamseteController ramseteController = new RamseteController(RamseteConstants.B, RamseteConstants.ZETA);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(RamseteConstants.ksVolts, RamseteConstants.kvVoltSecondsPerMeter, RamseteConstants.kaVoltSecondsSquaredPerMeter);
    PIDController pidLeft = new PIDController(RamseteConstants.kPDriveVel, 0, 0);
    PIDController pidRight = new PIDController(RamseteConstants.kPDriveVel, 0, 0);

    Trajectory trajectory0 = TrajectoryCache.get("SixBallBackwardsTurn");
    RamseteCommand ramsete0 = new RamseteCommand(trajectory0, drivetrain::getPose, ramseteController, feedforward, DrivetrainConstants.KINEMATICS, drivetrain::getWheelSpeeds, pidLeft, pidRight, drivetrain::tankDriveVolts, drivetrain);
    Trajectory trajectory1 = TrajectoryCache.get("SixBallBackwardsStraight");
    RamseteCommand ramsete1 = new RamseteCommand(trajectory1, drivetrain::getPose, ramseteController, feedforward, DrivetrainConstants.KINEMATICS, drivetrain::getWheelSpeeds, pidLeft, pidRight, drivetrain::tankDriveVolts, drivetrain);
    Trajectory trajectory2 = TrajectoryCache.get("SixBallForwardsTrench");
    RamseteCommand ramsete2 = new RamseteCommand(trajectory2, drivetrain::getPose, ramseteController, feedforward, DrivetrainConstants.KINEMATICS, drivetrain::getWheelSpeeds, pidLeft, pidRight, drivetrain::tankDriveVolts, drivetrain);
    
    addCommands(
      ramsete0.beforeStarting(() -> drivetrain.resetOdometry(trajectory0.getInitialPose()), drivetrain)
        .alongWith( new WaitCommand(2.5)
          .andThen( new AutonResetArmandEncoder())),
      new WaitCommand(0.1),
      ramsete1.beforeStarting(() -> drivetrain.resetOdometry(trajectory1.getInitialPose()), drivetrain),
      new ArmSmartMotionUp()
        .alongWith( new AutonTransportLoading()
            .withTimeout(3))
        .alongWith( new WaitCommand(0.11)
        .andThen( ramsete2.beforeStarting(() -> drivetrain.resetOdometry(trajectory2.getInitialPose()), drivetrain)
            .andThen(() -> drivetrain.tankDriveVolts(0, 0))))
    );
  }
}