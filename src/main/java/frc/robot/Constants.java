

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static int CANconfigTimeOut = 0;

    public static final class RobotConstants {
        public static final int minBatteryVoltage = 12;
    }

    public static final class DrivetrainConstants {
        public static final int FRONT_RIGHT_MOTOR = 2;
        public static final int BACK_RIGHT_MOTOR = 3;

        public static final int FRONT_LEFT_MOTOR = 4;
        public static final int BACK_LEFT_MOTOR = 5;

        // TODO: change to the trackwidth suggested in frc characterization
        public static final double DRIVE_BASE_WIDTH = 1.4; //29.5 Meters
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(6.0d); // Meters

        public static final double DISTANCE_PER_REVOLUTION = WHEEL_DIAMETER * Math.PI;
        public static final double PULSES_PER_REVOLUTION = 42 * 5.6;
        public static final double DISTANCE_PER_PULSE = DISTANCE_PER_REVOLUTION / PULSES_PER_REVOLUTION;
        public static final double SECONDS_PER_MINUTE = 60.0d;
        public static final double GEAR_REDUCTION = 13.8;

        public static final DifferentialDriveKinematics KINEMATICS =
            new DifferentialDriveKinematics(DRIVE_BASE_WIDTH);

        public static final double THROTTLE_DEADBAND = 0.01d;
        public static final double TWIST_DEADBAND = 0.01d;
        public static final double MAX_OUTPUT = 1.0d;

        public static final double MAX_SPEED = 0.0d; // Meters per second
        public static final double MAX_ACCELERATION = 0.0d; // Meters per second^2
        public static final boolean kGyroReversed = false;
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;
    }

    public static final class FlywheelConstants {
        public static final int LEFT_FLY_MOTOR = 7;
        public static final int RIGHT_FLY_MOTOR = 8;

        public static final double flykP = 0.0018;
        public static final double flykD = 0.004;

        public static final double TARGET_ZONE_VOLTAGE = 3.75;
        public static final double INITIATION_LINE_VOLTAGE = 4.0;
    }

    public static final class HoodConstants {
        public static final int HOOD_MOTOR_ID = 6;
        public static final int HOOD_SWITCH_PORT = 9;

        public static final double TARGET_ZONE_ANGLE_DEG = 7;
        public static final double INITIATION_LINE_ANGLE_DEG = 38;
    }

    public static final class RamseteConstants {
        public static final double B = 2.0d;
        public static final double ZETA = 0.7d;

        public static final double KS = 0.23d; // Volts

        public static final double ksVolts = 0.23;
        public static final double kvVoltSecondsPerMeter = 3.61;
        public static final double kaVoltSecondsSquaredPerMeter = 0.529;
        public static final double kPDriveVel = 0.0; //2.04;
    }

    public static final class MPConstants {
        public static final double DEFAULT_KV = 0.076; // 1/13
        public static final double DEFAULT_KA = 0.035;
        public static final double DEFAULT_KH = 0;
        public static final double DEFAULT_KP = 0.3;
        public static final double DEFAULT_KD = 0;
    }

    public static final class TowerConstants {
        public static final int TOWER_MOTOR = 12;
        public static final int HOPPER_MOTOR = 11;
        public static final int BOTTOM_SENSOR_DIO = 6 ;
        public static final int MIDDLE_SENSOR_DIO = 7;
        public static final int TOP_SENSOR_DIO = 8;
    }

    public static final class ClimberConstants {
        public static final int CLIMB_MOTOR = 9;
        public static final int CLIMB_SERVO = 9;
    }

    public static final class TraverseConstants {
        public static final int TRANSVERSE_CLIMB_MOTOR = 10;
    }

    public static final class IntakeConstants {
        public static final int ARM_INTAKE_MOTOR = 13;
        public static final int TOP_ROLLER_MOTOR = 14;
        public static final int BOT_ROLLER_MOTOR = 15;
    }

    // public static final class TransportConstants {
    //     public static final int HOPPER_MOTOR = 11;
    // }

    public static final class LimelightConstants {
        public static final double LIMELIGHT_ANGLE = 0.872665; // Radians
        public static final double LIMELIGHT_HEIGHT = 0.7747; // Meters
        private static final double PORT_HEIGHT = 2.5; // Meters
        public static final double HEIGHT_DIFFERENCE = PORT_HEIGHT - LIMELIGHT_HEIGHT;
    }

}
