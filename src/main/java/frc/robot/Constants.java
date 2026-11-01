// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Shared tuning values, field landmarks, and mechanism presets. Motor-specific configuration lives
 * with its subsystem.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    // Maximum change in commanded translation speed per second (m/s^2).
    public static final double DRIVE_SLEW_RATE = 10.0;
  }

  /** Illustrative settings for the standalone simulation, not calibrated real-robot values. */
  public static final class SimDriveConstants {
    public static final double MAX_SPEED_METERS_PER_SECOND = 2.0;
    public static final double LOOP_PERIOD_SECONDS = 0.020;
    public static final double HALF_WHEELBASE_METERS = 0.30;
    public static final double HALF_TRACK_WIDTH_METERS = 0.30;
    public static final double WHEEL_RADIUS_METERS = 0.0508;

    // Motor rotor rotations per mechanism output rotation.
    public static final double DRIVE_GEAR_RATIO = 6.75;
    public static final double STEER_GEAR_RATIO = 12.8;
    public static final double DRIVE_INERTIA_KG_METERS_SQUARED = 0.025;
    public static final double STEER_INERTIA_KG_METERS_SQUARED = 0.025;

    // Steering voltage per radian of error and per radian/second of angular velocity.
    public static final double STEER_KP = 8.0;
    public static final double STEER_KD = 0.5;
    public static final double STATIONARY_SPEED_METERS_PER_SECOND = 1e-6;

    // Desktop demo/controller settings; no real Driver Station or hardware is required.
    public static final double CONTROLLER_DEADBAND = 0.08;
    public static final double MAX_ANGULAR_RATE_RADIANS_PER_SECOND = 1.0;
    public static final double DEMO_SPEED_METERS_PER_SECOND = 0.5;
    public static final double DEMO_ANGULAR_RATE_RADIANS_PER_SECOND = 0.5;
    public static final double DEMO_PHASE_SECONDS = 3.0;
    public static final double START_X_METERS = 3.0;
    public static final double START_Y_METERS = 3.0;

    // Reserved on the simulation's default CAN bus; do not reuse in the same process.
    public static final int FRONT_LEFT_DRIVE_CAN_ID = 41;
    public static final int FRONT_LEFT_STEER_CAN_ID = 42;
    public static final int FRONT_RIGHT_DRIVE_CAN_ID = 43;
    public static final int FRONT_RIGHT_STEER_CAN_ID = 44;
    public static final int BACK_LEFT_DRIVE_CAN_ID = 45;
    public static final int BACK_LEFT_STEER_CAN_ID = 46;
    public static final int BACK_RIGHT_DRIVE_CAN_ID = 47;
    public static final int BACK_RIGHT_STEER_CAN_ID = 48;
  }

  public static class AlignConstants {
    // Position tolerances use meters; heading tolerances use degrees.
    public static double ALIGN_P = 2.0;
    public static double MAX_VELOCITY = 2.5;
    public static double MAX_ACCELERATION = 4.5;
    public static double VISION_ERROR_ACCOUNT = 0.15;

    public static double MAX_POSITION_ERROR_METERS = 0.08;
    public static double MAX_DEGREES_ERROR = 0.5;

    public static double MAX_POSITION_ERROR_METERS_AUTO = 0.2;
  }

  public static class AutoConstants {
    public static final double AUTO_MAX_VELOCITY_METERS_PER_SECOND = 4.0; // TODO update value
    public static final double AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED =
        5.0; // TODO update value
    public static final double AUTO_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC =
        2 * Math.PI; // TODO update value
    public static final double AUTO_MAX_ANGULAR_VELOCITY_RAD_PER_SEC =
        2 * Math.PI; // TODO update value

    public static final double X_DRIVE_P = 2.05;
    public static final double X_DRIVE_I = 0;
    public static final double X_DRIVE_D = 0.1;

    public static final double Y_DRIVE_P = 2.05;
    public static final double Y_DRIVE_I = 0;
    public static final double Y_DRIVE_D = 0.1;

    public static final double THETA_P = 5.0;
    public static final double THETA_I = 0;
    public static final double THETA_D = 0;
  }

  public static class TrajectoryConstants {

    // Blue-origin field frame: +X toward red, +Y toward the high-Y field edge.
    // Coordinates below are entered in inches; WPILib stores translations in meters.
    // Heading is 0 degrees with the intake facing red, and 90 degrees facing +Y.
    // Robot footprint: 36.125 inches intake-to-shooter, 31.125 inches side-to-side.

    public static final Pose2d BLUE_OUTPOST =
        new Pose2d(
            new Translation2d(Inches.of(0.5), Inches.of(25.62)), new Rotation2d(Degrees.of(180)));
    public static final Pose2d RED_OUTPOST =
        new Pose2d(
            new Translation2d(Inches.of(649.62), Inches.of(291.02)), new Rotation2d(Degrees.of(0)));

    // public static final Pose2d BLUE_TOWER_DEPOT = new Pose2d (new Translation2d(Inches.of(45.0),
    // Inches.of(146.86)), new Rotation2d(Degrees.of(180)));
    // public static final Pose2d RED_TOWER_DEPOT = new Pose2d (new Translation2d(Inches.of(605.12),
    // Inches.of(169.78)), new Rotation2d(Degrees.of(360)));

    // public static final Pose2d BLUE_TOWER_OUTPOST = new Pose2d (new
    // Translation2d(Inches.of(45.0), Inches.of(146.86)), new Rotation2d(Degrees.of(180)));
    // public static final Pose2d RED_TOWER_OUTPOST = new Pose2d (new
    // Translation2d(Inches.of(605.12), Inches.of(169.78)), new Rotation2d(Degrees.of(360)));

    // public static final Pose2d BLUE_ALLIANCE = new Pose2d (new
    // Translation2d(Inches.of(60.0).in(Meters), Inches.of(146.86).in(Meters)), new
    // Rotation2d(Degrees.of(180)));
    // public static final Pose2d RED_ALLIANCE = new Pose2d (new
    // Translation2d(Inches.of(590.12).in(Meters), Inches.of(169.78).in(Meters)), new
    // Rotation2d(Degrees.of(360)));

    // public static final Pose2d BLUE_DEPOT_SIDE = new Pose2d (new Translation2d(Inches.of(60.0),
    // Inches.of(146.86)), new Rotation2d(Degrees.of(180)));
    // public static final Pose2d RED_DEPOT_SIDE = new Pose2d (new Translation2d(Inches.of(590.12),
    // Inches.of(169.78)), new Rotation2d(Degrees.of(360)));

    // public static final Pose2d BLUE_OUTPOST_SIDE = new Pose2d (new Translation2d(Inches.of(60.0),
    // Inches.of(146.86)), new Rotation2d(Degrees.of(180)));
    // public static final Pose2d RED_OUTPOST_SIDE = new Pose2d (new
    // Translation2d(Inches.of(590.12), Inches.of(169.78)), new Rotation2d(Degrees.of(360)));

    public static final Pose2d NEUTRAL_BO =
        new Pose2d(
            new Translation2d(Inches.of(289.11), Inches.of(97.37)), new Rotation2d(Degrees.of(0)));
    public static final Pose2d NEUTRAL_BD =
        new Pose2d(
            new Translation2d(Inches.of(289.11), Inches.of(219.27)), new Rotation2d(Degrees.of(0)));
    public static final Pose2d NEUTRAL_RO =
        new Pose2d(
            new Translation2d(Inches.of(361.01), Inches.of(219.27)),
            new Rotation2d(Degrees.of(180)));
    public static final Pose2d NEUTRAL_RD =
        new Pose2d(
            new Translation2d(Inches.of(361.01), Inches.of(97.37)),
            new Rotation2d(Degrees.of(180)));

    public static final Pose2d RD_FAR_EDGE =
        new Pose2d(
            new Translation2d(Inches.of(325.06), Inches.of(85.0)), new Rotation2d(Degrees.of(90)));
    public static final Pose2d NEUTRAL_RD_EDGE =
        new Pose2d(
            new Translation2d(Inches.of(361.01), Inches.of(100.0)), new Rotation2d(Degrees.of(0)));

    public static final Pose2d RO_FAR_EDGE =
        new Pose2d(
            new Translation2d(Inches.of(325.06), Inches.of(235)), new Rotation2d(Degrees.of(270)));
    public static final Pose2d NEUTRAL_RO_EDGE =
        new Pose2d(
            new Translation2d(Inches.of(361.01), Inches.of(220)), new Rotation2d(Degrees.of(0)));

    public static final Pose2d NEUTRAL_BO_EDGE =
        new Pose2d(
            new Translation2d(Inches.of(289.11), Inches.of(100.0)),
            new Rotation2d(Degrees.of(180)));
    public static final Pose2d NEUTRAL_BD_EDGE =
        new Pose2d(
            new Translation2d(Inches.of(289.11), Inches.of(220.0)),
            new Rotation2d(Degrees.of(180)));

    public static final Pose2d CENTER_FACE_RED =
        new Pose2d(
            new Translation2d(Inches.of(325.06), Inches.of(158.32)), new Rotation2d(Degrees.of(0)));
    public static final Pose2d CENTER_FACE_BLUE =
        new Pose2d(
            new Translation2d(Inches.of(325.06), Inches.of(158.32)),
            new Rotation2d(Degrees.of(180)));
    public static final Pose2d CENTER_FACE_BORD =
        new Pose2d(
            new Translation2d(Inches.of(325.06), Inches.of(158.32)),
            new Rotation2d(Degrees.of(270)));
    public static final Pose2d CENTER_FACE_BDRO =
        new Pose2d(
            new Translation2d(Inches.of(325.06), Inches.of(158.32)),
            new Rotation2d(Degrees.of(90)));

    public static final Pose2d BLUE_BUMP_O_L =
        new Pose2d(
            new Translation2d(Inches.of(181.56), Inches.of(98.06)),
            new Rotation2d(Degrees.of(315)));
    public static final Pose2d BLUE_BUMP_D_L =
        new Pose2d(
            new Translation2d(Inches.of(181.56), Inches.of(218.58)),
            new Rotation2d(Degrees.of(45)));
    public static final Pose2d RED_BUMP_O_L =
        new Pose2d(
            new Translation2d(Inches.of(468.56), Inches.of(218.58)),
            new Rotation2d(Degrees.of(135)));
    public static final Pose2d RED_BUMP_D_L =
        new Pose2d(
            new Translation2d(Inches.of(468.56), Inches.of(98.06)),
            new Rotation2d(Degrees.of(225)));

    public static final Pose2d BLUE_BUMP_O_R =
        new Pose2d(
            new Translation2d(Inches.of(181.56), Inches.of(98.06)),
            new Rotation2d(Degrees.of(225)));
    public static final Pose2d BLUE_BUMP_D_R =
        new Pose2d(
            new Translation2d(Inches.of(181.56), Inches.of(218.58)),
            new Rotation2d(Degrees.of(135)));
    public static final Pose2d RED_BUMP_O_R =
        new Pose2d(
            new Translation2d(Inches.of(468.56), Inches.of(218.58)),
            new Rotation2d(Degrees.of(45)));
    public static final Pose2d RED_BUMP_D_R =
        new Pose2d(
            new Translation2d(Inches.of(468.56), Inches.of(98.06)),
            new Rotation2d(Degrees.of(315)));

    public static final Pose2d RED_CLIMB_DEPOT_FINAL =
        new Pose2d(
            new Translation2d(Inches.of(610), Inches.of(136)), new Rotation2d(Degrees.of(0)));
    public static final Pose2d RED_CLIMB_DEPOT =
        new Pose2d(
            new Translation2d(Inches.of(619), Inches.of(136)), new Rotation2d(Degrees.of(0)));
    public static final Pose2d RED_CLIMB_DEPOT_OFFSET =
        new Pose2d(
            new Translation2d(Inches.of(585.12), Inches.of(132)), new Rotation2d(Degrees.of(0)));

    public static final Translation2d RED_HUB_GOAL =
        new Translation2d(
            Inches.of(468.565), Inches.of(158.32)); // wrong x values, formerly 414.785
    public static final Translation2d BLUE_HUB_GOAL =
        new Translation2d(
            Inches.of(181.555), Inches.of(158.32)); // wrong x values, formerly 235.335

    public static final Pose2d RED_DEPOT_SCORE =
        new Pose2d(
            new Translation2d(Inches.of(520), Inches.of(98)), new Rotation2d(Degrees.of(315)));
    public static final Pose2d RED_OUTPOST_SCORE =
        new Pose2d(
            new Translation2d(Inches.of(520), Inches.of(218)), new Rotation2d(Degrees.of(45)));

    public static final Pose2d BLUE_PRE_SCORE =
        new Pose2d(
            new Translation2d(Inches.of(130), Inches.of(158)), new Rotation2d(Degrees.of(180)));

    public static final Pose2d RED_PRE_SCORE =
        new Pose2d(
            new Translation2d(Inches.of(520), Inches.of(158)), new Rotation2d(Degrees.of(0)));

    public static final Pose2d BLUE_OUTPOST_SCORE =
        new Pose2d(
            new Translation2d(Inches.of(130), Inches.of(98)), new Rotation2d(Degrees.of(225)));
    public static final Pose2d BLUE_DEPOT_SCORE =
        new Pose2d(
            new Translation2d(Inches.of(130), Inches.of(218)), new Rotation2d(Degrees.of(135)));

    public static final Pose2d RED_D_WALL =
        new Pose2d(new Translation2d(Inches.of(570), Inches.of(35)), new Rotation2d(Degrees.of(0)));

    public static final Pose2d RED_D_TRENCH =
        new Pose2d(
            new Translation2d(Inches.of(468.56), Inches.of(24.5)), new Rotation2d(Degrees.of(180)));
    public static final Pose2d RED_O_TRENCH =
        new Pose2d(
            new Translation2d(Inches.of(468.56), Inches.of(292.5)),
            new Rotation2d(Degrees.of(180)));
    public static final Pose2d BLUE_D_TRENCH =
        new Pose2d(
            new Translation2d(Inches.of(181.56), Inches.of(292.5)), new Rotation2d(Degrees.of(0)));
    public static final Pose2d BLUE_O_TRENCH =
        new Pose2d(
            new Translation2d(Inches.of(181.56), Inches.of(24.5)), new Rotation2d(Degrees.of(0)));

    // public static final Pose2d TEST = new Pose2d (new Translation2d(Inches.of(570),
    // Inches.of(98)), new Rotation2d(Degrees.of(90)));

    public static final Pose2d RED_DEPOT =
        new Pose2d(
            new Translation2d(Inches.of(614.0), Inches.of(82.32)), new Rotation2d(Degrees.of(0)));
    public static final Pose2d BLUE_DEPOT =
        new Pose2d(
            new Translation2d(Inches.of(36.0), Inches.of(234.32)), new Rotation2d(Degrees.of(180)));

    public static final Pose2d RED_DEPOT_OFFSET =
        new Pose2d(
            new Translation2d(Inches.of(594.0), Inches.of(82.32)), new Rotation2d(Degrees.of(0)));
    public static final Pose2d BLUE_DEPOT_OFFSET =
        new Pose2d(
            new Translation2d(Inches.of(56.0), Inches.of(234.32)), new Rotation2d(Degrees.of(180)));
  }

  public static class ClimbPositions {
    public static final double L1_INCHES = 6.9;
    public static final double MATCH_START_INCHES = 0;
  }

  public static class IntakePositions {
    // Pivot presets are degrees; motor speeds are signed duty-cycle fractions.
    public static final double INTAKE_RETRACTED_DEGREES = 0;
    public static final double INTAKE_EXTENDED_DEGREES = 90;
    public static final double INTAKE_SPEED = 0.82;
    public static final double INTAKE_PIVOT_SPEED = 0.35;
    public static final double INTAKE_PIVOT_SPEED_IN = 0.20;
  }

  public static class TransitionConstants {
    // Duty-cycle outputs for the hopper and shooter feed rollers.
    public static final double TRANSITION_HOPPER_SPEED = 0.5;
    public static final double TRANSITION_SHOOTER_SPEED = 1.0;
  }

  public static class ShooterConstants {
    public static final double SHOOTER_RPM = 3500;
    public static final double ACTUATOR_PASSING = 0.38;
    public static final double ACTUATOR_SHOOTING = 0.02;

    // Shoot-on-the-move model: seconds, meters, and degrees.
    public static final double LATENCY_SEC = 0.10;
    public static final double WHEEL_DIAMETER = 0.1016; // METERS
    public static final double HOOD_ANGLE_SHOOT = 69.5; // DEGREES
    public static final double HOOD_ANGLE_PASS_NEU_CLOSE = 69.5; // DEGREES
    public static final double HOOD_ANGLE_PASS_NEU_FAR = 69.5; // DEGREES
    public static final double HOOD_ANGLE_PASS_OPP_ALL = 55.0; // DEGREES
    public static final double HOOD_ANGLE_START = 40.0;
  }
}
