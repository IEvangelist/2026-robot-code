// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.Alliance;

/** Field regions used to choose shooting or passing behavior. */
public record Zones() {
  private static final class AreaList {
    // Zone origins are the minimum-X, maximum-Y corners of their rectangles.
    private static final class RED_ALLIANCE {
      private static final Translation2d TOP_LEFT =
          new Translation2d(Inches.of(469.11).in(Meters), Inches.of(317.69).in(Meters));

      private static final int WIDTH = (int) Inches.of(651.22 - 469.11).in(Meters);
      private static final int HEIGHT = (int) Inches.of(317.69).in(Meters);

      // new Translation2d(Inches.of(651.22).in(Meters), Inches.of(317.69).in(Meters))
      // new Translation2d(Inches.of(651.22).in(Meters), Inches.of(0).in(Meters)),
      // new Translation2d(Inches.of(469.11).in(Meters), Inches.of(0).in(Meters)),
      // new Translation2d(Inches.of(469.11).in(Meters), Inches.of(317.69).in(Meters))
    }
    ;

    private static final class BLUE_ALLIANCE {
      private static final Translation2d TOP_LEFT =
          new Translation2d(Inches.of(0).in(Meters), Inches.of(317.69).in(Meters));

      private static final int WIDTH = (int) Inches.of(182.11).in(Meters);
      private static final int HEIGHT = (int) Inches.of(317.69).in(Meters);

      // new Translation2d(Inches.of(182.11).in(Meters), Inches.of(317.69).in(Meters)),
      // new Translation2d(Inches.of(182.11).in(Meters), Inches.of(0).in(Meters)),
      // new Translation2d(Inches.of(0).in(Meters), Inches.of(0).in(Meters)),
      // new Translation2d(Inches.of(0).in(Meters), Inches.of(317.69).in(Meters))
    }
    ;

    private static final class NEUTRAL_HIGH {
      private static final Translation2d TOP_LEFT =
          new Translation2d(Inches.of(182.11).in(Meters), Inches.of(317.69).in(Meters));

      private static final int WIDTH = (int) Inches.of(469.11 - 182.11).in(Meters);
      private static final int HEIGHT = (int) Inches.of(317.69 / 2.0).in(Meters);
      // new Translation2d(Inches.of(469.11).in(Meters), Inches.of(317.69).in(Meters)),
      // new Translation2d(Inches.of(469.11).in(Meters), Inches.of(0).in(Meters)),
      // new Translation2d(Inches.of(182.11).in(Meters), Inches.of(0).in(Meters)),
      // new Translation2d(Inches.of(182.11).in(Meters), Inches.of(317.69).in(Meters))

    }
    ;

    private static final class NEUTRAL_LOW {
      private static final Translation2d TOP_LEFT =
          new Translation2d(Inches.of(182.11).in(Meters), Inches.of(317.69 / 2.0).in(Meters));

      private static final int WIDTH = (int) Inches.of(469.11 - 182.11).in(Meters);
      private static final int HEIGHT = (int) Inches.of(317.69 / 2.0).in(Meters);
      // new Translation2d(Inches.of(469.11).in(Meters), Inches.of(317.69).in(Meters)),
      // new Translation2d(Inches.of(469.11).in(Meters), Inches.of(0).in(Meters)),
      // new Translation2d(Inches.of(182.11).in(Meters), Inches.of(0).in(Meters)),
      // new Translation2d(Inches.of(182.11).in(Meters), Inches.of(317.69).in(Meters))

    }
    ;
  }

  // Alliance-relative regions are selected at class initialization, defaulting to blue.
  public static Zone allianceZone =
      org.wpilib.driverstation.MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.BLUE
          ? new Zone(
              AreaList.BLUE_ALLIANCE.TOP_LEFT,
              AreaList.BLUE_ALLIANCE.WIDTH,
              AreaList.BLUE_ALLIANCE.HEIGHT)
          : new Zone(
              AreaList.RED_ALLIANCE.TOP_LEFT,
              AreaList.RED_ALLIANCE.WIDTH,
              AreaList.RED_ALLIANCE.HEIGHT);

  public static Zone opposingZoneHigh =
      org.wpilib.driverstation.MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.BLUE
          ? new Zone(
              AreaList.RED_ALLIANCE.TOP_LEFT,
              AreaList.RED_ALLIANCE.WIDTH,
              AreaList.RED_ALLIANCE.HEIGHT / 2)
          : new Zone(
              AreaList.BLUE_ALLIANCE.TOP_LEFT,
              AreaList.BLUE_ALLIANCE.WIDTH,
              AreaList.BLUE_ALLIANCE.HEIGHT / 2);

  public static Zone opposingZoneLow =
      org.wpilib.driverstation.MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.BLUE
          ? new Zone(
              new Translation2d(
                  469.11, AreaList.RED_ALLIANCE.TOP_LEFT.getY() - AreaList.RED_ALLIANCE.HEIGHT / 2),
              AreaList.RED_ALLIANCE.WIDTH,
              AreaList.RED_ALLIANCE.HEIGHT)
          : new Zone(
              new Translation2d(0, AreaList.RED_ALLIANCE.HEIGHT / 2),
              AreaList.BLUE_ALLIANCE.WIDTH,
              AreaList.BLUE_ALLIANCE.HEIGHT);

  public static Zone neutralHighZone =
      new Zone(
          AreaList.NEUTRAL_HIGH.TOP_LEFT,
          AreaList.NEUTRAL_HIGH.WIDTH,
          AreaList.NEUTRAL_HIGH.HEIGHT);
  public static Zone neutralLowZone =
      new Zone(
          AreaList.NEUTRAL_LOW.TOP_LEFT, AreaList.NEUTRAL_LOW.WIDTH, AreaList.NEUTRAL_LOW.HEIGHT);
}
