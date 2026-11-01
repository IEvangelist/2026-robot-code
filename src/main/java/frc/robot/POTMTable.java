// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.wpilib.units.Units.*;

import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.math.interpolation.InterpolatingTreeMap;

/**
 * Separate long-range distance-to-RPM samples, currently unused by the shooter targeting methods.
 */
public class POTMTable {
  private static final InterpolatingDoubleTreeMap velocityMapRPM = new InterpolatingDoubleTreeMap();

  static {
    // key = Distance (m); value = Velocity (RPM)
    velocityMapRPM.put(Inches.of(167).in(Meters), 4236.0);
    velocityMapRPM.put(Inches.of(239).in(Meters), 5101.0);
    velocityMapRPM.put(Inches.of(400).in(Meters), 5541.0);
  }

  /**
   * Interpolates the stored RPM samples; distances outside the table use the nearest endpoint.
   * Callers must use the hood setting for which these samples were calibrated.
   *
   * @param distanceFromGoal distance to the target in meters
   * @return flywheel speed in RPM
   */
  public static double getSpeed(double distanceFromGoal) {
    return velocityMapRPM.get(distanceFromGoal);
  }
}
