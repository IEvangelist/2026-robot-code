// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.wpilib.units.Units.*;

import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.math.interpolation.InterpolatingTreeMap;

/** Distance-to-flywheel-RPM samples for fixed-hood shooting. */
public class SOTMTable {
  private static final InterpolatingDoubleTreeMap velocityMapRPM = new InterpolatingDoubleTreeMap();

  static {
    // key = Distance (m); value = Velocity (RPM)
    velocityMapRPM.put(Inches.of(55).in(Meters), 2600.0);
    velocityMapRPM.put(Inches.of(76).in(Meters), 2806.0);
    velocityMapRPM.put(Inches.of(91).in(Meters), 3243.0);
    velocityMapRPM.put(Inches.of(104).in(Meters), 3404.0);
    velocityMapRPM.put(Inches.of(132).in(Meters), 3856.0);
    velocityMapRPM.put(Inches.of(160).in(Meters), 4308.0);
    velocityMapRPM.put(Inches.of(180).in(Meters), 4337.0);
    velocityMapRPM.put(Inches.of(207).in(Meters), 4621.0);
  }

  /**
   * Interpolates between samples and uses the nearest endpoint outside the sampled range. Assumes
   * the shooting hood angle remains fixed.
   *
   * @param distanceFromGoal distance to the target in meters
   * @return flywheel speed in RPM
   */
  public static double getSpeed(double distanceFromGoal) {
    return velocityMapRPM.get(distanceFromGoal);
  }
}
