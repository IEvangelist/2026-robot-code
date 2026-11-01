// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.framework.RobotBase;

/**
 * Starts WPILib's robot lifecycle. Keep hardware initialization in Robot and RobotContainer, not
 * here.
 */
public final class Main {
  private Main() {}

  /**
   * Hands robot construction to WPILib so its runtime is initialized before hardware is created.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
