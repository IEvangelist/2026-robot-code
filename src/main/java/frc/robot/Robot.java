// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.system.DataLogManager;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.framework.TimedRobot;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;

/** Connects WPILib mode changes to command scheduling, autonomous selection, and robot logging. */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /** Creates the command bindings and starts robot and Driver Station data logging. */
  public Robot() {
    SystemCoreHardware.validate();
    m_robotContainer = new RobotContainer();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  /** Runs the command scheduler every 20 ms in every robot mode. */
  @Override
  public void robotPeriodic() {
    // Polls bindings, advances commands, and calls each subsystem's periodic method.
    CommandScheduler.getInstance().run();
    Dashboard.update();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.autoVisionGyroReset();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.autoVisionGyroReset();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // Release autonomous control before driver bindings take over.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // m_robotContainer.autoVisionGyroReset();
  }

  @Override
  public void utilityInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void utilityPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
