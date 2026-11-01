// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import org.wpilib.command2.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

/**
 * Spins the flywheel to the requested RPM, leaving its closed-loop request active on completion.
 */
public class ShooterRevUp extends Command {

  ShooterSubsystem shooterSubsystem;
  TransitionSubsystem transitionSubsystem;
  double targetRPM;

  /** Creates a new ShooterRevUp. */
  public ShooterRevUp(
      ShooterSubsystem shooterSubsystem,
      TransitionSubsystem transitionSubsystem,
      double targetRPM) {
    this.shooterSubsystem = shooterSubsystem;
    this.transitionSubsystem = transitionSubsystem;

    this.targetRPM = targetRPM;
    // Reserve the feed mechanism as well so other requiring commands cannot feed during spin-up.
    addRequirements(shooterSubsystem, transitionSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooterSubsystem.setRPM(targetRPM);
  }

  // Intentionally keep the flywheel request active for the following command.
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return shooterSubsystem.withinErrorRPM(targetRPM);
  }
}
