// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import org.wpilib.command2.Command;
import frc.robot.Constants.TransitionConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

/** Feeds continuously at a fixed turret angle and RPM until cancelled. */
public class FixedCommand extends Command {
  ShooterSubsystem shooterSubsystem;
  TransitionSubsystem transitionSubsystem;
  Supplier<SwerveDriveState> currentState;
  IntakeSubsystem intakeSubsystem;

  /** Connects the mechanisms used by the fixed-shot driver control. */
  public FixedCommand(
      IntakeSubsystem intakeSubsystem,
      TransitionSubsystem transitionSubsystem,
      ShooterSubsystem shooterSubsystem,
      Supplier<SwerveDriveState> currentState) {

    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.transitionSubsystem = transitionSubsystem;
    this.currentState = currentState;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    System.out.println("fixed :(");

    // shooterSubsystem.updateSOTM(currentState.get().Pose, currentState.get().Velocity);
    shooterSubsystem.turnShooter(0.0);
    shooterSubsystem.setRPM(2600);

    // Feeding starts immediately; this mode does not wait for flywheel speed.
    transitionSubsystem.shooterTransition(
        TransitionConstants.TRANSITION_SHOOTER_SPEED, TransitionConstants.TRANSITION_HOPPER_SPEED);
  }

  // Release all shooting outputs when the driver releases the control.
  @Override
  public void end(boolean interrupted) {
    transitionSubsystem.shooterTransition(0.0, 0.0);
    shooterSubsystem.stopShooter();
    shooterSubsystem.stopShooterTurn();
  }

  // Driver release or a containing autonomous deadline cancels this continuous command.
  @Override
  public boolean isFinished() {
    return false;
  }
}
