// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import org.wpilib.command2.Command;
import frc.robot.Constants;
import frc.robot.Constants.TransitionConstants;
import frc.robot.Zones;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

/** Continuously aims at the alliance hub and feeds until cancelled or timed out by its caller. */
public class TargetCommand extends Command {

  ShooterSubsystem shooterSubsystem;
  TransitionSubsystem transitionSubsystem;
  Supplier<SwerveDriveState> currentState;
  IntakeSubsystem intakeSubsystem;

  /** Creates a new TargetCommand. */
  public TargetCommand(
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

    System.out.println("targeting :)");
    shooterSubsystem.updateSOTM(currentState.get().Pose, currentState.get().Velocity);
    // Feed runs alongside aiming; there is no RPM or turret-readiness gate here.
    transitionSubsystem.shooterTransition(
        TransitionConstants.TRANSITION_SHOOTER_SPEED, TransitionConstants.TRANSITION_HOPPER_SPEED);
  }

  // Autonomous deadlines and driver release both stop the feed, flywheel, and turret.
  @Override
  public void end(boolean interrupted) {
    transitionSubsystem.shooterTransition(0.0, 0.0);
    shooterSubsystem.stopShooter();
    shooterSubsystem.stopShooterTurn();
  }

  // The driver binding or containing command controls this command's duration.
  @Override
  public boolean isFinished() {
    return false;
  }
}
