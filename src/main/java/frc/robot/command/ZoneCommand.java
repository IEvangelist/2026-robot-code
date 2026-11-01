// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import java.rmi.server.ServerCloneException;
import java.util.concurrent.TransferQueue;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.command2.Command;
import frc.robot.Constants;
import frc.robot.Zones;
import frc.robot.Constants.IntakePositions;
import frc.robot.Constants.TransitionConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

/** Chooses hub shooting or high/low passing targets from the robot's current field zone. */
public class ZoneCommand extends Command {

  ShooterSubsystem shooterSubsystem;
  TransitionSubsystem transitionSubsystem;
  Supplier<SwerveDriveState> currentState;
  IntakeSubsystem intakeSubsystem;

  /** Creates a new ZoneCommand. */
  public ZoneCommand(
      IntakeSubsystem intakeSubsystem,
      TransitionSubsystem transitionSubsystem,
      ShooterSubsystem shooterSubsystem,
      Supplier<SwerveDriveState> currentState) {
    // addRequirements(intakeSubsystem, transitionSubsystem, shooterSubsystem);

    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.transitionSubsystem = transitionSubsystem;
    this.currentState = currentState;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // System.out.println(Zones.allianceZone.isPointInZone(
    //   currentState.get().Pose) + "::" +
    //   (Zones.neutralZone.isPointInZone(currentState.get().Pose) ||
    // Zones.opposingZone.isPointInZone(currentState.get().Pose)));
    // First matching zone wins when inclusive rectangle boundaries overlap.
    if (Zones.allianceZone.isPointInZone(currentState.get().Pose)) {
      System.out.println("alliance zone");
      intakeSubsystem.intakeOut();
      shooterSubsystem.updateSOTM(currentState.get().Pose, currentState.get().Velocity);
      transitionSubsystem.shooterTransition(
          TransitionConstants.TRANSITION_SHOOTER_SPEED,
          TransitionConstants.TRANSITION_HOPPER_SPEED);

    } else if (Zones.neutralHighZone.isPointInZone(currentState.get().Pose)
        || Zones.opposingZoneHigh.isPointInZone(currentState.get().Pose)) {
      System.out.println("neutral high");
      intakeSubsystem.intakeOut();
      shooterSubsystem.updateHighPOTM(currentState.get().Pose, currentState.get().Velocity);
      transitionSubsystem.shooterTransition(
          TransitionConstants.TRANSITION_SHOOTER_SPEED,
          TransitionConstants.TRANSITION_HOPPER_SPEED);
    } else if (Zones.neutralLowZone.isPointInZone(currentState.get().Pose)
        || Zones.opposingZoneLow.isPointInZone(currentState.get().Pose)) {
      System.out.println("neutral low");
      intakeSubsystem.intakeOut();
      shooterSubsystem.updateLowPOTM(currentState.get().Pose, currentState.get().Velocity);
      transitionSubsystem.shooterTransition(
          TransitionConstants.TRANSITION_SHOOTER_SPEED,
          TransitionConstants.TRANSITION_HOPPER_SPEED);
    } else {
      System.out.println("No zone");
    }
  }

  // Stop collection and shooting, then return the hood to its start preset.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntake();
    intakeSubsystem.stopIntakePivot();
    transitionSubsystem.shooterTransition(0.0, 0.0);
    shooterSubsystem.stopShooter();
    shooterSubsystem.stopShooterTurn();
    shooterSubsystem.setHoodAngle(Constants.ShooterConstants.HOOD_ANGLE_START);
  }

  // Zone selection continues until the caller cancels the command.
  @Override
  public boolean isFinished() {
    return false;
  }
}
