// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.autoCommands;

import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.ParallelDeadlineGroup;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.WaitCommand;
import frc.robot.Constants.IntakePositions;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.command.TargetCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

/** Delayed blue depot routine that alternates timed shots with depot collection trips. */
public class BlueDepotCommand extends SequentialCommandGroup {
  private CommandSwerveDrivetrain driveSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private TransitionSubsystem transitionSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private ClimbSubsystem climbSubsystem;

  public BlueDepotCommand(
      IntakeSubsystem intake,
      ClimbSubsystem climb,
      CommandSwerveDrivetrain drive,
      ShooterSubsystem shooter,
      TransitionSubsystem transition) {

    addCommands(
        new WaitCommand(3),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.BLUE_DEPOT_SCORE),
        // A wait deadline bounds each shooting window and triggers targeting cleanup.
        new ParallelDeadlineGroup(
            new WaitCommand(3),
            new TargetCommand(intake, transition, shooter, () -> drive.getState())),
        new InstantCommand(() -> shooter.setRPM(0)),
        new InstantCommand(() -> intake.intakeOut()),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.BLUE_DEPOT_OFFSET),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.BLUE_DEPOT),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.BLUE_DEPOT_OFFSET),
        new ParallelDeadlineGroup(
            new WaitCommand(5),
            new TargetCommand(intake, transition, shooter, () -> drive.getState())),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.BLUE_DEPOT_OFFSET),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.BLUE_DEPOT),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.BLUE_DEPOT_OFFSET),
        // Continue targeting until autonomous is cancelled.
        new TargetCommand(intake, transition, shooter, () -> drive.getState()));
  }
}
