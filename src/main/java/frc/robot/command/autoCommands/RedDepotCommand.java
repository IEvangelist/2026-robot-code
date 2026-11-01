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

/** Delayed red depot routine with collection trips and stationary timed shooting windows. */
public class RedDepotCommand extends SequentialCommandGroup {
  private CommandSwerveDrivetrain driveSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private TransitionSubsystem transitionSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private ClimbSubsystem climbSubsystem;

  public RedDepotCommand(
      IntakeSubsystem intake,
      ClimbSubsystem climb,
      CommandSwerveDrivetrain drive,
      ShooterSubsystem shooter,
      TransitionSubsystem transition) {

    addCommands(
        new WaitCommand(2),
        new InstantCommand(() -> intake.intakeOut()),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.RED_DEPOT_SCORE),
        // The wait deadline ends targeting and the continuous stationary drive request.
        new ParallelDeadlineGroup(
            new WaitCommand(3),
            // This continuous hold prevents the following targeting command from starting.
            drive.stopAlign(),
            new TargetCommand(intake, transition, shooter, () -> drive.getState())),
        new InstantCommand(() -> shooter.setRPM(0)),
        new InstantCommand(() -> intake.intakeOut()),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.RED_DEPOT_OFFSET),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.RED_DEPOT),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.RED_DEPOT_OFFSET),
        new ParallelDeadlineGroup(
            new WaitCommand(5),
            drive.stopAlign(),
            new TargetCommand(intake, transition, shooter, () -> drive.getState())),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.RED_DEPOT_OFFSET),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.RED_DEPOT),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.RED_DEPOT_OFFSET),
        drive.stopAlign(),
        new TargetCommand(intake, transition, shooter, () -> drive.getState()));
  }
}
