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
import frc.robot.command.ZoneCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

/** Blue outpost-side route: score, sweep the neutral zone, and return to the scoring pose. */
public class BlueOBackCommand extends SequentialCommandGroup {
  private CommandSwerveDrivetrain driveSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private TransitionSubsystem transitionSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private ClimbSubsystem climbSubsystem;

  public BlueOBackCommand(
      IntakeSubsystem intake,
      ClimbSubsystem climb,
      CommandSwerveDrivetrain drive,
      ShooterSubsystem shooter,
      TransitionSubsystem transition) {

    addCommands(
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.BLUE_OUTPOST_SCORE),
        // Timed shooting stops when the wait deadline interrupts the targeting command.
        new ParallelDeadlineGroup(
            new WaitCommand(3),
            new TargetCommand(intake, transition, shooter, () -> drive.getState())),
        new InstantCommand(() -> shooter.setRPM(0)),
        new InstantCommand(() -> intake.intakeOut()),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.NEUTRAL_BO),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.RD_FAR_EDGE),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.NEUTRAL_BO_EDGE),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.BLUE_OUTPOST_SCORE),
        new ParallelDeadlineGroup(
            new WaitCommand(5),
            new TargetCommand(intake, transition, shooter, () -> drive.getState())),
        new InstantCommand(() -> shooter.setRPM(0)),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.NEUTRAL_BO),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.RD_FAR_EDGE),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.NEUTRAL_BO_EDGE),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.BLUE_OUTPOST_SCORE),
        // Keep targeting for the remainder of autonomous.
        new TargetCommand(intake, transition, shooter, () -> drive.getState()));
  }
}
