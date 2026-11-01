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

/** Blue depot-side route: score, collect in the neutral zone, and return for more shots. */
public class BlueDBackCommand extends SequentialCommandGroup {
  private CommandSwerveDrivetrain driveSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private TransitionSubsystem transitionSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private ClimbSubsystem climbSubsystem;

  public BlueDBackCommand(
      IntakeSubsystem intake,
      ClimbSubsystem climb,
      CommandSwerveDrivetrain drive,
      ShooterSubsystem shooter,
      TransitionSubsystem transition) {

    addCommands(
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.BLUE_DEPOT_SCORE),
        // The wait is the deadline: its completion interrupts targeting and stops shooting.
        new ParallelDeadlineGroup(
            new WaitCommand(3),
            new TargetCommand(intake, transition, shooter, () -> drive.getState())),
        new InstantCommand(() -> shooter.setRPM(0)),
        new InstantCommand(() -> intake.intakeOut()),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.NEUTRAL_BD),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.RO_FAR_EDGE),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.NEUTRAL_BD_EDGE),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.BLUE_DEPOT_SCORE),
        new ParallelDeadlineGroup(
            new WaitCommand(5),
            new TargetCommand(intake, transition, shooter, () -> drive.getState())),
        new InstantCommand(() -> shooter.setRPM(0)),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.NEUTRAL_BD),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.RO_FAR_EDGE),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.NEUTRAL_BD_EDGE),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.BLUE_DEPOT_SCORE),
        // The final targeting command runs until autonomous is cancelled.
        new TargetCommand(intake, transition, shooter, () -> drive.getState()));
  }
}
