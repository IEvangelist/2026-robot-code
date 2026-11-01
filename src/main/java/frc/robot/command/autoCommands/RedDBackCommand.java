// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.autoCommands;

import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.ParallelDeadlineGroup;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.IntakePositions;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.command.TargetCommand;
import frc.robot.command.ZoneCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

/** Red depot-side route with timed shots and neutral-zone collection waypoints. */
public class RedDBackCommand extends SequentialCommandGroup {
  private CommandSwerveDrivetrain driveSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private TransitionSubsystem transitionSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private ClimbSubsystem climbSubsystem;

  public RedDBackCommand(
      IntakeSubsystem intake,
      ClimbSubsystem climb,
      CommandSwerveDrivetrain drive,
      ShooterSubsystem shooter,
      TransitionSubsystem transition) {

    addCommands(
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.RED_DEPOT_SCORE),
        // The wait ends both targeting and the continuous zero-velocity drive request.
        new ParallelDeadlineGroup(
            new WaitCommand(3),
            new TargetCommand(intake, transition, shooter, () -> drive.getState()),
            drive.stopAlign()),
        new InstantCommand(() -> shooter.setRPM(0)),
        new InstantCommand(() -> intake.intakeOut()),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.NEUTRAL_RD),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.RD_FAR_EDGE),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.NEUTRAL_RD_EDGE),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.RED_DEPOT_SCORE),
        new ParallelDeadlineGroup(
            new WaitCommand(5),
            new TargetCommand(intake, transition, shooter, () -> drive.getState()),
            drive.stopAlign()),
        new InstantCommand(() -> shooter.setRPM(0)),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.NEUTRAL_RD),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.RD_FAR_EDGE),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.NEUTRAL_RD_EDGE),
        drive.profiledAutonAlignCommand(() -> TrajectoryConstants.RED_DEPOT_SCORE),
        // This continuous command holds here until cancelled; the next command cannot start.
        drive.stopAlign(),
        new TargetCommand(intake, transition, shooter, () -> drive.getState()));
  }
}
