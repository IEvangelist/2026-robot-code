// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.autoCommands;

import java.io.Console;

import org.wpilib.command2.Command;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.IntakePositions;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.command.ShooterRevUp;
import frc.robot.command.TargetCommand;
import frc.robot.command.ZoneCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

/** Blue preload routine: target and shoot from the starting pose without driving. */
public class BluePreCommand extends SequentialCommandGroup {
  private CommandSwerveDrivetrain driveSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private TransitionSubsystem transitionSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private ClimbSubsystem climbSubsystem;

  public BluePreCommand(
      IntakeSubsystem intake,
      ClimbSubsystem climb,
      CommandSwerveDrivetrain drive,
      ShooterSubsystem shooter,
      TransitionSubsystem transition) {

    addCommands(
        // drive.profiledAutonAlignCommand(() -> TrajectoryConstants.BLUE_PRE_SCORE),
        // TargetCommand is continuous; the robot mode transition ends this routine.
        new TargetCommand(intake, transition, shooter, () -> drive.getState()));
  }
}
