// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import org.wpilib.command2.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ClimbSubsystem.ClimbConstants;

/** Moves the climber to a height in inches, then stops the motor at the configured tolerance. */
public class ClimbCommand extends Command {
  ClimbSubsystem climbSubsystem;
  double setPosition;

  /** Creates a new ClimbCommand. */
  public ClimbCommand(ClimbSubsystem climbSubsystem, double position) {
    this.climbSubsystem = climbSubsystem;
    setPosition = position;
    addRequirements(climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // TalonFX position requests use motor rotations, while climb presets use inches.
    climbSubsystem.climbMotor.setControl(
        climbSubsystem.voltageRequest.withPosition(
            setPosition / ClimbConstants.CLIMBER_INCHES_PER_ROTATION));
  }

  // Stop on both normal completion and interruption.
  @Override
  public void end(boolean interrupted) {
    climbSubsystem.stopClimbMotors();
  }

  @Override
  public boolean isFinished() {
    return climbSubsystem.withinError(setPosition);
  }
}
