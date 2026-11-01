// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.telemetry.TelemetryTable;
import frc.robot.Dashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem.IntakeConstants;

/** Controls the independent hopper and shooter feed rollers. Outputs are duty-cycle fractions. */
public class TransitionSubsystem extends SubsystemBase {
  private TalonFX shooterTransition;
  private TalonFX hopperTransition;
  private TalonFXConfiguration transitionConfig;

  public static class TransitionConstants {

    private static final int SHOOTER_TRANSITION_ID = 20;
    private static final int HOPPER_TRANSITION_ID = 21;

    private static final MotorOutputConfigs TRANSITION_MOTOR_CONFIGS =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive);

    private static final CurrentLimitsConfigs TRANSITION_CURRENT_LIMITS =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(55)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(55)
            .withSupplyCurrentLimitEnable(true);
  }

  public TransitionSubsystem() {
    shooterTransition = new TalonFX(TransitionConstants.SHOOTER_TRANSITION_ID, frc.robot.SystemCoreHardware.auxiliaryBus());
    hopperTransition = new TalonFX(TransitionConstants.HOPPER_TRANSITION_ID, frc.robot.SystemCoreHardware.auxiliaryBus());

    RobotContainer.applyTalonConfigs(shooterTransition, new TalonFXConfiguration());
    RobotContainer.applyTalonConfigs(hopperTransition, new TalonFXConfiguration());

    transitionConfig =
        new TalonFXConfiguration()
            .withMotorOutput(TransitionConstants.TRANSITION_MOTOR_CONFIGS)
            .withCurrentLimits(TransitionConstants.TRANSITION_CURRENT_LIMITS);

    RobotContainer.applyTalonConfigs(shooterTransition, transitionConfig);
    RobotContainer.applyTalonConfigs(hopperTransition, transitionConfig);

    Dashboard.publish("Transition", this);
  }

  /** Runs only the hopper roller; the shooter feed retains its previous output. */
  public void intakeTransition(double output) {
    hopperTransition.setThrottle(output);
  }

  /** Runs both feed stages for shooting. */
  public void shooterTransition(double shooterOutput, double transitionOutput) {
    shooterTransition.setThrottle(shooterOutput);
    hopperTransition.setThrottle(transitionOutput);
  }

  /** Runs both feed stages for passing, using the caller's chosen outputs. */
  public void passingTransition(double shooterOutput, double transitionOutput) {
    shooterTransition.setThrottle(shooterOutput);
    hopperTransition.setThrottle(transitionOutput);
  }

  public void stopTransition() {
    shooterTransition.stopMotor();
    hopperTransition.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void logTo(TelemetryTable table) {

    table.log("ShooterTransitionRPM", shooterTransition.getVelocity().getValueAsDouble() * 60);

    table.log("ShooterTransition Current Supply", shooterTransition.getSupplyCurrent().getValueAsDouble());
    table.log("ShooterTransition Current Stator", shooterTransition.getStatorCurrent().getValueAsDouble());
    table.log("HopperTransition Current Supply", hopperTransition.getSupplyCurrent().getValueAsDouble());
    table.log("HopperTransition Current Stator", hopperTransition.getStatorCurrent().getValueAsDouble());
  }
}
