// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.system.DataLogManager;
import frc.robot.Dashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem.IntakeConstants;

/** Controls the climber motor and converts encoder rotations to extension in inches. */
public class ClimbSubsystem extends SubsystemBase {
  public TalonFX climbMotor;
  private TalonFXConfiguration climbMotorConfig;
  public MotionMagicVoltage climbRequest;
  public PositionVoltage voltageRequest;

  /** Motor configuration and encoder-to-extension calibration. */
  public static class ClimbConstants {

    private static final int CLIMB_MOTOR_ID = 31;

    private static final double MAX_ERROR_INCHES = 0.005;

    public static final double CLIMBER_INCHES_PER_ROTATION = 0.026415; // TODO find real value

    // private static final double MAX_VELOCITY = 50.0; //rotations per second

    // private static final double MAX_ACCEL = 250.0; //rotations per second^2

    // private static final double MAX_JERK = 2500.0; //rotations per second^3

    // private static final MotionMagicConfigs CLIMB_MOTION_CONFIG = new MotionMagicConfigs()
    // .withMotionMagicCruiseVelocity(MAX_VELOCITY)
    // .withMotionMagicAcceleration(MAX_ACCEL)
    // .withMotionMagicJerk(MAX_JERK);

    private static final Voltage MAX_VOLTS = Volts.of(32);

    private static final VoltageConfigs CLIMBER_VOLTAGE_CONFIGS =
        new VoltageConfigs()
            .withPeakForwardVoltage(MAX_VOLTS)
            .withPeakReverseVoltage(MAX_VOLTS.times(-1.0));

    private static final MotorOutputConfigs CLIMB_MOTOR_CONFIGS =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);

    private static final Current CLIMB_CURRENT_LIMIT = Amps.of(250);

    private static final CurrentLimitsConfigs CLIMB_CURRENT_CONFIGS =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(CLIMB_CURRENT_LIMIT)
            .withSupplyCurrentLimit(CLIMB_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);

    // TalonFX soft limits use motor rotations, not inches of extension.
    private static final double FORWARD_SOFT_LIMIT = 265.0;
    private static final double REVERSE_SOFT_LIMIT = 0.0;

    private static final SoftwareLimitSwitchConfigs CLIMB_SOFTWARE_CONFIGS =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(ClimbConstants.FORWARD_SOFT_LIMIT)
            .withReverseSoftLimitThreshold(ClimbConstants.REVERSE_SOFT_LIMIT);

    private static final double kP = 35.6;
    private static final double kS = -.19;
    private static final double kV = .423;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private static final Slot0Configs CLIMBER_PID_CONFIGS =
        new Slot0Configs().withKS(kS).withKP(kP).withKV(kV).withKI(kI).withKD(kD);
  }

  public ClimbSubsystem() {

    climbMotor = new TalonFX(ClimbConstants.CLIMB_MOTOR_ID, frc.robot.SystemCoreHardware.auxiliaryBus());
    // climbRequest = new MotionMagicVoltage(0);
    voltageRequest = new PositionVoltage(0).withEnableFOC(true);

    RobotContainer.applyTalonConfigs(climbMotor, new TalonFXConfiguration());

    climbMotorConfig =
        new TalonFXConfiguration()
            .withMotorOutput(ClimbConstants.CLIMB_MOTOR_CONFIGS)
            .withCurrentLimits(ClimbConstants.CLIMB_CURRENT_CONFIGS)
            // .withMotionMagic(ClimbConstants.CLIMB_MOTION_CONFIG)
            .withVoltage(ClimbConstants.CLIMBER_VOLTAGE_CONFIGS)
            .withSlot0(ClimbConstants.CLIMBER_PID_CONFIGS)
            .withSoftwareLimitSwitch(ClimbConstants.CLIMB_SOFTWARE_CONFIGS);

    RobotContainer.applyTalonConfigs(climbMotor, climbMotorConfig);
    Dashboard.publish("Climber", this);

    // Software zero assumes the mechanism is physically at its reference position at startup.
    climbMotor.setPosition(0);
  }

  /** Applies a signed duty-cycle output rather than position control. */
  public void setSpeed(double speed) {
    climbMotor.setThrottle(speed);
  }

  public void setVoltage(double voltage) {
    climbMotor.setVoltage(voltage);
  }

  /** Requests motor velocity in rotations per second. */
  public void setVelocity(double velocity) {
    climbMotor.setControl(new VelocityVoltage(velocity));
  }

  /** Tests a target height in inches against the configured position tolerance. */
  public boolean withinError(double position) {
    return (Math.abs(getClimberHeight() - position) <= ClimbConstants.MAX_ERROR_INCHES);
  }

  private boolean withinError(DoubleSupplier position) {
    return (Math.abs(getClimberHeight() - position.getAsDouble())
        <= ClimbConstants.MAX_ERROR_INCHES);
  }

  /** Returns extension in inches relative to the startup encoder zero. */
  public double getClimberHeight() {
    return (climbMotor.getPosition().getValueAsDouble()
        * ClimbConstants.CLIMBER_INCHES_PER_ROTATION);
  }

  public void stopClimbMotors() {
    climbMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void logTo(TelemetryTable table) {

    table.log("Motor Rotations", getClimberHeight() / ClimbConstants.CLIMBER_INCHES_PER_ROTATION);
    table.log("Climber Velocity", climbMotor.getVelocity().getValueAsDouble());
    table.log("Climber Voltage", climbMotor.getMotorVoltage().getValueAsDouble());
    table.log("Climber Current Stator", climbMotor.getStatorCurrent().getValueAsDouble());
    table.log("Climber Current Supply", climbMotor.getSupplyCurrent().getValueAsDouble());
    table.log("Climber Height", getClimberHeight());
    table.log("Climber Within Error", withinError(Constants.ClimbPositions.L1_INCHES));
  }
}
