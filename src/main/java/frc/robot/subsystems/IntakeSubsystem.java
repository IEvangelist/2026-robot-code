// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import org.wpilib.math.system.DCMotor;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.smartdashboard.MechanismRoot2d;
import frc.robot.Dashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakePositions;
import frc.robot.subsystems.ClimbSubsystem.ClimbConstants;
import frc.robot.subsystems.ShooterSubsystem.ShooterConstants;

/** Controls the intake roller and mirrored pivot motors, with a dashboard arm simulation. */
public class IntakeSubsystem extends SubsystemBase {

  private TalonFX intakeMotor;
  private TalonFX intakePivotRight;
  private TalonFX intakePivotLeft;
  private TalonFXConfiguration intakeMotorConfig;
  private TalonFXConfiguration intakePivotConfig;
  private PositionVoltage intakeRequestTurn;

  private Slot0Configs PIDConfigIntake;

  public static class IntakeConstants {
    // .6557rot 90 deg
    private static final int INTAKE_MOTOR_ID = 60;
    private static final int INTAKE_PIVOT_RIGHT_ID = 62;
    private static final int INTAKE_PIVOT_LEFT_ID = 61;

    private static final MotorOutputConfigs INTAKE_MOTOR_CONFIGS =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.CounterClockwise_Positive);

    private static final MotorOutputConfigs INTAKE_PIVOT_CONFIGS =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive);

    // Soft limits are motor rotations, relative to the encoder zero set at startup.
    private static final double FORWARD_SOFT_LIMIT = 4.72;
    private static final double REVERSE_SOFT_LIMIT = 0.15;

    private static final double INTAKE_MOTOR_SPEED = IntakePositions.INTAKE_SPEED;
    private static final double INTAKE_PIVOT_SPEED = IntakePositions.INTAKE_PIVOT_SPEED;
    private static final double INTAKE_PIVOT_SPEED_IN = IntakePositions.INTAKE_PIVOT_SPEED_IN;

    private static final SoftwareLimitSwitchConfigs INTAKE_PIVOT_SOFTWARE_CONFIGS =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(IntakeConstants.FORWARD_SOFT_LIMIT)
            .withReverseSoftLimitThreshold(IntakeConstants.REVERSE_SOFT_LIMIT);

    private static final CurrentLimitsConfigs INTAKE_CURRENT_LIMITS =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true);

    private static final double INTAKE_ROTATIONS_PER_DEGREE = 0.007285;

    private static final double kP = 0.1;

    private static final double kS = 0.1;

    private static final double kV = 0.1;

    private static final double kI = 0.0;

    private static final double kD = 0.0;
  }

  // Simulation
  private TalonFXSimState pivotSim;
  private TalonFXSimState intakeSim;

  private final SingleJointedArmSim pivotPhysicsSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX44(1),
          1.0 / IntakeConstants.INTAKE_ROTATIONS_PER_DEGREE,
          0.05,
          0.35,
          0,
          Math.toRadians(90),
          true,
          Math.toRadians(90));

  // Mechanism2d
  private final Mechanism2d intakeMech = new Mechanism2d(2, 2);
  private final MechanismRoot2d intakeRoot = intakeMech.getRoot("Pivot", 1, 1);
  private final MechanismLigament2d intakeLigament =
      intakeRoot.append(new MechanismLigament2d("IntakeArm", 0.5, 90));

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, frc.robot.SystemCoreHardware.auxiliaryBus());
    intakePivotRight = new TalonFX(IntakeConstants.INTAKE_PIVOT_RIGHT_ID, frc.robot.SystemCoreHardware.auxiliaryBus());
    intakePivotLeft = new TalonFX(IntakeConstants.INTAKE_PIVOT_LEFT_ID, frc.robot.SystemCoreHardware.auxiliaryBus());

    RobotContainer.applyTalonConfigs(intakeMotor, new TalonFXConfiguration());
    RobotContainer.applyTalonConfigs(intakePivotRight, new TalonFXConfiguration());
    RobotContainer.applyTalonConfigs(intakePivotLeft, new TalonFXConfiguration());

    PIDConfigIntake = new Slot0Configs();
    PIDConfigIntake.kS = IntakeConstants.kS;
    PIDConfigIntake.kP = IntakeConstants.kP;
    PIDConfigIntake.kV = IntakeConstants.kV;
    PIDConfigIntake.kI = IntakeConstants.kI;
    PIDConfigIntake.kD = IntakeConstants.kD;

    intakeMotorConfig =
        new TalonFXConfiguration()
            .withMotorOutput(IntakeConstants.INTAKE_MOTOR_CONFIGS)
            .withCurrentLimits(IntakeConstants.INTAKE_CURRENT_LIMITS);

    intakePivotConfig =
        new TalonFXConfiguration()
            .withMotorOutput(IntakeConstants.INTAKE_PIVOT_CONFIGS)
            .withSoftwareLimitSwitch(IntakeConstants.INTAKE_PIVOT_SOFTWARE_CONFIGS)
            .withCurrentLimits(IntakeConstants.INTAKE_CURRENT_LIMITS)
            .withSlot0(PIDConfigIntake);

    RobotContainer.applyTalonConfigs(intakeMotor, intakeMotorConfig);
    RobotContainer.applyTalonConfigs(intakePivotRight, intakePivotConfig);
    RobotContainer.applyTalonConfigs(intakePivotLeft, intakePivotConfig);

    intakePivotLeft.getConfigurator().apply(PIDConfigIntake);
    intakePivotRight.getConfigurator().apply(PIDConfigIntake);
    // Both encoders assume the intake starts at its mechanical reference position.
    intakePivotLeft.setPosition(0);
    intakePivotRight.setPosition(0);

    intakeRequestTurn = new PositionVoltage(0).withEnableFOC(true);

    // Mirrored pivot motors need opposite shaft directions to move the arm together.
    intakePivotRight.setControl(
        new Follower(IntakeConstants.INTAKE_PIVOT_LEFT_ID, MotorAlignmentValue.Opposed));

    pivotSim = intakePivotLeft.getSimState();
    intakeSim = intakeMotor.getSimState();

    Dashboard.publish("Intake Mech2d", intakeMech);
    Dashboard.publish("Intake", this);
  }

  /** Converts a pivot goal in degrees to the left motor's position request. */
  public void turnIntake(double angle) {
    intakeRequestTurn.withPosition(angle * IntakeConstants.INTAKE_ROTATIONS_PER_DEGREE);
    intakePivotLeft.setControl(intakeRequestTurn);
  }

  /** Creates a one-shot roller duty-cycle command; output persists after the command ends. */
  public Command setIntakeSpeed(double output) {
    return runOnce(() -> intakeMotor.setThrottle(output));
  }

  /** Creates a one-shot open-loop pivot command; the right motor follows the left. */
  public Command setIntakePivotSpeed(double output) {
    return runOnce(() -> intakePivotLeft.setThrottle(output));
  }

  /** Starts collection while driving the pivot outward under its configured soft limits. */
  public void intakeOut() {
    intakeMotor.setThrottle(IntakeConstants.INTAKE_MOTOR_SPEED);
    intakePivotLeft.setThrottle(IntakeConstants.INTAKE_PIVOT_SPEED);
  }

  public Command intakeOutAuto() {
    return runOnce(
        () -> {
          intakeMotor.setThrottle(IntakeConstants.INTAKE_MOTOR_SPEED);
          intakePivotLeft.setThrottle(IntakeConstants.INTAKE_PIVOT_SPEED);
        });
  }

  /** Stops the roller and drives the pivot inward; a later action must stop the pivot. */
  public void intakeIn() {
    intakeMotor.setThrottle(0);
    intakePivotLeft.setThrottle(-IntakeConstants.INTAKE_PIVOT_SPEED_IN);
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  public void stopIntakePivot() {
    intakePivotLeft.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    pivotPhysicsSim.setInput(pivotSim.getMotorVoltage());
    pivotPhysicsSim.update(.020);

    // move the simulated motors
    pivotSim.setRawRotorPosition(
        Math.toDegrees(pivotPhysicsSim.getAngle())
            * IntakeConstants.INTAKE_ROTATIONS_PER_DEGREE);
    pivotSim.setRotorVelocity(
        Math.toDegrees(pivotPhysicsSim.getAngle())
            * IntakeConstants.INTAKE_ROTATIONS_PER_DEGREE);

    intakeLigament.setAngle(Math.toDegrees(pivotPhysicsSim.getAngle()));
  }

  @Override
  public void logTo(TelemetryTable table) {

    table.log("Pivot voltage", intakePivotLeft.getMotorVoltage().getValueAsDouble());
    table.log("Pivot velocity", intakePivotLeft.getVelocity().getValueAsDouble());
    table.log("Pivot angle",
        intakePivotLeft.getPosition().getValueAsDouble()
            / IntakeConstants.INTAKE_ROTATIONS_PER_DEGREE);
    table.log(
        "Pivot position", intakePivotLeft.getPosition().getValueAsDouble());
    table.log("Pivot current", intakePivotLeft.getSupplyCurrent().getValueAsDouble());
    table.log("Pivot stator current", intakePivotLeft.getStatorCurrent().getValueAsDouble());

    table.log("Intake RPM", intakeMotor.getVelocity().getValueAsDouble() * 60);
  }
}
