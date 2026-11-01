// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Velocity;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.Alliance;
import org.wpilib.hardware.expansionhub.ExpansionHubServo;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.smartdashboard.MechanismRoot2d;
import frc.robot.Dashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.RunCommand;
import org.wpilib.command2.SubsystemBase;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Constants;
import frc.robot.SOTMTable;
import frc.robot.subsystems.IntakeSubsystem.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

/** Controls flywheel speed, turret yaw, and hood position for pose-based shooting and passing. */
public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX shooterMotorLeft;
  private final TalonFX shooterMotorRight;
  private final TalonFX shooterMotorTurn;

  private final ExpansionHubServo leftActuator;
  private final ExpansionHubServo rightActuator;
  private double actuatorPosition;

  private final TalonFXConfiguration shooterMotorConfig;
  private final TalonFXConfiguration shooterMotorTurnConfig;

  private Slot0Configs PIDConfigShooter;
  private Slot0Configs PIDConfigTurn;

  private VelocityVoltage shooterRequest;
  private PositionVoltage shooterRequestTurn;

  // Live state used for dashboard/debugging and target-solving feedback.
  private double setAngle = 0.0;
  private double shotAngle = 0.0;
  private double horizSpeed = 0.0;

  // Visualization for the turret angle in NetworkTables / Dashboard.
  private Mechanism2d turretmech = new Mechanism2d(2, 2);
  private MechanismRoot2d turretRoot = turretmech.getRoot("TurretRoot", 1, 1);
  private MechanismLigament2d turretLigament =
      turretRoot.append(new MechanismLigament2d("TurretLigament", 0.5, 180));
  private TalonFXSimState shooterTurnSim;
  private SingleJointedArmSim shooterTurnPhysics;

  public static final class ShooterConstants {
    private static final int SHOOTER_LEFT_CAN_ID = 51;
    private static final int SHOOTER_RIGHT_CAN_ID = 52;
    private static final int SHOOTER_STEER_CAN_ID = 50;

    // TODO: This value is still placeholder-level and must be measured on the real mechanism.
    private static final double SHOOTER_GEAR_RATIO = 0.0;

    // Hood travel spans 40 to 65 degrees in the actuator-position approximation.
    private static final double SHOOTER_HOOD_MAXIMUM_ANGLE =
        65.0; // Hood is fully down / lowest point.
    private static final double SHOOTER_HOOD_MAX_CHANGE_ANGLE =
        25.0; // Hood travel range in degrees.

    // Turret position requests and soft limits use rotor rotations.
    private static final double SHOOTER_ROTATIONS_PER_DEGREE = 0.13568;
    private static final double MAX_RPM_ERROR = 50.0;

    private static final MotorOutputConfigs SHOOTER_MOTOR_CONFIG =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive);

    private static final MotorOutputConfigs SHOOTER_MOTOR_TURN_CONFIG =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);

    private static final SoftwareLimitSwitchConfigs SHOOTER_TURN_SOFT_LIMITS =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(14.7)
            .withReverseSoftLimitThreshold(-15.0);

    private static final CurrentLimitsConfigs SHOOTER_CURRENT_CONFIGS =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(65)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(65)
            .withSupplyCurrentLimitEnable(true);

    private static final class SHOOTER_PID_VALUES {
      private static final double kP = 0.1;
      private static final double kS = 0.1;
      private static final double kV = 0.065;
      private static final double kI = 0.0;
      private static final double kD = 0.0;
    }

    private static final class SHOOTER_TURN_PID_VALUES {
      private static final double kP = 2.015;
      private static final double kS = 0.3835;
      private static final double kV = 0.1;
      private static final double kI = 0.0;
      private static final double kD = 0.0;
    }
  }

  /** Creates a new ShooterSubsystem and configures all motor and actuator control loops. */
  public ShooterSubsystem() {
    shooterMotorLeft = new TalonFX(ShooterConstants.SHOOTER_LEFT_CAN_ID, frc.robot.SystemCoreHardware.auxiliaryBus());
    shooterMotorRight = new TalonFX(ShooterConstants.SHOOTER_RIGHT_CAN_ID, frc.robot.SystemCoreHardware.auxiliaryBus());
    shooterMotorTurn = new TalonFX(ShooterConstants.SHOOTER_STEER_CAN_ID, frc.robot.SystemCoreHardware.auxiliaryBus());

    leftActuator = new ExpansionHubServo(frc.robot.SystemCoreHardware.expansionHubUsbPort(), frc.robot.SystemCoreHardware.hoodServoChannel(true));
    rightActuator = new ExpansionHubServo(frc.robot.SystemCoreHardware.expansionHubUsbPort(), frc.robot.SystemCoreHardware.hoodServoChannel(false));


    leftActuator.setPWMRange(1000, 2000);
    rightActuator.setPWMRange(1000, 2000);

    // Flywheel PID tuning.
    PIDConfigShooter = new Slot0Configs();
    PIDConfigShooter.kS = ShooterConstants.SHOOTER_PID_VALUES.kS;
    PIDConfigShooter.kP = ShooterConstants.SHOOTER_PID_VALUES.kP;
    PIDConfigShooter.kV = ShooterConstants.SHOOTER_PID_VALUES.kV;
    PIDConfigShooter.kI = ShooterConstants.SHOOTER_PID_VALUES.kI;
    PIDConfigShooter.kD = ShooterConstants.SHOOTER_PID_VALUES.kD;

    // Turret PID tuning.
    PIDConfigTurn = new Slot0Configs();
    PIDConfigTurn.kS = ShooterConstants.SHOOTER_TURN_PID_VALUES.kS;
    PIDConfigTurn.kP = ShooterConstants.SHOOTER_TURN_PID_VALUES.kP;
    PIDConfigTurn.kV = ShooterConstants.SHOOTER_TURN_PID_VALUES.kV;
    PIDConfigTurn.kI = ShooterConstants.SHOOTER_TURN_PID_VALUES.kI;
    PIDConfigTurn.kD = ShooterConstants.SHOOTER_TURN_PID_VALUES.kD;

    // TalonFX configs for the flywheel and turret.
    shooterMotorConfig =
        new TalonFXConfiguration()
            .withMotorOutput(ShooterConstants.SHOOTER_MOTOR_CONFIG)
            .withCurrentLimits(ShooterConstants.SHOOTER_CURRENT_CONFIGS);

    shooterMotorTurnConfig =
        new TalonFXConfiguration()
            .withMotorOutput(ShooterConstants.SHOOTER_MOTOR_TURN_CONFIG)
            .withSoftwareLimitSwitch(ShooterConstants.SHOOTER_TURN_SOFT_LIMITS)
            .withCurrentLimits(ShooterConstants.SHOOTER_CURRENT_CONFIGS)
            .withSlot0(PIDConfigTurn);

    shooterRequest = new VelocityVoltage(0).withEnableFOC(true);
    shooterRequestTurn = new PositionVoltage(0).withEnableFOC(true).withSlot(0);

    // Apply default config before applying tuned values to each motor.
    shooterMotorLeft.getConfigurator().apply(new TalonFXConfiguration());
    shooterMotorRight.getConfigurator().apply(new TalonFXConfiguration());
    shooterMotorTurn.getConfigurator().apply(new TalonFXConfiguration());

    shooterMotorLeft.getConfigurator().apply(shooterMotorConfig);
    shooterMotorRight.getConfigurator().apply(shooterMotorConfig);
    shooterMotorTurn.getConfigurator().apply(shooterMotorTurnConfig);

    shooterMotorLeft.getConfigurator().apply(PIDConfigShooter);
    shooterMotorRight.getConfigurator().apply(PIDConfigShooter);
    shooterMotorTurn.getConfigurator().apply(PIDConfigTurn);

    // Right flywheel is mirrored opposite the left flywheel.
    shooterMotorRight.setControl(
        new Follower(ShooterConstants.SHOOTER_LEFT_CAN_ID, MotorAlignmentValue.Opposed));
    // The turret must physically match this reference when the robot starts.
    shooterMotorTurn.setPosition(0);

    Dashboard.publish("Shooter", this);
    Dashboard.publish("TurretMech", turretmech);

    // Simulation support for turret angle after initial config.
    shooterTurnSim = shooterMotorTurn.getSimState();
    shooterTurnPhysics =
        new SingleJointedArmSim(
            DCMotor.getKrakenX44(1),
            1 / ShooterConstants.SHOOTER_ROTATIONS_PER_DEGREE,
            0.05,
            0.3,
            Math.toRadians(-90),
            Math.toRadians(90),
            false,
            0);
  }

  /** Sets the flywheel target RPM. */
  public void setRPM(double rpm) {
    double rps = rpm / 60.0;

    shooterRequest =
        shooterRequest
            .withVelocity(rps)
            .withFeedForward(ShooterConstants.SHOOTER_PID_VALUES.kV * rps);
    shooterMotorLeft.setControl(shooterRequest);
  }

  /** Returns a command that sets flywheel RPM once when scheduled. */
  public Command setRPMAuto(double rpm) {
    return runOnce(
        () -> {
          double rps = rpm / 60.0;
          shooterRequest =
              shooterRequest
                  .withVelocity(rps)
                  .withFeedForward(ShooterConstants.SHOOTER_PID_VALUES.kV * rps);
          shooterMotorLeft.setControl(shooterRequest);
        });
  }

  /** Returns the current flywheel speed in RPM. */
  public double getRPM() {
    return shooterMotorLeft.getVelocity().getValueAsDouble() * 60;
  }

  /** True when flywheel speed is within the configured tolerance band. */
  public boolean withinErrorRPM(double setRPM) {
    return Math.abs(getRPM() - setRPM) <= ShooterConstants.MAX_RPM_ERROR;
  }

  /** Commands the turret to a target angle in degrees relative to the mechanism zero. */
  public void turnShooter(double angle) {
    shooterMotorTurn.setControl(
        shooterRequestTurn.withPosition(angle * ShooterConstants.SHOOTER_ROTATIONS_PER_DEGREE));
  }

  /** Directly drives the turret motor by voltage. Useful for manual testing or calibration. */
  public void setShooterTurnVoltage(double voltage) {
    shooterMotorTurn.setVoltage(voltage);
  }

  /** Sends both hood servos a normalized position from 0 to 1, not a length measurement. */
  public void extendActuator(double length) {
    actuatorPosition = Math.clamp(length, 0.0, 1.0);
    leftActuator.setPosition(actuatorPosition);
    rightActuator.setPosition(actuatorPosition);
  }

  /** Clamps the hood goal and writes a proportional actuator position to move toward it. */
  public void setHoodAngle(double angle) {
    double targetAngle = angle;
    if (angle > ShooterConstants.SHOOTER_HOOD_MAXIMUM_ANGLE) {
      targetAngle = ShooterConstants.SHOOTER_HOOD_MAXIMUM_ANGLE;
    } else if (angle
        < (ShooterConstants.SHOOTER_HOOD_MAXIMUM_ANGLE
            - ShooterConstants.SHOOTER_HOOD_MAX_CHANGE_ANGLE)) {
      targetAngle =
          ShooterConstants.SHOOTER_HOOD_MAXIMUM_ANGLE
              - ShooterConstants.SHOOTER_HOOD_MAX_CHANGE_ANGLE;
    }

    // This uses the previous commanded position, not measured hood-angle feedback.
    double normalizedPosition =
        Math.abs(getHoodAngle() - targetAngle) / ShooterConstants.SHOOTER_HOOD_MAX_CHANGE_ANGLE;
    actuatorPosition = Math.clamp(normalizedPosition, 0.0, 1.0);
    leftActuator.setPosition(actuatorPosition);
    rightActuator.setPosition(actuatorPosition);
  }

  /** Returns the actuator command position, which is used to infer hood angle. */
  public double getActuatorLength() {
    return actuatorPosition;
  }

  /**
   * Estimates hood angle from the commanded servo position; there is no angle sensor here.
   *
   * @return hood angle in degrees
   */
  public double getHoodAngle() {
    return ShooterConstants.SHOOTER_HOOD_MAXIMUM_ANGLE
        - (getActuatorLength() * ShooterConstants.SHOOTER_HOOD_MAX_CHANGE_ANGLE);
  }

  /** Returns the turret angle in degrees from the configured encoder zero. */
  public double getShooterAngle() {
    return shooterMotorTurn.getRotorPosition().getValueAsDouble()
        / ShooterConstants.SHOOTER_ROTATIONS_PER_DEGREE;
  }

  /** Direct open-loop flywheel output, mainly for debugging and manual testing. */
  public void setOutput(double output) {
    shooterMotorLeft.setThrottle(output);
  }

  /** Stops the flywheel. */
  public void stopShooter() {
    shooterMotorLeft.stopMotor();
  }

  /** Stops the turret motor. */
  public void stopShooterTurn() {
    shooterMotorTurn.stopMotor();
  }

  private Rotation2d getTurretAngle(Translation2d shotVector, Rotation2d robotRotation) {
    var angle = shotVector.getAngle();
    if (angle.isEmpty()) {
      stopShooter();
      stopShooterTurn();
      throw new IllegalStateException("Cannot aim a shot with zero horizontal velocity");
    }
    return angle.get().minus(robotRotation);
  }

  /**
   * Aims at the alliance hub using the fixed-hood RPM table and motion compensation. Pose is
   * field-relative; chassis speeds are robot-relative in meters per second.
   */
  public void updateSOTM(Pose2d robotPose, ChassisVelocities robotSpeed) {
    double latency = frc.robot.Constants.ShooterConstants.LATENCY_SEC;
    // Rotate robot-relative travel into the field frame before predicting the release position.
    Translation2d futurePos =
        robotPose
            .getTranslation()
            .plus(
                new Translation2d(robotSpeed.vx, robotSpeed.vy)
                    .times(latency)
                    .rotateBy(robotPose.getRotation()));

    Translation2d goalLocation =
        (org.wpilib.driverstation.MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.BLUE)
            ? TrajectoryConstants.BLUE_HUB_GOAL
            : TrajectoryConstants.RED_HUB_GOAL;
    Translation2d targetVector = goalLocation.minus(futurePos);
    double distance = targetVector.getNorm();

    // Convert table RPM to wheel surface speed, then split it at the configured launch angle.
    double idealHorizontalSpeed =
        (SOTMTable.getSpeed(distance) / 60.0)
            * (frc.robot.Constants.ShooterConstants.WHEEL_DIAMETER * Math.PI)
            * Math.cos(Math.toRadians(frc.robot.Constants.ShooterConstants.HOOD_ANGLE_SHOOT));
    double idealVerticalSpeed =
        (SOTMTable.getSpeed(distance) / 60.0)
            * (frc.robot.Constants.ShooterConstants.WHEEL_DIAMETER * Math.PI)
            * Math.sin(Math.toRadians(frc.robot.Constants.ShooterConstants.HOOD_ANGLE_SHOOT));

    // Subtract robot motion to compensate the shot. Both vectors must share a coordinate frame.
    Translation2d robotVelocityVector =
        new Translation2d(robotSpeed.vx, robotSpeed.vy);
    Translation2d shotVector =
        targetVector.div(distance).times(idealHorizontalSpeed).minus(robotVelocityVector);

    double turretAngle = getTurretAngle(shotVector, robotPose.getRotation()).getDegrees();
    double newHorizontalSpeed = shotVector.getNorm();

    this.setRPM(
        (Math.hypot(newHorizontalSpeed, idealVerticalSpeed)
                / (frc.robot.Constants.ShooterConstants.WHEEL_DIAMETER * Math.PI))
            * 60);
    // Convert field bearing to the turret's reversed, rear-facing angular convention.
    this.turnShooter(MathUtil.inputModulus(360 - turretAngle - 180.0, -180, 180));
    this.setHoodAngle(Constants.ShooterConstants.HOOD_ANGLE_SHOOT);

    turretLigament.setAngle(MathUtil.inputModulus(turretAngle, -180, 180) + 180);
    horizSpeed = idealHorizontalSpeed;
  }

  /**
   * Aims toward the alliance-dependent high-Y bump target using the shooting RPM table. Leaves the
   * hood at its previous position.
   */
  public void updateHighPOTM(Pose2d robotPose, ChassisVelocities robotSpeed) {
    double latency = frc.robot.Constants.ShooterConstants.LATENCY_SEC;
    Translation2d futurePos =
        robotPose
            .getTranslation()
            .plus(
                new Translation2d(robotSpeed.vx, robotSpeed.vy)
                    .times(latency)
                    .rotateBy(robotPose.getRotation()));

    Translation2d goalLocation =
        (org.wpilib.driverstation.MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.BLUE)
            ? TrajectoryConstants.BLUE_BUMP_D_L.getTranslation()
            : TrajectoryConstants.RED_BUMP_O_L.getTranslation();
    Translation2d targetVector = goalLocation.minus(futurePos);
    double distance = targetVector.getNorm();

    double idealHorizontalSpeed =
        (SOTMTable.getSpeed(distance) / 60.0)
            * (frc.robot.Constants.ShooterConstants.WHEEL_DIAMETER * Math.PI)
            * Math.cos(Math.toRadians(frc.robot.Constants.ShooterConstants.HOOD_ANGLE_SHOOT));
    double idealVerticalSpeed =
        (SOTMTable.getSpeed(distance) / 60.0)
            * (frc.robot.Constants.ShooterConstants.WHEEL_DIAMETER * Math.PI)
            * Math.sin(Math.toRadians(frc.robot.Constants.ShooterConstants.HOOD_ANGLE_SHOOT));

    Translation2d robotVelocityVector =
        new Translation2d(robotSpeed.vx, robotSpeed.vy);
    Translation2d shotVector =
        targetVector.div(distance).times(idealHorizontalSpeed).minus(robotVelocityVector);

    double turretAngle = getTurretAngle(shotVector, robotPose.getRotation()).getDegrees();
    double newHorizontalSpeed = shotVector.getNorm();
    double totalExitVelocity = Math.hypot(newHorizontalSpeed, idealVerticalSpeed);

    this.setRPM(
        (totalExitVelocity / (frc.robot.Constants.ShooterConstants.WHEEL_DIAMETER * Math.PI)) * 60);
    this.turnShooter(MathUtil.inputModulus(360 - turretAngle - 180.0, -180, 180));

    turretLigament.setAngle(MathUtil.inputModulus(turretAngle, -180, 180) + 180);
    horizSpeed = idealHorizontalSpeed;
  }

  /**
   * Aims toward the alliance-dependent low-Y bump target using the shooting RPM table. Leaves the
   * hood at its previous position.
   */
  public void updateLowPOTM(Pose2d robotPose, ChassisVelocities robotSpeed) {
    double latency = frc.robot.Constants.ShooterConstants.LATENCY_SEC;
    Translation2d futurePos =
        robotPose
            .getTranslation()
            .plus(
                new Translation2d(robotSpeed.vx, robotSpeed.vy)
                    .times(latency)
                    .rotateBy(robotPose.getRotation()));

    Translation2d goalLocation =
        (org.wpilib.driverstation.MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.BLUE)
            ? TrajectoryConstants.BLUE_BUMP_O_L.getTranslation()
            : TrajectoryConstants.RED_BUMP_D_L.getTranslation();
    Translation2d targetVector = goalLocation.minus(futurePos);
    double distance = targetVector.getNorm();

    double idealHorizontalSpeed =
        (SOTMTable.getSpeed(distance) / 60.0)
            * (frc.robot.Constants.ShooterConstants.WHEEL_DIAMETER * Math.PI)
            * Math.cos(Math.toRadians(frc.robot.Constants.ShooterConstants.HOOD_ANGLE_SHOOT));
    double idealVerticalSpeed =
        (SOTMTable.getSpeed(distance) / 60.0)
            * (frc.robot.Constants.ShooterConstants.WHEEL_DIAMETER * Math.PI)
            * Math.sin(Math.toRadians(frc.robot.Constants.ShooterConstants.HOOD_ANGLE_SHOOT));

    Translation2d robotVelocityVector =
        new Translation2d(robotSpeed.vx, robotSpeed.vy);
    Translation2d shotVector =
        targetVector.div(distance).times(idealHorizontalSpeed).minus(robotVelocityVector);

    double turretAngle = getTurretAngle(shotVector, robotPose.getRotation()).getDegrees();
    double newHorizontalSpeed = shotVector.getNorm();
    double totalExitVelocity = Math.hypot(newHorizontalSpeed, idealVerticalSpeed);

    this.setRPM(
        (totalExitVelocity / (frc.robot.Constants.ShooterConstants.WHEEL_DIAMETER * Math.PI)) * 60);
    this.turnShooter(MathUtil.inputModulus(360 - turretAngle - 180.0, -180, 180));

    turretLigament.setAngle(MathUtil.inputModulus(turretAngle, -180, 180) + 180);
    horizSpeed = idealHorizontalSpeed;
  }

  @Override
  public void periodic() {
    // No periodic logic is required for the basic shooter loop; all actuation is driven by
    // commands.
  }

  @Override
  public void logTo(TelemetryTable table) {

    table.log("Shooter RPM", shooterMotorLeft.getVelocity().getValueAsDouble() * 60);
    table.log("Shooter Voltage", shooterMotorLeft.getMotorVoltage().getValueAsDouble());
    table.log("Shooter Amps Supply", shooterMotorLeft.getSupplyCurrent().getValueAsDouble());
    table.log("Shooter Amps Stator", shooterMotorLeft.getStatorCurrent().getValueAsDouble());

    table.log("ShooterTurn SetAngle", setAngle);
    table.log("Shooter Angle", getShooterAngle());
    table.log("TargetHorizontalSpeed", horizSpeed);

    table.log("ShooterTurn Amps Supply", shooterMotorTurn.getSupplyCurrent().getValueAsDouble());
    table.log("ShooterTurn Amps Stator", shooterMotorTurn.getStatorCurrent().getValueAsDouble());

    table.log("Actuator Length", actuatorPosition);
  }

  public void simulationPeriodic() {
    shooterTurnPhysics.setInput(shooterTurnSim.getMotorVoltage());
    shooterTurnPhysics.update(0.020);

    shooterTurnSim.setRawRotorPosition(
        Math.toDegrees(shooterTurnPhysics.getAngle())
            * ShooterConstants.SHOOTER_ROTATIONS_PER_DEGREE);
    shooterTurnSim.setRotorVelocity(
        Math.toDegrees(shooterTurnPhysics.getAngle())
            * ShooterConstants.SHOOTER_ROTATIONS_PER_DEGREE);
  }
}
