package frc.robot.subsystems;

import static frc.robot.Constants.SimDriveConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.framework.RobotBase;
import org.wpilib.system.RobotController;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.SubsystemBase;
import java.util.Arrays;
import java.util.Objects;

/**
 * Standalone, simulation-only four-module swerve drivetrain for learning.
 *
 * <p>Each module has simulated TalonFX drive and steer controllers with WPILib motor physics. The
 * pose assumes perfect wheel traction: this is not a model of robot mass, tire slip, or collisions.
 * Uses the project's 2026 APIs; it is not a 2027 hardware implementation.
 */
public class SimTalonFXDrivetrain extends SubsystemBase implements AutoCloseable {
  private static final DCMotor MOTOR = DCMotor.getKrakenX60(1);

  // Module order is front-left, front-right, back-left, back-right; +X is forward and +Y is left.
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          new Translation2d(HALF_WHEELBASE_METERS, HALF_TRACK_WIDTH_METERS),
          new Translation2d(HALF_WHEELBASE_METERS, -HALF_TRACK_WIDTH_METERS),
          new Translation2d(-HALF_WHEELBASE_METERS, HALF_TRACK_WIDTH_METERS),
          new Translation2d(-HALF_WHEELBASE_METERS, -HALF_TRACK_WIDTH_METERS));
  private final Module[] modules = new Module[4];
  private Pose2d pose = new Pose2d();
  private boolean driving;

  /** Reserves simulated CAN IDs 41-48 on the default bus. Never instantiate on a real robot. */
  public SimTalonFXDrivetrain() {
    if (RobotBase.isReal()) {
      CommandScheduler.getInstance().unregisterSubsystem(this);
      throw new IllegalStateException("SimTalonFXDrivetrain is only available in simulation.");
    }

    // Keep these IDs distinct from any other controllers created in the same simulation.
    try {
      modules[0] = new Module(FRONT_LEFT_DRIVE_CAN_ID, FRONT_LEFT_STEER_CAN_ID);
      modules[1] = new Module(FRONT_RIGHT_DRIVE_CAN_ID, FRONT_RIGHT_STEER_CAN_ID);
      modules[2] = new Module(BACK_LEFT_DRIVE_CAN_ID, BACK_LEFT_STEER_CAN_ID);
      modules[3] = new Module(BACK_RIGHT_DRIVE_CAN_ID, BACK_RIGHT_STEER_CAN_ID);
    } catch (RuntimeException error) {
      close();
      throw error;
    }
  }

  /**
   * Requests robot-relative motion: +X forward, +Y left, and positive rotation counterclockwise.
   * Translation is meters/second and rotation is radians/second.
   */
  public void drive(double forward, double left, double rotation) {
    if (!Double.isFinite(forward) || !Double.isFinite(left) || !Double.isFinite(rotation)) {
      stop();
      throw new IllegalArgumentException("Drive speeds must be finite.");
    }
    var speeds =
        new ChassisVelocities(forward, left, rotation).discretize(LOOP_PERIOD_SECONDS);
    SwerveModuleVelocity[] targets = kinematics.toSwerveModuleVelocities(speeds);
    targets = SwerveDriveKinematics.desaturateWheelVelocities(targets, MAX_SPEED_METERS_PER_SECOND);
    for (int i = 0; i < modules.length; i++) {
      if (Math.abs(targets[i].velocity) < STATIONARY_SPEED_METERS_PER_SECOND) {
        targets[i].angle = modules[i].target.angle;
      }
      // Reversing wheel direction can avoid steering more than 90 degrees.
      targets[i] = targets[i].optimize(modules[i].getState().angle);
      modules[i].target = targets[i];
    }
    driving = true;
  }

  /** Stops controller output and clears the request; simulated wheels can still coast briefly. */
  public void stop() {
    driving = false;
    for (Module module : modules) {
      if (module != null) {
        module.target = new SwerveModuleVelocity(0, module.getState().angle);
        module.driveMotor.stopMotor();
        module.steerMotor.stopMotor();
      }
    }
  }

  /** Returns simulated ground-truth pose in meters, not a camera or gyro estimate. */
  public Pose2d getPose() {
    return pose;
  }

  /** Moves the simulated field reference without changing wheel positions or velocities. */
  public void resetPose(Pose2d newPose) {
    Objects.requireNonNull(newPose, "Pose must not be null.");
    if (!Double.isFinite(newPose.getX())
        || !Double.isFinite(newPose.getY())
        || !Double.isFinite(newPose.getRotation().getRadians())) {
      throw new IllegalArgumentException("Pose coordinates must be finite.");
    }
    pose = newPose;
  }

  /** Returns independent snapshots in front-left, front-right, back-left, back-right order. */
  public SwerveModuleVelocity[] getModuleStates() {
    return Arrays.stream(modules).map(Module::getState).toArray(SwerveModuleVelocity[]::new);
  }

  /** Returns copies of the optimized, speed-limited module requests. */
  public SwerveModuleVelocity[] getTargetStates() {
    return Arrays.stream(modules)
        .map(
            module ->
                new SwerveModuleVelocity(module.target.velocity, module.target.angle))
        .toArray(SwerveModuleVelocity[]::new);
  }

  /** Elastic's SwerveDrive widget expects counterclockwise angles in radians and speeds in m/s. */
  @Override
  public String getTelemetryType() { return "SwerveDrive"; }

  @Override
  public void logTo(TelemetryTable table) {
    String[] names = {"Front Left", "Front Right", "Back Left", "Back Right"};
    for (int i = 0; i < modules.length; i++) {
      final int index = i;
      table.log(names[i] + " Angle", modules[index].getState().angle.getRadians());
      table.log(names[i] + " Velocity", modules[index].getState().velocity);
    }
    table.log("Robot Angle", pose.getRotation().getRadians());
  }

  @Override
  public void simulationPeriodic() {
    if (!org.wpilib.driverstation.RobotState.isEnabled()) {
      stop();
    }
    double batteryVoltage = RobotController.getBatteryVoltage();
    for (Module module : modules) {
      module.update(batteryVoltage, driving);
    }
    ChassisVelocities speeds = kinematics.toChassisVelocities(getModuleStates());
    pose = pose.transformBy(speeds.toTwist2d(LOOP_PERIOD_SECONDS).exp());
  }

  /** Releases CAN simulation devices and scheduler registration, especially between tests. */
  @Override
  public void close() {
    CommandScheduler.getInstance().unregisterSubsystem(this);
    for (Module module : modules) {
      if (module != null) {
        module.driveMotor.close();
        module.steerMotor.close();
      }
    }
  }

  private static final class Module {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final TalonFXSimState driveSim;
    private final TalonFXSimState steerSim;
    private final DCMotorSim drivePhysics =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                MOTOR, DRIVE_INERTIA_KG_METERS_SQUARED, DRIVE_GEAR_RATIO),
            MOTOR);
    private final DCMotorSim steerPhysics =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                MOTOR, STEER_INERTIA_KG_METERS_SQUARED, STEER_GEAR_RATIO),
            MOTOR);
    private SwerveModuleVelocity target = new SwerveModuleVelocity();

    private Module(int driveId, int steerId) {
      driveMotor = new TalonFX(driveId, new com.ctre.phoenix6.CANBus());
      steerMotor = new TalonFX(steerId, new com.ctre.phoenix6.CANBus());
      driveSim = driveMotor.getSimState();
      steerSim = steerMotor.getSimState();
      try {
        configure(driveMotor);
        configure(steerMotor);
      } catch (RuntimeException error) {
        driveMotor.close();
        steerMotor.close();
        throw error;
      }
    }

    private static void configure(TalonFX motor) {
      StatusCode status = motor.getConfigurator().apply(new TalonFXConfiguration());
      if (!status.isOK()) {
        throw new IllegalStateException(
            "Could not configure simulated TalonFX " + motor.getDeviceID() + ": " + status);
      }
    }

    private SwerveModuleVelocity getState() {
      return new SwerveModuleVelocity(
          drivePhysics.getAngularVelocity() * WHEEL_RADIUS_METERS,
          Rotation2d.fromRadians(steerPhysics.getAngularPosition()));
    }

    private void update(double batteryVoltage, boolean driving) {
      driveSim.setSupplyVoltage(batteryVoltage);
      steerSim.setSupplyVoltage(batteryVoltage);
      if (driving) {
        // Use ideal simulated angle feedback so this lesson does not require a separate CANcoder.
        double error =
            MathUtil.angleModulus(target.angle.getRadians() - steerPhysics.getAngularPosition());
        double steerVolts =
            STEER_KP * error - STEER_KD * steerPhysics.getAngularVelocity();
        steerMotor.setVoltage(Math.clamp(steerVolts, -batteryVoltage, batteryVoltage));

        // Reduce drive speed while the wheel turns toward its goal; the voltage is feedforward
        // only.
        double wheelSpeed = target.velocity * Math.cos(error);
        double rotorSpeed = wheelSpeed / WHEEL_RADIUS_METERS * DRIVE_GEAR_RATIO;
        double driveVolts = rotorSpeed / MOTOR.Kv;
        driveMotor.setVoltage(Math.clamp(driveVolts, -batteryVoltage, batteryVoltage));
      }

      // Phoenix supplies controller voltage; WPILib integrates motion and feeds encoder data back.
      drivePhysics.setInputVoltage(driveSim.getMotorVoltage());
      steerPhysics.setInputVoltage(steerSim.getMotorVoltage());
      drivePhysics.update(LOOP_PERIOD_SECONDS);
      steerPhysics.update(LOOP_PERIOD_SECONDS);
      updateRotor(driveSim, drivePhysics, DRIVE_GEAR_RATIO);
      updateRotor(steerSim, steerPhysics, STEER_GEAR_RATIO);
    }

    private static void updateRotor(TalonFXSimState sim, DCMotorSim physics, double gearRatio) {
      // Physics describes the mechanism output; TalonFX encoders describe the motor rotor.
      sim.setRawRotorPosition((physics.getAngularPosition() / (2 * Math.PI)) * gearRatio);
      sim.setRotorVelocity(physics.getAngularVelocity() / (2 * Math.PI) * gearRatio);
    }
  }
}
