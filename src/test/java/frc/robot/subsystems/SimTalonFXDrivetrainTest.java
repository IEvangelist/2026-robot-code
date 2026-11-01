package frc.robot.subsystems;

import static frc.robot.Constants.SimDriveConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.RoboRioSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SimTalonFXDrivetrainTest {
  private SimTalonFXDrivetrain drivetrain;

  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize());
    DriverStationSim.resetData();
    RoboRioSim.setVInVoltage(12.0);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    drivetrain = new SimTalonFXDrivetrain();
  }

  @AfterEach
  void tearDown() {
    if (drivetrain != null) {
      drivetrain.close();
    }
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  @Test
  void convertsForwardStrafeAndRotationIntoModuleRequests() {
    var kinematics =
        new SwerveDriveKinematics(
            new Translation2d(HALF_WHEELBASE_METERS, HALF_TRACK_WIDTH_METERS),
            new Translation2d(HALF_WHEELBASE_METERS, -HALF_TRACK_WIDTH_METERS),
            new Translation2d(-HALF_WHEELBASE_METERS, HALF_TRACK_WIDTH_METERS),
            new Translation2d(-HALF_WHEELBASE_METERS, -HALF_TRACK_WIDTH_METERS));
    for (double[] request :
        new double[][] {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}}) {
      drivetrain.drive(request[0], request[1], request[2]);
      var speeds = kinematics.toChassisVelocities(drivetrain.getTargetStates());
      assertEquals(request[0], speeds.vx, 1e-9);
      assertEquals(request[1], speeds.vy, 1e-9);
      assertEquals(request[2], speeds.omega, 1e-9);
    }
  }

  @Test
  void limitsWheelSpeedAndReturnsIndependentSnapshots() {
    drivetrain.drive(20, 10, 4);
    SwerveModuleVelocity[] targets = drivetrain.getTargetStates();
    for (SwerveModuleVelocity target : targets) {
      assertTrue(Math.abs(target.velocity) <= MAX_SPEED_METERS_PER_SECOND + 1e-9);
    }
    targets[0].velocity = 100;
    assertTrue(
        Math.abs(drivetrain.getTargetStates()[0].velocity)
            <= MAX_SPEED_METERS_PER_SECOND);
    SwerveModuleVelocity[] measured = drivetrain.getModuleStates();
    measured[0].velocity = 100;
    assertEquals(0, drivetrain.getModuleStates()[0].velocity);
  }

  @Test
  void rejectsInvalidInputsAndClearsPreviousDriveRequest() {
    for (double invalid :
        new double[] {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
      drivetrain.drive(1, 0, 0);
      assertThrows(IllegalArgumentException.class, () -> drivetrain.drive(invalid, 0, 0));
      assertThrows(IllegalArgumentException.class, () -> drivetrain.drive(0, invalid, 0));
      assertThrows(IllegalArgumentException.class, () -> drivetrain.drive(0, 0, invalid));
      for (var target : drivetrain.getTargetStates()) {
        assertEquals(0, target.velocity);
      }
    }
    var pose = new Pose2d(2, 3, Rotation2d.fromDegrees(90));
    drivetrain.resetPose(pose);
    assertEquals(pose, drivetrain.getPose());
    assertThrows(NullPointerException.class, () -> drivetrain.resetPose(null));
    assertThrows(
        IllegalArgumentException.class,
        () -> drivetrain.resetPose(new Pose2d(Double.NaN, 0, new Rotation2d())));
    assertEquals(pose, drivetrain.getPose());
  }

  @Test
  void simulatesForwardMotionThenStopsWhenDisabled() throws InterruptedException {
    runSimulation(1, 0, 0, 75);
    assertTrue(drivetrain.getPose().getX() > 0.5, drivetrain.getPose().toString());
    assertEquals(0, drivetrain.getPose().getY(), 0.05);
    assertEquals(0, drivetrain.getPose().getRotation().getRadians(), 0.05);

    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    for (int i = 0; i < 50; i++) {
      Thread.sleep(Math.round(LOOP_PERIOD_SECONDS * 1000));
      drivetrain.simulationPeriodic();
    }
    for (var state : drivetrain.getModuleStates()) {
      assertEquals(0, state.velocity, 0.05);
    }
    // Re-enabling alone must not resurrect the old drive request.
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    var stoppedPose = drivetrain.getPose();
    for (int i = 0; i < 10; i++) {
      Unmanaged.feedEnable(100);
      Thread.sleep(Math.round(LOOP_PERIOD_SECONDS * 1000));
      drivetrain.simulationPeriodic();
    }
    assertEquals(stoppedPose.getX(), drivetrain.getPose().getX(), 0.02);
  }

  @Test
  void simulatesSidewaysMotion() throws InterruptedException {
    runSimulation(0, 1, 0, 100);
    assertTrue(drivetrain.getPose().getY() > 0.5, drivetrain.getPose().toString());
    assertEquals(0, drivetrain.getPose().getX(), 0.20);
    assertEquals(0, drivetrain.getPose().getRotation().getRadians(), 0.05);
  }

  @Test
  void simulatesCounterclockwiseRotation() throws InterruptedException {
    runSimulation(0, 0, 1, 100);
    assertTrue(
        drivetrain.getPose().getRotation().getRadians() > 0.5, drivetrain.getPose().toString());
    assertEquals(0, drivetrain.getPose().getX(), 0.05);
    assertEquals(0, drivetrain.getPose().getY(), 0.05);
  }

  @Test
  void integratesRobotRelativeMotionAtRotatedPoseAndStopsOnRequest() throws InterruptedException {
    drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.fromDegrees(90)));
    runSimulation(1, 0, 0, 75);
    assertEquals(2, drivetrain.getPose().getX(), 0.05);
    assertTrue(drivetrain.getPose().getY() > 3.5, drivetrain.getPose().toString());
    assertEquals(90, drivetrain.getPose().getRotation().getDegrees(), 0.05);

    // runSimulation called stop(); keep the robot enabled to check explicit stopping independently.
    for (int i = 0; i < 50; i++) {
      Unmanaged.feedEnable(100);
      Thread.sleep(Math.round(LOOP_PERIOD_SECONDS * 1000));
      drivetrain.simulationPeriodic();
    }
    for (var state : drivetrain.getModuleStates()) {
      assertEquals(0, state.velocity, 0.05);
    }
    for (var target : drivetrain.getTargetStates()) {
      assertEquals(0, target.velocity);
    }
  }

  private void runSimulation(double forward, double left, double rotation, int steps)
      throws InterruptedException {
    // Phoenix controller output runs on a native thread, so allow real time between 20 ms steps.
    for (int i = 0; i < steps; i++) {
      Unmanaged.feedEnable(100);
      drivetrain.drive(forward, left, rotation);
      Thread.sleep(Math.round(LOOP_PERIOD_SECONDS * 1000));
      drivetrain.simulationPeriodic();
    }
    drivetrain.stop();
  }
}
