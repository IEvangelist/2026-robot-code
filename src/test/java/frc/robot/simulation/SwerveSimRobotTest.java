package frc.robot.simulation;

import static frc.robot.Constants.SimDriveConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import org.wpilib.hardware.hal.HAL;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.GenericHIDSim;
import org.wpilib.simulation.SimHooks;
import org.wpilib.smartdashboard.Field2d;
import frc.robot.Dashboard;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SimTalonFXDrivetrain;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SwerveSimRobotTest {
  private SwerveSimRobot robot;

  @BeforeAll
  static void startTestNetworkTables() {
    // Let tests run while the student's desktop simulation already owns ports 1735/5810.
    NetworkTableInstance.getDefault().startServer("", "127.0.0.1", "", 0);
  }

  @AfterAll
  static void stopTestNetworkTables() {
    NetworkTableInstance.getDefault().stopServer();
  }

  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize());
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    SimHooks.pauseTiming();
    // Clear selections published by earlier tests before the new Selectable subscribes.
    NetworkTableInstance.getDefault()
        .getTable("SmartDashboard/RobotControls/Swerve Sim/Control Mode")
        .getEntry("selected").setString("Stopped");
    NetworkTableInstance.getDefault()
        .getTable("Tunables/RobotControls/Swerve Sim/Control Mode/selected")
        .getEntry("tune").setString("Stopped");
    robot = new SwerveSimRobot();
  }

  @AfterEach
  void tearDown() {
    if (robot != null) {
      robot.close();
    }
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    SimHooks.resumeTiming();
  }

  @Test
  void publishesElasticWidgetTypesAndStartsStopped() {
    var pose = robot.getDrivetrain().getPose();
    assertEquals(START_X_METERS, pose.getX());
    assertEquals(START_Y_METERS, pose.getY());
    tick();
    var table = NetworkTableInstance.getDefault().getTable("SmartDashboard/Swerve Sim");
    assertEquals("Field2d", table.getSubTable("Field").getEntry(".type").getString(""));
    assertEquals("SwerveDrive", table.getSubTable("Modules").getEntry(".type").getString(""));
    assertEquals("String Chooser", NetworkTableInstance.getDefault().getTable("SmartDashboard/RobotControls/Swerve Sim/Control Mode").getEntry(".type").getString(""));
    assertEquals("Stopped", NetworkTableInstance.getDefault().getTable("SmartDashboard/Swerve Sim").getEntry("Status").getString(""));
    assertStopped();
    assertTrue(table.getSubTable("Modules").getEntry("Front Left Velocity").exists());
  }

  @Test
  void demoCyclesThroughAllPhasesAndRestartsAfterDisable() {
    selectMode("Auto demo");
    for (String phase :
        new String[] {"forward", "backward", "left", "right", "counterclockwise", "clockwise", "stop"}) {
      assertEquals("Auto: " + phase, NetworkTableInstance.getDefault().getTable("SmartDashboard/Swerve Sim").getEntry("Status").getString(""));
      // We invoke the loop ourselves; do not wait for TimedRobot's inactive notifier.
      SimHooks.stepTimingAsync(DEMO_PHASE_SECONDS);
      tick();
    }
    assertEquals("Auto: forward", NetworkTableInstance.getDefault().getTable("SmartDashboard/Swerve Sim").getEntry("Status").getString(""));
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    tick();
    assertFalse(NetworkTableInstance.getDefault().getTable("SmartDashboard/Swerve Sim").getEntry("Enabled").getBoolean(true));
    assertStopped();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    tick();
    assertEquals("Auto: forward", NetworkTableInstance.getDefault().getTable("SmartDashboard/Swerve Sim").getEntry("Status").getString(""));
  }

  @Test
  void missingControllerStopsInsteadOfFallingBackToDemo() {
    selectMode("Auto demo");
    selectMode("Controller");
    assertTrue(NetworkTableInstance.getDefault().getTable("SmartDashboard/Swerve Sim").getEntry("Status").getString("").startsWith("Controller missing"));
    assertStopped();
    selectMode("Stopped");
    assertEquals("Stopped", NetworkTableInstance.getDefault().getTable("SmartDashboard/Swerve Sim").getEntry("Status").getString(""));
  }

  @Test
  void controllerAxesDriveAndDisconnectClearsRequests() {
    var controller = new GenericHIDSim(OperatorConstants.kDriverControllerPort);
    controller.setAxesMaximumIndex(5);
    controller.setAxesAvailable(0b111111);
    controller.setGamepadType(org.wpilib.driverstation.GenericHID.HIDType.XBOX_ONE);
    controller.setButtonsMaximumIndex(10);
    controller.setButtonsAvailable(0b11111111110);
    controller.setRawAxis(1, -0.5);
    controller.notifyNewData();
    selectMode("Controller");
    assertEquals("Controller (robot-relative)", NetworkTableInstance.getDefault().getTable("SmartDashboard/Swerve Sim").getEntry("Status").getString(""));
    var drivetrain = robot.getDrivetrain();
    for (var target : drivetrain.getTargetStates()) {
      assertTrue(target.velocity > 0);
      assertEquals(0, target.angle.getRadians(), 1e-9);
    }
    controller.setAxesMaximumIndex(-1);
    controller.setAxesAvailable(0);
    controller.setButtonsMaximumIndex(0);
    controller.setButtonsAvailable(0);
    controller.notifyNewData();
    tick();
    assertStopped();
  }

  private void selectMode(String mode) {
    NetworkTableInstance.getDefault()
        .getTable("SmartDashboard/RobotControls/Swerve Sim/Control Mode")
        .getEntry("selected")
        .setString(mode);
    org.wpilib.tunable.TunableRegistry.update();
    tick();
  }

  private void tick() {
    robot.robotPeriodic();
    org.wpilib.tunable.TunableRegistry.update();
  }

  private void assertStopped() {
    var drivetrain = robot.getDrivetrain();
    for (var target : drivetrain.getTargetStates()) {
      assertEquals(0, target.velocity);
    }
  }
}
