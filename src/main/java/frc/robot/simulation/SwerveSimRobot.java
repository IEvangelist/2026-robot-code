package frc.robot.simulation;

import static frc.robot.Constants.SimDriveConstants.*;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.framework.RobotBase;
import org.wpilib.framework.TimedRobot;
import org.wpilib.system.Timer;
import org.wpilib.driverstation.XboxController;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.tunable.Selectable;
import frc.robot.Dashboard;
import org.wpilib.command2.CommandScheduler;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SimTalonFXDrivetrain;

/** Separate desktop entry point: never constructs the real Robot or RobotContainer. */
public final class SwerveSimRobot extends TimedRobot {
  private enum ControlMode {
    STOPPED,
    AUTO_DEMO,
    CONTROLLER
  }

  private final SimTalonFXDrivetrain drivetrain = new SimTalonFXDrivetrain();
  private final XboxController controller =
      new XboxController(OperatorConstants.kDriverControllerPort);
  private final Field2d field = new Field2d();
  private final Selectable<ControlMode> modeChooser = new Selectable<>();
  private ControlMode previousMode = ControlMode.STOPPED;
  private boolean previouslyEnabled;
  private double demoStart;

  public SwerveSimRobot() {
    super(LOOP_PERIOD_SECONDS);
    controller.setLeftXDeadband(0);
    controller.setLeftYDeadband(0);
    controller.setRightXDeadband(0);
    drivetrain.resetPose(new Pose2d(START_X_METERS, START_Y_METERS, new Rotation2d()));
    modeChooser.addDefault("Stopped", ControlMode.STOPPED);
    modeChooser.add("Auto demo", ControlMode.AUTO_DEMO);
    modeChooser.add("Controller", ControlMode.CONTROLLER);
    field.setRobotPose(drivetrain.getPose());
    Dashboard.publishSelection("Swerve Sim/Control Mode", modeChooser);
    Dashboard.publish("Swerve Sim/Field", field);
    Dashboard.publish("Swerve Sim/Modules", drivetrain);
    Dashboard.log("Swerve Sim/Status", "Disabled");
    System.out.println(
        "Swerve simulation ready. Connect Elastic to localhost, choose a control mode, "
            + "then enable Teleoperated in the simulation GUI.");
  }

  public static void main(String... args) {
    if (RobotBase.isReal()) {
      throw new IllegalStateException("The standalone swerve runner is desktop-only.");
    }
    RobotBase.startRobot(SwerveSimRobot::new);
  }

  @Override
  public void robotPeriodic() {
    ControlMode mode = modeChooser.getSelected();
    boolean enabled = org.wpilib.driverstation.RobotState.isEnabled() && !org.wpilib.driverstation.RobotState.isUtility();
    if (mode != previousMode || enabled != previouslyEnabled) {
      drivetrain.stop();
      demoStart = Timer.getTimestamp();
      previousMode = mode;
      previouslyEnabled = enabled;
    }
    if (!enabled) {
      drivetrain.stop();
      Dashboard.log("Swerve Sim/Status", "Disabled (or Test mode)");
    } else {
      switch (mode) {
        case AUTO_DEMO -> runDemo();
        case CONTROLLER -> driveController();
        case STOPPED -> {
          drivetrain.stop();
          Dashboard.log("Swerve Sim/Status", "Stopped");
        }
      }
    }
    // This also calls the standalone subsystem's simulationPeriodic exactly once per cycle.
    CommandScheduler.getInstance().run();
    field.setRobotPose(drivetrain.getPose());
    Dashboard.log("Swerve Sim/Enabled", enabled);
    Dashboard.log("Swerve Sim/Controller Connected", controller.isConnected());
    Dashboard.update();
  }

  SimTalonFXDrivetrain getDrivetrain() {
    return drivetrain;
  }

  private void driveController() {
    if (!controller.isConnected()) {
      drivetrain.stop();
      Dashboard.log("Swerve Sim/Status", "Controller missing: assign Xbox to joystick 0");
      return;
    }
    drivetrain.drive(
        -MathUtil.applyDeadband(controller.getLeftY(), CONTROLLER_DEADBAND)
            * MAX_SPEED_METERS_PER_SECOND,
        -MathUtil.applyDeadband(controller.getLeftX(), CONTROLLER_DEADBAND)
            * MAX_SPEED_METERS_PER_SECOND,
        -MathUtil.applyDeadband(controller.getRightX(), CONTROLLER_DEADBAND)
            * MAX_ANGULAR_RATE_RADIANS_PER_SECOND);
    Dashboard.log("Swerve Sim/Status", "Controller (robot-relative)");
  }

  private void runDemo() {
    int phase = (int) ((Timer.getTimestamp() - demoStart) / DEMO_PHASE_SECONDS) % 7;
    String status;
    switch (phase) {
      case 0 -> {
        drivetrain.drive(DEMO_SPEED_METERS_PER_SECOND, 0, 0);
        status = "Auto: forward";
      }
      case 1 -> {
        drivetrain.drive(-DEMO_SPEED_METERS_PER_SECOND, 0, 0);
        status = "Auto: backward";
      }
      case 2 -> {
        drivetrain.drive(0, DEMO_SPEED_METERS_PER_SECOND, 0);
        status = "Auto: left";
      }
      case 3 -> {
        drivetrain.drive(0, -DEMO_SPEED_METERS_PER_SECOND, 0);
        status = "Auto: right";
      }
      case 4 -> {
        drivetrain.drive(0, 0, DEMO_ANGULAR_RATE_RADIANS_PER_SECOND);
        status = "Auto: counterclockwise";
      }
      case 5 -> {
        drivetrain.drive(0, 0, -DEMO_ANGULAR_RATE_RADIANS_PER_SECOND);
        status = "Auto: clockwise";
      }
      default -> {
        drivetrain.stop();
        status = "Auto: stop";
      }
    }
    Dashboard.log("Swerve Sim/Status", status);
  }

  @Override
  public void close() {
    drivetrain.close();
    field.close();
    org.wpilib.tunable.TunableRegistry.remove(modeChooser);
    Dashboard.remove("Swerve Sim/Field");
    Dashboard.remove("Swerve Sim/Modules");
    super.close();
  }
}
