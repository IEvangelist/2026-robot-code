package frc.robot;

import java.util.LinkedHashMap;
import java.util.Map;
import org.wpilib.backend.NetworkTablesTelemetryBackend;
import org.wpilib.backend.NetworkTablesTunableBackend;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryLoggable;
import org.wpilib.telemetry.TelemetryRegistry;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.TunableRegistry;
import org.wpilib.tunable.Tunables;

/** Keeps the existing Elastic topic paths while using WPILib 2027 telemetry and tunables. */
public final class Dashboard {
  private static final Map<String, TelemetryLoggable> objects = new LinkedHashMap<>();

  static {
    var instance = NetworkTableInstance.getDefault();
    TelemetryRegistry.registerBackend(
        "/SmartDashboard", new NetworkTablesTelemetryBackend(instance, ""));
    TunableRegistry.registerBackend(
        "/RobotControls", new NetworkTablesTunableBackend(instance, "/Tunables"));
  }

  private Dashboard() {}

  public static void publish(String name, TelemetryLoggable value) {
    objects.put(name, value);
  }

  public static void publishSelection(String name, ComplexTunable value) {
    // Alpha 8 mirrors this Selectable to /SmartDashboard/RobotControls for 2026 dashboards.
    Tunables.publish("/RobotControls/" + name, value);
  }

  public static void log(String name, String value) {
    Telemetry.log("/SmartDashboard/" + name, value);
  }

  public static void log(String name, boolean value) {
    Telemetry.log("/SmartDashboard/" + name, value);
  }

  public static void update() {
    objects.forEach((name, value) -> {
      String path = "/SmartDashboard/" + name;
      if (value instanceof Field2d field) {
        // Elastic 2026 expects its legacy Field2d array, not the 2027 structured Pose2d topic.
        var pose = field.getRobotPose();
        Telemetry.log(path + "/.type", "Field2d");
        Telemetry.log(path + "/robot",
            new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
      } else {
        Telemetry.log(path, value);
      }
    });
  }

  public static void remove(String name) {
    objects.remove(name);
  }
}
