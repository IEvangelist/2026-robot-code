package frc.robot;

import com.ctre.phoenix6.CANBus;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.bus.CANPort;

/** Physical mappings must be confirmed before this project can run on SystemCore. */
public final class SystemCoreHardware {
  public static final CANPort AUXILIARY_CAN_PORT = null;
  public static final int EXPANSION_HUB_USB_PORT = -1;
  public static final int LEFT_HOOD_SERVO_CHANNEL = -1;
  public static final int RIGHT_HOOD_SERVO_CHANNEL = -1;

  private SystemCoreHardware() {}

  public static void validate() {
    if (RobotBase.isReal()
        && (AUXILIARY_CAN_PORT == null
            || EXPANSION_HUB_USB_PORT < 0
            || LEFT_HOOD_SERVO_CHANNEL < 0
            || RIGHT_HOOD_SERVO_CHANNEL < 0
            || LEFT_HOOD_SERVO_CHANNEL == RIGHT_HOOD_SERVO_CHANNEL)) {
      throw new IllegalStateException(
          "Configure SystemCoreHardware with verified CAN, Expansion Hub USB, and hood servo ports "
              + "before running on real hardware.");
    }
  }

  public static CANBus auxiliaryBus() {
    validate();
    return RobotBase.isSimulation() ? new CANBus() : new CANBus(AUXILIARY_CAN_PORT);
  }

  public static int expansionHubUsbPort() {
    validate();
    return RobotBase.isSimulation() ? 0 : EXPANSION_HUB_USB_PORT;
  }

  public static int hoodServoChannel(boolean left) {
    validate();
    return RobotBase.isSimulation()
        ? (left ? 0 : 1)
        : (left ? LEFT_HOOD_SERVO_CHANNEL : RIGHT_HOOD_SERVO_CHANNEL);
  }
}
