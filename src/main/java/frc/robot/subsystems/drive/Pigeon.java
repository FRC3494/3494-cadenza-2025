package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pigeon {
  private static Pigeon2 pigeon = new Pigeon2(52);

  public static void initialize() {

  }

  public static double isCompassValid() {
    return pigeon.getYaw().getValueAsDouble();
  }

  public static double getYaw() {
    return -pigeon.getYaw().getValueAsDouble();
  }

  public static double getPitch() {
    return pigeon.getPitch().getValueAsDouble();
  }

  public static double getRoll() {
    return pigeon.getRoll().getValueAsDouble();
  }

  public static void putShuffleBoardData() {
    SmartDashboard.putNumber("Current Angle", -pigeon.getYaw().getValueAsDouble());
  }

  public static void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Gyro");
    builder.addDoubleProperty("Value", Pigeon::getYaw, null);
  }

  public static Pigeon2 getPigeon() {
    return pigeon;
  }
}
