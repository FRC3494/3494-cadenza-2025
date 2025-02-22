package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

public class Pigeon {
  private static Pigeon2 pigeon = new Pigeon2(52);

  public static void initialize() {

  }

  public static double isCompassValid() {
    return pigeon.getYaw().getValueAsDouble();
  }
}
