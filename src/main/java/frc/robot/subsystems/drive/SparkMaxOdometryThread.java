package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.function.Supplier;

import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.Notifier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements
 * to a set of queues.
 *
 * <p>
 * This version is intended for devices like the SparkMax that require polling
 * rather than a
 * blocking thread. A Notifier thread is used to gather samples with consistent
 * timing.
 */
public class SparkMaxOdometryThread {
  private List<Supplier<OptionalDouble>> signals = new ArrayList<>();
  private List<Queue<Double>> queues = new ArrayList<>();
  private List<Queue<Double>> timestampQueues = new ArrayList<>();

  private final Notifier notifier;
  private static SparkMaxOdometryThread instance = null;

  private REVLibError lastDriveError;
  private REVLibError lastTurnError;

  // TODO: Experiment with different capacities, if a motor dies completely it
  // will fill up this queue and potentially slow down the main loop
  public LinkedBlockingQueue<REVLibError> pastDriveErrors = new LinkedBlockingQueue<REVLibError>(20);
  public LinkedBlockingQueue<REVLibError> pastTurnErrors = new LinkedBlockingQueue<REVLibError>(20);
}
