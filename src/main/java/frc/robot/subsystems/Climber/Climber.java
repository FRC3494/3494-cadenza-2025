package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.ElevatorSensorState;

public class Climber extends SubsystemBase {
    public SparkMax mainMotor;
    private SparkMaxConfig mainMotorConfig;
    public double manualPower = 0;

    public RatchetServo ratchetServo = new RatchetServo(Constants.Climber.pwmServoPort);
    public boolean ratchetEngaged = false;

    private DigitalInput bottomMagnetSensor;

    public Climber() {
        mainMotor = new SparkMax(Constants.Climber.mainMotor, MotorType.kBrushless);
        mainMotorConfig = new SparkMaxConfig();

        mainMotorConfig.idleMode(IdleMode.kCoast);

        mainMotorConfig.closedLoop.outputRange(-1, 1);
        mainMotorConfig.closedLoop.p(0.1);

        bottomMagnetSensor = new DigitalInput(Constants.Climber.bottomMagnetSensorDIO);

        mainMotor.configure(mainMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getCurrentPosition() {
        return mainMotor.getEncoder().getPosition();
    }

    public void disenageRatchet() {
        System.out.println("disengaged: rachet to 0.5");
        ratchetServo.set(0.3);
        ratchetEngaged = false;
    }

    public void engageRatchet() {
        System.out.println("engaged: rachet to 1.0");
        ratchetServo.set(.9);
        ratchetEngaged = true;
    }

    public void setElevatorPower(double power) {
        power = Math.max(Math.min(power, 1), -1);
        manualPower = power;
        mainMotor.set(manualPower);
    }

    @Override
    public void periodic() {
        if (getClimberSensorState() == ElevatorSensorState.BOTTOM) {
            resetPosition(0);
        }

        Logger.recordOutput("Climber/ClimberPower", manualPower);
        Logger.recordOutput("Climber/RatchetEngaged", ratchetEngaged);
    }

    public ElevatorSensorState getClimberSensorState() {
        if (!bottomMagnetSensor.get())
            return ElevatorSensorState.BOTTOM;
        return ElevatorSensorState.MIDDLE;
    }

    public void setElevatorVoltage(double voltage) {
        mainMotor.getClosedLoopController().setReference(voltage, SparkMax.ControlType.kVoltage);
    }

    public void setElevatorPosition(double position, double arbFFVoltage) {
        mainMotor.getClosedLoopController().setReference(position, SparkMax.ControlType.kPosition,
                ClosedLoopSlot.kSlot0, arbFFVoltage, SparkClosedLoopController.ArbFFUnits.kVoltage);
    }

    public void resetPosition(double position) {
        mainMotor.getEncoder().setPosition(position);
    }

    public double getTicks() {
        return mainMotor.getEncoder().getPosition();
    }

    public double getMotorPower() {
        return mainMotor.getAppliedOutput();
    }

    public double getManualMotorPower() {
        return manualPower;
    }
}
