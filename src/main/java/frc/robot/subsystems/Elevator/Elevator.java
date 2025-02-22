package frc.robot.subsystems.Elevator;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    public SparkMax mainMotor;
    private SparkMaxConfig mainMotorConfig;

    private DigitalInput bottomMagnetSensor;
    private DigitalInput topMagnetSensor;

    public double manualPower = 0;

    public Elevator() {
        mainMotor = new SparkMax(Constants.Elevator.mainMotor, MotorType.kBrushless);
        mainMotorConfig = new SparkMaxConfig();

        bottomMagnetSensor = new DigitalInput(Constants.Elevator.bottomMagnetSensorDIO);
        topMagnetSensor = new DigitalInput(Constants.Elevator.topMagnetSensorDIO);

        mainMotorConfig.idleMode(IdleMode.kCoast); // was kBrake

        mainMotorConfig.closedLoop.outputRange(-1.0, 1.0); // STATE was 0.75
        mainMotorConfig.closedLoop.p(0.05);

        mainMotor.configure(mainMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void setElevatorPower(double power) {
        power = Math.max(Math.min(power, 1), -1);
        manualPower = power;
        mainMotor.set(manualPower);
    }

    public void setElevatorVoltage(double voltage) {
        mainMotor.getClosedLoopController().setReference(voltage, SparkMax.ControlType.kVoltage);
    }

    public void setBrakes(IdleMode neutralMode) {
        this.mainMotorConfig.idleMode(neutralMode);
        mainMotor.configure(mainMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        if (DriverStation.isEnabled())
            this.setBrakes(IdleMode.kBrake);

        if (getElevatorSensorState() == ElevatorSensorState.BOTTOM) {
            mainMotor.getEncoder().setPosition(0);
        } else if (getElevatorSensorState() == ElevatorSensorState.TOP) {
            // THE TOP of TH ELEVATOR IN TICKS
            mainMotor.getEncoder().setPosition(-62.3);
        }

        Logger.recordOutput("Elevator/ElevatorSensorState", getElevatorSensorState());
        Logger.recordOutput("Elevator/Power", mainMotor.get());
    }

    /**
     * Combines the two Magnet Sensor inputs to generate an enum that can be used
     * for software limiting
     * 
     * @return {@link ElevatorSensorState} currentState
     */
    public ElevatorSensorState getElevatorSensorState() {
        if (!topMagnetSensor.get())
            return ElevatorSensorState.TOP;
        if (!bottomMagnetSensor.get())
            return ElevatorSensorState.BOTTOM;
        return ElevatorSensorState.MIDDLE;
    }

    public void setElevatorPosition(double position, double arbFFVoltage) {
        mainMotor.getClosedLoopController().setReference(position, SparkMax.ControlType.kPosition,
                ClosedLoopSlot.kSlot0, arbFFVoltage,
                SparkClosedLoopController.ArbFFUnits.kVoltage);
    }

    public void resetPosition(double position) {
        mainMotor.getEncoder().setPosition(position);
    }

    public double getManualMotorPower() {
        return manualPower;
    }

    public double getTicks() {
        return mainMotor.getEncoder().getPosition();
    }
}
