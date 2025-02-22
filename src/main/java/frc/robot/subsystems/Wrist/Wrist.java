package frc.robot.subsystems.Wrist;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
    public SparkMax wristMotor;
    private SparkMaxConfig wristMotorConfig;
    private double manualPower = 0;
    private double targetPos = 0;
    private boolean targeting = false;

    // private DigitalInput bottomMagnetSensor;
    public Wrist() {
        wristMotor = new SparkMax(Constants.Wrist.mainMotor, MotorType.kBrushless);
        wristMotorConfig = new SparkMaxConfig();

        wristMotorConfig.closedLoop.p(0.001);
        wristMotorConfig.closedLoop.velocityFF(0.0);
        wristMotorConfig.closedLoop.outputRange(-0.1, 0.1);
        wristMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        wristMotorConfig.idleMode(IdleMode.kBrake);

        // bottomMagnetSensor = new
        // DigitalInput(Constants.Climber.bottomMagnetSensorDIO);
        // wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        // wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        double error = targetPos - wristMotor.getEncoder().getPosition();
        error *= 100;
        double kP = 0.04;

        if (targeting) {
            double PIDpower = error * kP;
            PIDpower = Math.min(0.2, PIDpower);
            PIDpower = Math.max(PIDpower, -0.2);

            // System.out.println(PIDpower);
            wristMotor.set(PIDpower);
        }

        Logger.recordOutput("Wrist/Power", wristMotor.get());
    }

    public void setWristPower(double power) {
        targeting = false;
        power = Math.max(Math.min(power, 1), -1);
        manualPower = power;
        wristMotor.set(manualPower);
    }

    public void setWristVoltage(double voltage) {
        wristMotor.getClosedLoopController().setReference(voltage, SparkMax.ControlType.kVoltage);
    }

    public void setWristPosition(double position, double arbFFVoltage) {
        targetPos = position + Constants.Wrist.globalWristOffset;
        targeting = true;
        // wristMotor.getPIDController().setReference(position,
        // CANSparkMax.ControlType.kPosition, 0, arbFFVoltage,
        // SparkPIDController.ArbFFUnits.kVoltage);
    }

    public void resetPosition(double position) {
        wristMotor.getEncoder().setPosition(position);
    }

    public double getManualMotorPower() {
        return manualPower;
    }

    public double getRelativeTicks() {
        return wristMotor.getEncoder().getPosition();
    }

    public double getAbsoluteTicks() {
        return wristMotor.getEncoder().getPosition();
        // return
        // wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition();
    }
}
