package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    SparkMax armMotor;
    SparkMaxConfig armMotorConfig;
    double manualPower = 0;
    private double targetPosition;

    public Arm() {
        armMotor = new SparkMax(Constants.Arm.armMotor, MotorType.kBrushless);
        armMotorConfig = new SparkMaxConfig();
        armMotorConfig.idleMode(IdleMode.kCoast); // was kBrake

        armMotorConfig.closedLoop.p(2); // 1.5
        armMotorConfig.closedLoop.velocityFF(0.5);
        armMotorConfig.closedLoop.outputRange(-0.7, 0.7);
        armMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setBrakes(IdleMode neutralMode) {
        this.armMotorConfig.idleMode(neutralMode);
        armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setTargetAngle(double ticks, double arbFFVoltage) {
        targetPosition = ticks + Constants.Arm.globalArmOffset;
        armMotor.getClosedLoopController().setReference(ticks + Constants.Arm.globalArmOffset,
                SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void periodic() {
        if (DriverStation.isEnabled())
            this.setBrakes(IdleMode.kBrake);

        Logger.recordOutput("Arm/TargetPosition", targetPosition);
        Logger.recordOutput("Arm/RelativeTicks", getRelativeTicks());
        Logger.recordOutput("Arm/AbsoluteTicks", getAbsoluteTicks());
    }

    public void setMotorPower(double power) {
        manualPower = Math.max(Math.min(power, 1), -1);

        armMotor.set(manualPower);
    }

    public double getManualMotorPower() {
        return manualPower;
    }

    public double getRelativeTicks() {
        return armMotor.getEncoder().getPosition();
    }

    public double getAbsoluteTicks() {
        return armMotor.getEncoder().getPosition();
    }

    public double getTargetPosition() {
        return targetPosition;
    }
}
