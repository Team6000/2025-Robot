package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.ScrubberConstants.*;

@Logged
public class Arm extends SubsystemBase {
    private final SparkMax angle = new SparkMax(angleID, MotorType.kBrushless);
    private final SparkMax flywheel = new SparkMax(flywheelID, MotorType.kBrushless);

    @NotLogged
    private final SparkMaxConfig angleConfig = new SparkMaxConfig();
    @NotLogged
    private final SparkMaxConfig flywheelConfig = new SparkMaxConfig();

    private final DutyCycleEncoder encoder = new DutyCycleEncoder(encoderChannel);

    public enum flywheelDirection {
        Forward,
        Reverse,
        Stop
    }

    public Arm() {
        angleConfig
            .inverted(angleInverted)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(angleCurrentLimit);
        angle.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        flywheelConfig
            .inverted(flywheelInverted)
            .smartCurrentLimit(flywheelCurrentLimit);
        flywheel.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder.setAssumedFrequency(975.6); // from product page
        encoder.setInverted(encoderInverted);

    }

    public boolean getIsSafe() {
        return encoder.get() >= angleBumperLimit;
    }

    public Trigger isSafe = new Trigger(this::getIsSafe);


    public Command evacuateCommand() {
        return runOnce(() -> angle.set(evacuateRate))
                .until(isSafe)
                .andThen(angle::stopMotor)
                .withName("Evacuate");
    }

    public Command manualAngleControl(DoubleSupplier anglePower) {
        return run(() -> {
            angle.set(anglePower.getAsDouble());
        })
        .withName("Manual Angle");
    }

    public Command runFlywheelCommand(flywheelDirection direction) {
        return startEnd(
            () -> runFlywheel(direction), 
            () -> runFlywheel(flywheelDirection.Stop));
    }

    private void runFlywheel(flywheelDirection direction) {
        switch (direction) {
            case Forward:
                flywheel.set(flywheelSpeed);
                break;
            case Reverse:
                flywheel.set(-flywheelSpeed);
                break;
            case Stop:
                flywheel.set(0);
                break;
        }
    }

    @Override
    public void periodic() {
        
    }
}
