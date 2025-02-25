package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

@Logged
public class Scrubber extends SubsystemBase {
    //yet unclear brushless (sparkmax) vs brushed (victorspx)
    private final DutyCycleEncoder angle = new DutyCycleEncoder(0);

    public Scrubber() {

    }

    private boolean getIsSafe() {
        return true;
    }

    public Trigger isSafe = new Trigger(this::getIsSafe);

    public Command avoidElevatorCollision() {
        return Commands.none().withName("Avoid Elevator Collision");
    }

    public Command manualControl(DoubleSupplier anglePower, BooleanSupplier runEndMotor) {
        return Commands.none().withName("Manual Control");
    }

    @Override
    public void periodic() {
        
    }
}
