package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    public Shooter() {
    }

    public Command runSpeedCommand(double speed) {
        return startEnd(
            () -> shoot(speed), 
            () -> shoot(0));
    }

    private void shoot(double speed) {
    }

    @Override
    public void periodic() {
    }
}
