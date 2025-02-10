package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Shooter extends SubsystemBase {
    private static VictorSPX leftShooter = new VictorSPX(leftShooterID);
    private static VictorSPX rightShooter = new VictorSPX(rightShooterID);

    public Shooter() {
    }

    public Command runSpeedCommand(double speed) {
        return startEnd(
            () -> shoot(speed), 
            () -> shoot(0));
    }

    public Command shootL4() {
        return startEnd(
            () -> shoot(L4Speed), 
            () -> shoot(0));
    }

    public Command shootMid() {
        return startEnd(
            () -> shoot(MidSpeed), 
            () -> shoot(0));
    }

    public Command shootL1() {
        return startEnd(
            () -> shootWithDifferential(L1Speed, L1SpeedDifferential), 
            () -> shoot(0));
    }

    private void shoot(double speed) {
    }

    private void shootWithDifferential(double speed, double differential) {
    }

    @Override
    public void periodic() {
    }
}
