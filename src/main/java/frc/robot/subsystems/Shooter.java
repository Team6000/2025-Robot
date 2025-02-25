package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants.*;

// import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

@Logged
public class Shooter extends SubsystemBase {
    // private static VictorSPX leftShooter = new VictorSPX(leftID);
    // private static VictorSPX rightShooter = new VictorSPX(rightID);
    private static SparkMax left = new SparkMax(leftID, MotorType.kBrushed);
    private static SparkMax right = new SparkMax(rightID, MotorType.kBrushed);

    private static Canandcolor proximity = new Canandcolor(canandcolorID);

    public Shooter() {
    }

    public Command runSpeedCommand(double speed) {
        return startEnd(
            () -> shoot(speed), 
            () -> shoot(0))
            .withName("Speed: " + speed);
    }

    public Command shootL4() {
        return startEnd(
            () -> shoot(L4Speed), 
            () -> shoot(0))
            .withName("L4");
    }

    public Command shootMid() {
        return startEnd(
            () -> shoot(MidSpeed), 
            () -> shoot(0))
            .withName("Mid");
    }

    public Command shootL1() {
        return startEnd(
            () -> shootWithDifferential(L1Speed, L1SpeedDifferential), 
            () -> shoot(0))
            .withName("L1");
    }

    private void shoot(double speed) {
    }

    private void shootWithDifferential(double speed, double differential) {
    }

    @Override
    public void periodic() {
    }
}
