package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants.*;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

@Logged
public class Shooter extends SubsystemBase {
    private static SparkMax left = new SparkMax(leftID, MotorType.kBrushed);
    private static SparkMax right = new SparkMax(rightID, MotorType.kBrushed);

    @NotLogged
    private static SparkMaxConfig leftConfig = new SparkMaxConfig();
    @NotLogged
    private static SparkMaxConfig rightConfig = new SparkMaxConfig();

    private static Canandcolor proximity = new Canandcolor(canandcolorID);

    public Shooter() {
        leftConfig
            .inverted(leftInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(currentLimit);

        rightConfig
            .apply(leftConfig)
            .inverted(rightInverted);
        
        left.configure(leftConfig, null, null);
        right.configure(rightConfig, null, null);
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
        left.set(speed);
        right.set(speed);
    }

    private void shootWithDifferential(double speed, double differential) {
        left.set(speed);
        right.set(speed * differential);
    }

    @Override
    public void periodic() {
    }
}