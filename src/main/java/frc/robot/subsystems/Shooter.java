package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.ShooterConstants.*;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
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

    private static boolean hasCoral = false;

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

        // unsure if we actually need to change anything
        // CanandcolorSettings settings =
        //     new CanandcolorSettings()
        //     .setLampLEDBrightness(100);
        // proximity.setSettings(settings);

        // stop when we have successful intake
        shooterClearTrigger.onFalse(runOnce(() -> {
            shoot(0);
            //hasCoral = true;
        }));

    }

    public boolean clearOfElevator() {
        return proximity.getProximity() < .015;
    }

    public Trigger shooterClearTrigger = new Trigger(this::clearOfElevator);

    public Command basicShootCommand() {
        return run(() -> {
            shoot(outtakeSpeed);
            //hasCoral = false;
        });
        // .andThen(new WaitUntilCommand(this::shooterClear))
        // .andThen(() -> shoot(0))
        //.withName("shoot");
    }

    public Command stop() {
        return run(() -> {
            shoot(0);});
        // .andThen(new WaitUntilCommand(this::shooterClear))
        // .andThen(() -> shoot(0))
        //.withName("shoot");
    }


    public Command runSpeedCommand(double speed) {
        return startEnd(
            () -> shoot(speed), 
            () -> shoot(0))
            .withName("Speed: " + speed);
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