package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import static com.revrobotics.spark.SparkLowLevel.MotorType.*;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private static SparkMax leader = new SparkMax(ElevatorConstants.leftID, kBrushless);
    private static SparkMax follower = new SparkMax(ElevatorConstants.rightID, kBrushless);

    private static SparkMaxConfig leaderConfig = new SparkMaxConfig();
    private static SparkMaxConfig followerConfig = new SparkMaxConfig();


    public Elevator() {

    }

    public Command goToL4() {
        return Commands.none();
    }

    public Command goToL3() {
        return Commands.none();
    }
    
    public Command goToL2() {
        return Commands.none();
    }

    public Command goToL1() {
        return Commands.none();
    }

    public Command goToIntake() {
        return Commands.none();
    }

    @Override
    public void periodic() {
        
    }
}
