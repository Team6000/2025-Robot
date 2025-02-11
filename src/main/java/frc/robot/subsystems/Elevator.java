/* https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatortrapezoidprofile/Robot.java */
package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;


import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ElevatorConstants.*;


public class Elevator extends SubsystemBase {
    private static SparkMax leader = new SparkMax(leftID, MotorType.kBrushless);
    private static SparkMax follower = new SparkMax(rightID, MotorType.kBrushless);

    private static SparkMaxConfig leaderConfig = new SparkMaxConfig();
    private static SparkMaxConfig followerConfig = new SparkMaxConfig();

    private static RelativeEncoder mainEncoder = leader.getEncoder();

    private static SparkClosedLoopController mainPIDController = leader.getClosedLoopController();

    private static ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
    private static ProfiledPIDController pid = new ProfiledPIDController(
        kP, kI, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration)); //use the SparkMax PID instead?
    private static TrapezoidProfile motionProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration)
    );

    private Height setpointHeight = Height.Floor;
    private boolean isMoving = false;

    public enum Height {
        L4(L4Height),
        L3(L3Height),
        L2(L2Height),
        L1(L1Height),
        Floor(bottomHeight);

        public final double heightSetpoint;

        Height(double setpoint) {
            this.heightSetpoint = setpoint;
        }
    }

    public Elevator() {
        leaderConfig
            .inverted(leftInverted)
            .smartCurrentLimit(currentLimit);
        leaderConfig.softLimit
            .forwardSoftLimit(forwardSoftLimit)
            .reverseSoftLimit(reverseSoftLimit);
        leaderConfig.closedLoop
            .pidf(kP, kI, kD, kV);
        leaderConfig.closedLoop.maxMotion
                .maxVelocity(maxVelocity)
                .maxAcceleration(maxAcceleration)
                .allowedClosedLoopError(permissibleError)
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        followerConfig
            .apply(leaderConfig)
            .inverted(rightInverted)
            .follow(leader);
        follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public boolean isMoving() {
        return this.isMoving;
    }

    public Trigger elevatorInMotion = new Trigger(this::isMoving);

    public Height getCurrentHeight() {
        return this.setpointHeight;
    }

    public Command goToReefHeightCommand(Height height) {
        if (height == Height.Floor) {
            throw new IllegalArgumentException("Use goToFloorCommand() to send the elevator to the lowest position.");
        }

        final double setpoint = height.heightSetpoint;
        /* switch (height) {
            case L4:
                setpoint = L4Height;
                break;
            case L3:
                setpoint = L3Height;
                break;
            case L2:
                setpoint = L2Height;
                break;
            case L1:
                setpoint = L1Height;
                break;
            case Floor:
                throw new IllegalArgumentException("How did you even get here? ABORT IMMEDIATELY");
            default:
                throw new IllegalArgumentException("How did you even get here? ABORT IMMEDIATELY");
        } */

        return startRun(
            () -> {
                this.isMoving = true;
                this.setpointHeight = height;
                pid.setGoal(setpoint);
            }, 
            () -> {
                //neither are complete, just kinda sorta representative

                //use trapezoidal profile etc
                mainPIDController.setReference(setpoint, ControlType.kMAXMotionPositionControl);

                //OR
                
                var voltage = feedforward.calculate(setpoint) + pid.calculate(mainEncoder.getPosition());
                leader.set(voltage);
            });
    }

    public Command goToFloorCommand() {
        return Commands.none();
    }

    @Override
    public void periodic() {
        
    }

    private final MutVoltage appliedOutput = Volts.mutable(0);
    private final MutAngle angle = Radians.mutable(0);
    private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0);
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            leader::setVoltage,
            log -> {
                log.motor("elevator")
                    .voltage(
                        appliedOutput.mut_replace(leader.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(
                        angle.mut_replace(mainEncoder.getPosition(), Rotations))
                    .angularVelocity(
                        velocity.mut_replace(mainEncoder.getVelocity(), RotationsPerSecond));
            },
            this));
    /**
     * Returns a command that will execute a quasistatic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
