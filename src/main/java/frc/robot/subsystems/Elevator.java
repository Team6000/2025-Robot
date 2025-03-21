/* https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatortrapezoidprofile/Robot.java */
package frc.robot.subsystems;


import com.fasterxml.jackson.databind.deser.impl.SetterlessProperty;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ElevatorConstants.*;

// @Logged
public class Elevator extends SubsystemBase {
    private static SparkMax leader = new SparkMax(leftID, MotorType.kBrushless);
    private static SparkMax follower = new SparkMax(rightID, MotorType.kBrushless);

    //private static SparkMaxSim leaderSim = new SparkMaxSim(leader, DCMotor.getNEO(1));

    private static SparkMaxConfig leaderConfig = new SparkMaxConfig();
    private static SparkMaxConfig followerConfig = new SparkMaxConfig();

    private static RelativeEncoder leaderEncoder = leader.getEncoder();
    private static RelativeEncoder followerEncoder = follower.getEncoder();

    //private static SparkClosedLoopController mainPIDController = leader.getClosedLoopController();

    private static ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
    private static ProfiledPIDController pid = new ProfiledPIDController(
        kP, kI, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration)); //use the SparkMax PID instead?
    private static TrapezoidProfile motionProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration)
    );

    private Height setpoint = Height.Floor;
    @NotLogged
    private TrapezoidProfile.State reference = new TrapezoidProfile.State();
    @NotLogged
    private Timer timer = new Timer();
    private boolean isMoving = false;

    public enum Height {
        L4(L4Height),
        L3(L3Height),
        L2(L2Height),
        L1(L1Height),
        Limelight(limelightVisibilityHeight),
        Floor(bottomHeight);

        public final double height;

        Height(double setpoint) {
            this.height = setpoint;
        }
    }

    public Elevator() {
        leaderConfig
            .smartCurrentLimit(currentLimit);
        leaderConfig.softLimit
            .forwardSoftLimit(forwardSoftLimit)
            .reverseSoftLimit(reverseSoftLimit);
        // leaderConfig.closedLoop
        //     .pidf(kP, kI, kD, kV);
        leaderConfig.closedLoop.maxMotion
                .maxVelocity(maxVelocity)
                .maxAcceleration(maxAcceleration)
                .allowedClosedLoopError(permissibleError)
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        followerConfig
            .apply(leaderConfig)
            .follow(leader, true);
        follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // this.setDefaultCommand(this.reefHeightCommand(Height.Floor));
    }

    public boolean getIsMoving() {
        return this.isMoving;
    }

    public Trigger elevatorInMotion = new Trigger(this::getIsMoving);

    public Height getSetpoint() {
        return this.setpoint;
    }

    public double getCurrentHeight() {
        return Math.max(leaderEncoder.getPosition(), followerEncoder.getPosition());
    }
    public double getCurrentVelocity() {
        return Math.max(leaderEncoder.getVelocity(), followerEncoder.getVelocity());
    }

    public double remainingTimeToGoal() {
        //hope this works?
        return motionProfile.timeLeftUntil(setpoint.height);
    }
    public double totalTimeToGoal() {
        return motionProfile.totalTime();
    }
    public boolean isAtSetpoint() {
        return motionProfile.isFinished(timer.get());
    }

    public Trigger inScrubberDangerZone = new Trigger(() -> getCurrentHeight() < scrubberDangerZone);

    public Command reefHeightCommand(Height height) {
        // if (height == Height.Floor) {
        //     throw new IllegalArgumentException("Use goToFloorCommand() to send the elevator to the lowest position.");
        // }

        return run(
            () -> {
                if (this.setpoint != height) {
                    this.isMoving = true;
                    this.setpoint = height;
                    // pid.setGoal(setpoint);
                    timer.reset();
                }
            }
            // , () -> {}
            ).withName(height.toString());
    }

    /** thought this command might be necessary for special case of avoiding algae mech.
      * this doesn't currently seem to be the case though *
    public Command goToFloorCommand() {
        return startEnd(
            () -> {
                if (this.setpoint != Height.Floor) {
                    this.isMoving = true;
                    this.setpoint = Height.Floor;
                    // pid.setGoal(setpoint)
                    timer.reset();
                }
            },
            () -> {}
            ).withName("Floor");
    }
    */

    public Command simplestGoToHeightCommand(Height setpoint) {
        // SmartDashboard.putNumber(key, mainEncoder.getPosition());

        if (setpoint.height < 0) throw new IllegalArgumentException("Setpoint is too low");
        if (setpoint.height > 250) throw new IllegalArgumentException("Setpoint is too high");

        return run(() -> {
            var position = getCurrentHeight();
            if (position < setpoint.height) {
                leader.set(0.6);
            }  else if (position > setpoint.height + 5) {
                leader.set(-0.6);
            } else {
                leader.stopMotor();
            }
        })
        .finallyDo(interrupted -> leader.stopMotor())
        .withName("Go to" + setpoint);
    }

    public Command setZeroCommand() {
        return runOnce(() -> leaderEncoder.setPosition(0));
    }
    public Command disableBrakeModeCommand() {
        return startEnd(
            () -> {
                leaderConfig.idleMode(IdleMode.kCoast);
                followerConfig.idleMode(IdleMode.kCoast);
                leader.configure(leaderConfig, null, null);
                follower.configure(followerConfig, null, null);
            }, 
            () -> {
                leaderConfig.idleMode(IdleMode.kBrake);
                followerConfig.idleMode(IdleMode.kBrake);
                leader.configure(leaderConfig, null, null);
                follower.configure(followerConfig, null, null);
            });
    }




    @Override
    public void periodic() {
        /*
        //use trapezoidal profile etc
        reference = motionProfile.calculate(
            // timer.get(), //time since new setpoint was commanded
            0.02, //20ms
            new TrapezoidProfile.State(mainEncoder.getPosition(), mainEncoder.getVelocity()), //current state
            new TrapezoidProfile.State(setpoint.height, 0)); //goal state
        var ffout = feedforward.calculate(reference.velocity);
        
        mainPIDController.setReference(
            reference.position,
            ControlType.kMAXMotionPositionControl,
            ClosedLoopSlot.kSlot0, //default slot
            ffout);

        this.isMoving = !MathUtil.isNear(setpoint.height, mainEncoder.getPosition(), permissibleError);

        leaderSim.iterate(reference.velocity, 12, 0.02);

        SmartDashboard.putNumber("height", reference.position);
        leader.configAccessor.getIdleMode();

        //OR
        
        // var voltage = feedforward.calculate(setpoint) + pid.calculate(mainEncoder.getPosition());
        // leader.set(voltage);
        */

    }

    private void setVoltage(Voltage voltage) {
        leader.setVoltage(voltage);
        follower.setVoltage(voltage);
    }

    private final MutVoltage appliedOutput = Volts.mutable(0);
    private final MutAngle angle = Radians.mutable(0);
    private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0);
    @NotLogged
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            this::setVoltage,
            log -> {
                log.motor("elevator")
                    .voltage(
                        appliedOutput.mut_replace(leader.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(
                        angle.mut_replace(getCurrentHeight(), Rotations))
                    .angularVelocity(
                        velocity.mut_replace(getCurrentVelocity(), RotationsPerSecond));
            },
            this));
    /**
     * Returns a command that will execute a quasistatic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction).withName("Quasi " + direction);
    }
    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction).withName("Dynamic " + direction);
    }
}
