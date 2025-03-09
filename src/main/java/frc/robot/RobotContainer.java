// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.drivetrain.AlignCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.flywheelDirection;
import frc.robot.subsystems.Elevator.Height;

import static frc.robot.Constants.ShooterConstants.outtakeSpeed;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged
public class RobotContainer {
  // Initialize subsystems.
  private final SwerveDrive swerveDrive = new SwerveDrive();
  private final Elevator elevator = new Elevator();
  private final Shooter shooter = new Shooter();
  // private final Arm arm = new Arm();
  private final LimelightSubsystem limelight = new LimelightSubsystem();
  
  @NotLogged
  private final CommandXboxController driverController =
    new CommandXboxController(ControllerConstants.driverControllerPort);
  @NotLogged
  private final CommandXboxController operatorController =
    new CommandXboxController(ControllerConstants.operatorControllerPort);

  // Initialize auto selector.
  private final SendableChooser<Command> autoSelector = AutoBuilder.buildAutoChooser();

  // Create Field2d object to put on Dashboard
  private final Field2d field = new Field2d();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // SmartDashboard.putData("Elevator", elevator);
    SmartDashboard.putData("Swerve", swerveDrive);

    SmartDashboard.putData("auto selector", autoSelector);

    //copied from PP docs.
    SmartDashboard.putData(field);
    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.setRobotPose(pose);
    });
    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.getObject("target pose").setPose(pose);
    });
    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      field.getObject("path").setPoses(poses);
    });
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger (java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
   private void configureBindings() {
    /* */
    //Swerve
    driverController.a().onTrue(new InstantCommand(()-> swerveDrive.setSpeedFactor(2)));
    //driverController.leftTrigger(ControllerConstants.triggerPressedThreshhold).whileTrue(swerveDrive.lockCommand());
    driverController.b().onTrue(new InstantCommand(()-> swerveDrive.setSpeedFactor(6)));
    driverController.back().onTrue(swerveDrive.toggleFieldOrientedCommand());
    driverController.start().onTrue(swerveDrive.resetHeadingCommand());
    driverController.leftStick().whileTrue(swerveDrive.pauseFieldOrientedCommand());


        
    RobotModeTriggers.test().whileTrue(swerveDrive.encodersTestModeCommand());   
    swerveDrive.setDefaultCommand(
        swerveDrive.driveCommand(
          () -> -deadband(driverController.getRawAxis(0)),
          () -> deadband(driverController.getRawAxis(1)),
          () -> -deadband(driverController.getRawAxis(4))
          )
    );

    driverController.x().whileTrue(
      new AlignCommand(limelight, AlignConstants.centerTX, AlignConstants.centerTZ, AlignConstants.centerRY, swerveDrive)
      .andThen(
        Commands.runOnce(
          () -> swerveDrive.drive(0, 0.2, 0),
          swerveDrive)
        .withTimeout(0.5))
      .andThen(
        Commands.runOnce(
          () -> swerveDrive.drive(0, 0, 0), 
          swerveDrive))
      );
    // driverController.x().whileTrue(swerveDrive.turnToIDCommand(() -> (int)LimelightHelpers.getFiducialID("limelight")));


      /* */
      //elevator
      operatorController.y().onTrue(
        elevator.simplestGoToHeightCommand(Height.L4)
        .until(operatorController.leftBumper().or(operatorController.rightBumper()))
        );
      operatorController.a().onTrue(
        elevator.simplestGoToHeightCommand(Height.Floor)
        .until(operatorController.leftBumper().or(operatorController.rightBumper()))        
        );
      operatorController.x().whileTrue(elevator.disableBrakeModeCommand());
      // operatorController.povDown().onTrue(elevator.simplestGoToHeightCommand(Height.L1));
      // operatorController.povRight().onTrue(elevator.simplestGoToHeightCommand(Height.L2));
      // operatorController.povUp().onTrue(elevator.simplestGoToHeightCommand(Height.L3));
      // operatorController.povLeft().onTrue(elevator.simplestGoToHeightCommand(Height.L4));

      operatorController.b().whileTrue(shooter.basicShootCommand());
      operatorController.b().onFalse(shooter.stop());
      





      // operatorController.b().onTrue(
      //   elevator.testMidHeightCommand()
      //   .until(driverController.leftBumper().or(driverController.rightBumper()))
      // );
      // operatorController.x().whileTrue(shooter.shootL4());
      // driverController.x().onTrue(
      //   elevator.testPowerDownCommand()
      //   .until(driverController.leftBumper().or(driverController.rightBumper()))
      // );
      //operatorController.leftStick().onTrue(elevator.setZeroCommand());

      //operatorController.b().onFalse(shooter.Stop());

      //operatorController.pov(0).whileTrue(arm.runFlywheelCommand(flywheelDirection.Forward));
      //operatorController.pov(180).whileTrue(arm.runFlywheelCommand(flywheelDirection.Reverse));

      // arm.setDefaultCommand(
      //   arm.manualAngleControl(operatorController::getRightX)
      // );
  
    /* *

    //Elevator
    // operatorController.button(1).whileTrue(
    //   arm.evacuateCommand()
    //   .andThen(elevator.reefHeightCommand(Elevator.Height.L1)));
    // operatorController.button(2).whileTrue(elevator.reefHeightCommand(Elevator.Height.L2));
    // operatorController.button(3).whileTrue(elevator.reefHeightCommand(Elevator.Height.L3));
    operatorController.button(4).whileTrue(elevator.reefHeightCommand(Elevator.Height.L4));
    //arm
    //Flywheel Forward
    operatorController.button(0).onTrue(
      arm.runFlywheelCommand(flywheelDirection.Forward)
      .until(operatorController.button(0).negate()));
    //Flywheel Reverse
    operatorController.button(0).onTrue(
      arm.runFlywheelCommand(flywheelDirection.Forward)
      .until(operatorController.button(0).negate()));

    driverController.a().whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
    driverController.b().whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));
    driverController.x().whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
    driverController.y().whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));
    */
  }

  /**
   * Deadbands inputs to eliminate tiny unwanted values from the joysticks or gamepad sticks.

   * <p> If the distance between the input and zero is less than the deadband amount, the output will be zero.
   * Otherwise, the value will not change.
   * 
   * @param value The controller value to deadband.
   * @return The deadbanded controller value.
   */
  public double deadband(double value) {
    if (Math.abs(value) < ControllerConstants.joystickDeadband)
        return 0.0;
    return value;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return autoSelector.getSelected();
    return swerveDrive.driveCommand(()-> 0.3, ()-> 0, ()->0).withTimeout(1);
  
    
  }
}


