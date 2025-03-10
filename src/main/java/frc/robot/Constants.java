// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ControllerConstants {
    public static final int driverControllerPort = 0;
    public static final int operatorControllerPort = 1;
    
    public static final double joystickDeadband = 0.15;
    public static final double triggerPressedThreshhold = 0.25;
  }

  public static class DriveConstants {
    /* FOR REFERENCE: ACTUAL CODE DOESN'T USE THESE */
    /* Swerve Module IDs
     * 1 = Front Left
     * 2 = Front Right
     * 3 = Back Left
     * 4 = Back Right
     */

    /* Swerve Motor IDs
     * 1 = Drive
     * 2 = Turn
     */

    /* Swerve Encoder ID
     * 3
    */

    
    public static final int frontLeftDriveMotorId = 11;
    public static final int frontLeftSteerMotorId = 12;
    public static final int frontLeftSteerEncoderId = 13;

    public static final int frontRightDriveMotorId = 21;
    public static final int frontRightSteerMotorId = 22;
    public static final int frontRightSteerEncoderId = 23;

    public static final int rearLeftDriveMotorId = 31;
    public static final int rearLeftSteerMotorId = 32;
    public static final int rearLeftSteerEncoderId = 33;

    public static final int rearRightDriveMotorId = 41;
    public static final int rearRightSteerMotorId = 42;
    public static final int rearRightSteerEncoderId = 43;
    

    public static final boolean  frontLeftDriveInverted = false;
    public static final boolean frontRightDriveInverted = true;
    public static final boolean   backLeftDriveInverted = false;
    public static final boolean  backRightDriveInverted = true;

    public static final boolean  frontLeftSteerInverted = true;
    public static final boolean frontRightSteerInverted = true;
    public static final boolean   backLeftSteerInverted = true;
    public static final boolean  backRightSteerInverted = true;

    public static final double steerkP = 0.37431;
    public static final double steerkI = 0;
    public static final double steerkD = 0;

    public static final int driveCurrentLimitAmps = 50;
    public static final int steerCurrentLimitAmps = 40;
    
    /**
     * The track width from wheel center to wheel center.
     */
    public static final double trackWidth = Units.inchesToMeters(24.375);
    /**
     * The track length from wheel center to wheel center.
     */
    public static final double wheelBase = Units.inchesToMeters(24.375);
    /**
     * The SwerveDriveKinematics used for control and odometry.
     */
    public static final SwerveDriveKinematics kinematics = 
    new SwerveDriveKinematics(
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),  // front left
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // front right
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // rear left
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // rear right
    );


    /**
     * The gear reduction from the drive motor to the wheel.
     */
    public static final double driveMtrGearReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);

    /**
     * The gear reduction from the steer motor to the wheel.
     */
    public static final double steerMtrGearReduction = (14.0 / 50.0) * (10.0 / 60.0);

    public static final double wheelRadiusMeters = 0.0508;
    public static final double wheelCircumferenceMeters = 2 * wheelRadiusMeters * Math.PI;

    public static final double driveMetersPerEncRev = wheelCircumferenceMeters * driveMtrGearReduction;
    public static final double driveMetersPerSecPerRPM = driveMetersPerEncRev / 60;

    public static final double steerRadiansPerEncRev = 2 * Math.PI * DriveConstants.steerMtrGearReduction;

    public static final double freeMetersPerSecond = 5600 * driveMetersPerSecPerRPM;

    public static final double steerMtrMaxSpeedRadPerSec = 2.0;
    public static final double steerMtrMaxAccelRadPerSecSq = 1.0;

    public static final double maxDriveSpeedMetersPerSec = 2; //Benjy was here 5
    
    public static final double frontLeftModuleOffset  = Units.degreesToRadians(95);
    public static final double frontRightModuleOffset = Units.degreesToRadians(190);
    public static final double backLeftModuleOffset   = Units.degreesToRadians(250);
    public static final double backRightModuleOffset  = Units.degreesToRadians(-15);

    /**
     * The rate the robot will spin with full Rot command.
     */
    public static final double maxTurnRateRadiansPerSec = 2 * Math.PI;

    public static final double ksVolts = 0.667;
    public static final double kvVoltSecsPerMeter = 2.44;
    public static final double kaVoltSecsPerMeterSq = 0.0;
    public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(ksVolts, kvVoltSecsPerMeter, kaVoltSecsPerMeterSq);

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    public static RobotConfig PPRobotConfig;
  }

  public static final class AutoAimConstants {
    public static final double kP = 0.004537;
    public static final double kI = 0.0000;
    public static final double kD = 0.000;

    public static final double AutoAimPIDTolerance = 1.0;
    // public static final double DeflectorPosInValue = 0.0;
    // public static final double DeflectorPosOutValue = 0.0;
  }

  public static final class AutoFollowConstants {
    public static final double kP = 0.271;
    public static final double kI = 0;
    public static final double kD = 0.0;

    public static final double AutoFollowPIDTolerance = 1.0;
    // public static final double DeflectorPosInValue = 0.0;
    // public static final double DeflectorPosOutValue = 0.0;
  }

  public static class ElevatorConstants {
    public static final int leftID = 50; /* leader */
    public static final int rightID = 51; /* follower */

    public static final boolean leftInverted = false; /* unused */
    public static final boolean rightInverted = true; /* unused */

    public static final int currentLimit = 50;

    public static final double forwardSoftLimit = 248;
    public static final double reverseSoftLimit = 0;

    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double maxVelocity = 0;
    public static final double maxAcceleration = 0;
    public static final double permissibleError = 0;

    public static final double L4Height = 248;
    public static final double L3Height = 146;
    public static final double L2Height = 0;
    public static final double L1Height = 0;
    public static final double limelightVisibilityHeight = 0;
    public static final double bottomHeight = 0;

    public static final double scrubberDangerZone = 0;
  }

  public static class ShooterConstants {
    public static final int leftID = 52;
    public static final int rightID = 53;
    public static final int canandcolorID = 56;

    public static final boolean leftInverted = false;
    public static final boolean rightInverted = false;

    public static final int currentLimit = 50;

    public static final int coralDistance = 100;

    public static final int outtakeSpeed = 1;

    // public static final double L4Speed = 1; //0.75??
    // public static final double MidSpeed = 1;
    // public static final double L1Speed = 1;
    // public static final double L1SpeedDifferential = 1;
  }
  
  public static class ScrubberConstants {
    public static final int angleID = 54;
    public static final int flywheelID = 55;
    public static final int encoderChannel = 0;
    
    public static final boolean angleInverted = false;
    public static final boolean flywheelInverted = false;
    public static final boolean encoderInverted = false;

    public static final int angleCurrentLimit = 50;
    public static final int flywheelCurrentLimit = 50;

    public static final double angleFactor = 1;
    public static final double evacuateRate = 1;
    public static final double flywheelSpeed = 1;

    public static final double angleTopLimit = 0;
    public static final double angleBumperLimit = 15;
    public static final double angleBackLimit = 50;
  }

  public static final class AlignConstants{

    public static final double rightRY = 3;
    public static final double rightX = -0.12;
    public static final double rightZ = .54;

    public static final double leftRY = 0;
    public static final double leftX = 0.20;
    public static final double leftZ = 0.54;
    
    public static double centerRY = 0; //11
    public static double centerTX = 0.0;
    public static double centerTZ = 0.0; //0.54
  }

  
}