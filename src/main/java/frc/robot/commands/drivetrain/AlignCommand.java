// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copied from 702

package frc.robot.commands.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDrive;

public class AlignCommand extends Command {
  boolean interrupted;

  private PIDController TranslatePID = new PIDController(

      Constants.AutoFollowConstants.kP,
      Constants.AutoFollowConstants.kI,
      Constants.AutoFollowConstants.kD);

  private PIDController RotatePID = new PIDController(
      Constants.AutoAimConstants.kP,
      Constants.AutoAimConstants.kI,
      Constants.AutoAimConstants.kD);
  private PIDController StrafePID = new PIDController(

      Constants.AutoFollowConstants.kP,
      Constants.AutoFollowConstants.kI,
      Constants.AutoFollowConstants.kD);
  
  
  DoubleSupplier TX;
  DoubleSupplier TZ;
  DoubleSupplier RY;
  BooleanSupplier tv;
  SwerveDrive s_Swerve;
  LimelightSubsystem l_LimelightSubsystem;
  Rotation2d headingprev;
  final double x;
  final double z;
  final double ry;
  

  /** Creates a new AutoAim. */
  public AlignCommand(LimelightSubsystem l_LimelightSubsystem, double x, double z, double ry, SwerveDrive s_Swerve) {
    this.TX = ()-> l_LimelightSubsystem.getTargetPos(0);
    this.TZ = ()-> l_LimelightSubsystem.getTargetPos(2);
    this.RY = ()-> l_LimelightSubsystem.getTargetPos(4);
    this.tv = ()-> l_LimelightSubsystem.IsTargetAvailable();
    this.s_Swerve = s_Swerve;
    this.l_LimelightSubsystem = l_LimelightSubsystem;
    

    this.x = x;
    this.z = z;
    this.ry = ry;

    addRequirements(s_Swerve);
    addRequirements(l_LimelightSubsystem);

    TranslatePID.setTolerance(0.01);
    StrafePID.setTolerance(0.01);
    RotatePID.setTolerance(1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Swerve.setFieldOriented(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    TranslatePID.setSetpoint(x);
    StrafePID.setSetpoint(z);
    RotatePID.setSetpoint(ry);
    

    double x =  l_LimelightSubsystem.getTargetPos(0);
    boolean Target =  l_LimelightSubsystem.IsTargetAvailable();
    double value = TranslatePID.calculate(x);
    //double result = Math.copySign(Math.abs(value) + 0.01, value); 
    double Translate = (Target && !TranslatePID.atSetpoint()  ? MathUtil.clamp(value, -0.87, 0.87) : 0);
    SmartDashboard.putNumber("TPID", value);
    SmartDashboard.putNumber("TTX", x);

    double z =  l_LimelightSubsystem.getTargetPos(2);
    double value1 = StrafePID.calculate(z);
    //double result1 = Math.copySign(Math.abs(value1) + 0.0955, value1); 
    double Strafe = (Target && !StrafePID.atSetpoint()? MathUtil.clamp(value1, -0.87, 0.87) : 0);
    SmartDashboard.putNumber("SPID", value1);
    SmartDashboard.putNumber("STZ", z);

    double a =  l_LimelightSubsystem.getTargetPos(4);
    double value2 = RotatePID.calculate(a);
    //double result2 = Math.copySign(Math.abs(value2) + 0.0955, value2); 
    double Rotate = (Target && !RotatePID.atSetpoint() ? MathUtil.clamp(value2, -0.57, 0.57) : 0);
    SmartDashboard.putNumber("RRY", a);
    SmartDashboard.putNumber("RPID", Rotate);
    var translation = new Translation2d(-Strafe, Translate).times(Constants.DriveConstants.maxDriveSpeedMetersPerSec);
    s_Swerve.drive(
                translation.getX(),
                translation.getY(),
                Rotate * Constants.DriveConstants.maxTurnRateRadiansPerSec);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   s_Swerve.setFieldOriented(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RotatePID.atSetpoint() && TranslatePID.atSetpoint() && StrafePID.atSetpoint();
  }
}
