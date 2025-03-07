// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copied from 702

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  public NetworkTable table;
  public NetworkTableEntry tx;
  public NetworkTableEntry ty;
  public NetworkTableEntry tv;
  public NetworkTableEntry ta;
  public NetworkTableEntry pipeline;
  public NetworkTableEntry getpipe;
  public NetworkTableEntry tclass;
  public NetworkTableEntry botpose_wpiblue;
  public NetworkTableEntry botpose_wpired;
  public NetworkTableEntry botpose;
  public NetworkTableEntry camMode;
  public NetworkTableEntry targetpose_cameraspace;
  public NetworkTableEntry camerapose_targetspace;

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight-front");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
    pipeline = table.getEntry("pipeline");
    getpipe = table.getEntry("getpipe");
    tclass = table.getEntry("tclass");
    camMode = table.getEntry("camMode");
    botpose = table.getEntry("botpose");
    botpose_wpiblue = table.getEntry("botpose_wpiblue");
    botpose_wpired = table.getEntry("botpose_wpired");
    targetpose_cameraspace = table.getEntry("targetpose_cameraspace");
    camerapose_targetspace = table.getEntry("targetpose_cameraspace");

  }

  public double getTargetX() {
    return tx.getNumber(0).doubleValue();
  }

  public double getTargetY() {
    return ty.getNumber(0).doubleValue();
  }

  public double getTargetA() {
    return ta.getNumber(0).doubleValue();
  }

  public boolean IsTargetAvailable() {
    return tv.getNumber(0).intValue() == 1 ? true : false;
  }

  public void setPipeline(int value) {
    pipeline.setNumber(value);
  }

  public int getPipeline() {
    return getpipe.getNumber(0).intValue();
  }

  public int getClassifier() {
    return tclass.getNumber(0).intValue();
  }

  public double getBotPoseX() {
    double pose[] = botpose.getDoubleArray(new double[6]);
    return pose[0];
  }

  public double getBotPoseY() {
    double pose[] = botpose.getDoubleArray(new double[6]);
    return pose[1];
  }

  public double[] getBotPoseTeamRelative() {
    var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        if(alliance.get() == DriverStation.Alliance.Red){
          
          return botpose_wpired.getDoubleArray(new double[7]);
        }else{
          return botpose_wpiblue.getDoubleArray(new double[7]);
        }
      }
      return new double[7];
  }

  public Pose2d getBotPose2d(){
    double[] pose = getBotPoseTeamRelative();
    Pose2d botpose = pose.equals(new double[7]) || !IsTargetAvailable()? null: new Pose2d(pose[0], pose[2], new Rotation2d(pose[5]));
    return botpose;
  }

  // public double getBotPoseYTeamRelative() {
  //   double pose[] = RobotContainer.color == Color.kRed ? botpose_wpired.getDoubleArray(new double[6])
  //       : botpose_wpiblue.getDoubleArray(new double[6]);
  //   return pose[1];
  // }

  public double getTargetPos(int value){
    double pos[] = targetpose_cameraspace.getDoubleArray(new double[6]);
    return pos[value];
  }

  public double getCameraPos(int value){
    double pos[] = camerapose_targetspace.getDoubleArray(new double[6]);
    return pos[value];
  }

  public double TargetDistance(){
    return Math.sqrt(Math.pow(getTargetPos(0), 2) + Math.pow(getTargetPos(1), 2));
  }

  public void setCamMode(int value) {
    camMode.setDouble(value);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LLX", getTargetPos(0));
    SmartDashboard.putNumber("LLZ", getTargetPos(2));
    SmartDashboard.putNumber("LLRY", getTargetPos(4));
    

    SmartDashboard.putNumber("tclass", getClassifier());
    SmartDashboard.putNumber("BotPoseX", getBotPoseX());
  }
}
