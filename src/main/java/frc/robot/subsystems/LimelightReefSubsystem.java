// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TuningModeConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.LimelightHelpers;

public class LimelightReefSubsystem extends SubsystemBase {
  private boolean TUNING_MODE = TuningModeConstants.kLimelightTuning;
  private boolean PRACTICE_MODE = TuningModeConstants.kPracticeMode;

  /** Creates a new LimelightSubsystem. */
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  DriveSubsystem m_drive;

  public static HttpCamera m_limelight;

  private boolean m_visionMode;

  public LimelightReefSubsystem(
      DriveSubsystem drive) {
    m_drive = drive;

    m_limelight = new HttpCamera("LL", "http://limelight:5809/stream.mjpg");
    m_limelight.setResolution(320, 240);
    m_limelight.setFPS(90);

    CameraServer.addCamera(m_limelight);

    // Change the camera pose relative to robot center (x forward, y left, z up,
    // degrees)
    LimelightHelpers.setCameraPose_RobotSpace("LL",
        0.5, // Forward offset (meters)
        0.0, // Side offset (meters)
        0.5, // Height offset (meters)
        0.0, // Roll (degrees)
        30.0, // Pitch (degrees)
        0.0 // Yaw (degrees)
    );

  }

  public static double getAprilTagID() {
    double[] id = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid")
        .getDoubleArray(new double[6]);
    return id[0];
  };

  public static double[] getRobotLocation() {
    // gets robot position realtive to AprilTag
    // double[] location = NetworkTableInstance.getDefault().getTable("limelight")
    //     .getEntry("botpose_targetspace")
    //     .getDoubleArray(new double[6]);
    // return location;
    return LimelightHelpers.getBotPose_TargetSpace("LL");
}

  @Override
  public void periodic() {
    if (TUNING_MODE) {
      SmartDashboard.putNumber("April tag ID", getAprilTagID());
      SmartDashboard.putNumberArray("LL Bot Location", getRobotLocation());
      SmartDashboard.putBoolean("Target", hasTarget());
      // SmartDashboard.putNumber("LL ID", getID());

      SmartDashboard.putNumber("LL Reef X", getXRobotRelative());
      SmartDashboard.putNumber("LL Reef Y", getYRobotRelative());

    }

  }

  public double[] getPose3d() {
    return LimelightHelpers.pose3dToArray(LimelightHelpers.getTargetPose3d_RobotSpace("LL")); 
    //CameraSpace: Gets the target's 3D pose with respect to the camera's coordinate system.
    //RobotSpace: Gets the target's 3D pose with respect to the robot's coordinate system.
    
    //ARRAY FORMAT: [x, y, z, roll, pitch, yaw].
  }

  public double getXRobotRelative() {
    //return table.getEntry("tx").getDouble(0.0);
    return getPose3d()[1];
  }

  public double getYRobotRelative() {
    //return table.getEntry("ty").getDouble(0.0);
    return getPose3d()[2];
  }

  public double getArea() {
    //return table.getEntry("ta").getDouble(0.0);
    return LimelightHelpers.getTA("LL");
  }

  public boolean hasTarget() {
    //return table.getEntry("tv").getDouble(0.0) == 1;
    return LimelightHelpers.getTV("LL");
  }

  public void alignToTarget() {
    
  }

  // public int getID() {
  //   //return (int) table.getEntry("tid").getDouble(0.0);
  // }

  /**
   * @param piplineNumber driver = 0, aprilTags = 1, retroreflective = 2
   */
  public void setPipeline(int pipelineNumber) {
    Number numObj = (Number) pipelineNumber;
    table.getEntry("pipeline").setNumber(numObj);
  }

  public boolean inVisionMode() {
    return m_visionMode;
  }

  public void setVisionModeOn() {
    m_visionMode = true;
  }

  public void setVisionModeOff() {
    m_visionMode = false;
  }
}
