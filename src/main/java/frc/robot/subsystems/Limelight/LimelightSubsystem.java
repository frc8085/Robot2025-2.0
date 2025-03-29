// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TuningModeConstants;

public class LimelightSubsystem extends SubsystemBase {
    private boolean TUNING_MODE = TuningModeConstants.kLimelightTuning;

    enum Color {
        BLUE, YELLOW
    }

    /** Creates a new LimelightSubsystem. */
    // DriveSubsystem m_drive;

    private boolean m_visionMode;

    private String m_limelightBlue = "limelight-blue";
    private String m_limelightYellow = "limelight-yellow";

    public LimelightSubsystem() {
        // Setting up the limelight pose
        LimelightHelpers.setCameraPose_RobotSpace(m_limelightBlue, LimelightConstants.limelightBlueForward,
                LimelightConstants.limelightBlueSide, LimelightConstants.limelightBlueUp,
                LimelightConstants.limelightBlueRoll, LimelightConstants.limelightBluePitch,
                LimelightConstants.limelightBlueYaw);
        LimelightHelpers.setCameraPose_RobotSpace(m_limelightYellow, LimelightConstants.limelightYellowForward,
                LimelightConstants.limelightYellowSide, LimelightConstants.limelightYellowUp,
                LimelightConstants.limelightYellowRoll, LimelightConstants.limelightYellowPitch,
                LimelightConstants.limelightYellowYaw);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // read values periodically
        SmartDashboard.putBoolean("B Target", hasTarget("limelight-blue"));
        SmartDashboard.putBoolean("Y Target", hasTarget("limelight-yellow"));
        if (TUNING_MODE || true) {
            // SmartDashboard.putNumber("April tag ID", getAprilTagID("limelight-blue"));
            SmartDashboard.putNumber("B LL ID", getID("limelight-blue"));

            SmartDashboard.putNumber("B LL X", getX("limelight-blue"));
            SmartDashboard.putNumber("B LL Y", getY("limelight-blue"));

            // SmartDashboard.putNumber("April tag ID", getAprilTagID("limelight-yellow"));
            SmartDashboard.putNumber("Y LL ID", getID("limelight-yellow"));

            SmartDashboard.putNumber("Y LL X", getX("limelight-yellow"));
            SmartDashboard.putNumber("Y LL Y", getY("limelight-yellow"));
        }

    }

    public double getX(String limelightName) {
        return LimelightHelpers.getTX(limelightName);
    }

    public double getY(String limelightName) {
        return LimelightHelpers.getTY(limelightName);
    }

    public double getArea(String limelightName) {
        return LimelightHelpers.getTA(limelightName);
    }

    public boolean hasTarget(String limelightName) {
        return LimelightHelpers.getTV(limelightName);
    }

    public int getID(String limelightName) {
        return (int) LimelightHelpers.getFiducialID(limelightName);
    }

    public Pose2d getBotPoseBlue(String limelightName) {
        return LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
    }

    public double getVisionTime(String limelightName) {
        return LimelightHelpers.getLatency_Pipeline(limelightName);
    }

    public int getTagCount(String limelightName) {
        return LimelightHelpers.getTargetCount(limelightName);
    }

    public int getPrimaryId(String limelightName) {
        return (int) LimelightHelpers.getFiducialID(limelightName);
    }

    public double getDistanceToTarget(String limelightName) {
        return LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getTranslation()
                .getDistance(new Translation3d());
    }

    /**
     * @param piplineNumber driver = 0, aprilTags = 1, retroreflective = 2
     */

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
