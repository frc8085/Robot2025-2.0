// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TuningModeConstants;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
    private boolean TUNING_MODE = TuningModeConstants.kLimelightTuning;

    /** Creates a new LimelightSubsystem. */
    DriveSubsystem m_drive;

    private boolean m_visionMode;

    public LimelightSubsystem(
            DriveSubsystem drive) {
        m_drive = drive;

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // read values periodically
        // limelight-left, limelight-right
        if (TUNING_MODE || true) {
            // SmartDashboard.putNumber("April tag ID", getAprilTagID("limelight-blue"));
            SmartDashboard.putBoolean("Target", hasTarget("limelight-blue"));
            SmartDashboard.putNumber("LL ID", getID("limelight-blue"));

            SmartDashboard.putNumber("LL X", getX("limelight-blue"));
            SmartDashboard.putNumber("LL Y", getY("limelight-blue"));

            // // SmartDashboard.putNumber("April tag ID",
            // getAprilTagID("limelight-yellow"));
            // SmartDashboard.putBoolean("Target", hasTarget("limelight-yellow"));
            // SmartDashboard.putNumber("LL ID", getID("limelight-yellow"));

            // SmartDashboard.putNumber("LL X", getX("limelight-yellow"));
            // SmartDashboard.putNumber("LL Y", getY("limelight-yellow"));

        }

    }

    public double getX(String limelightName) {
        return LimelightHelpers.getTX(limelightName);
    }

    // Account for the Robot's current heading when returning the X value
    public double getXfromRobotPerspective() {
        return 0;
    }

    public double getY(String limelightName) {
        return LimelightHelpers.getTY(limelightName);
    }

    public double getYfromRobotPerspective() {
        return 0;
    }

    public double getArea(String limelightName) {
        return LimelightHelpers.getTA(limelightName);
    }

    public double getShooterPivotSetpointFromArea() {
        return 0;
    }

    public boolean hasTarget(String limelightName) {
        return LimelightHelpers.getTV(limelightName);
    }

    public int getID(String limelightName) {
        return (int) LimelightHelpers.getFiducialID(limelightName);
    }

    /**
     * @param piplineNumber driver = 0, aprilTags = 1, retroreflective = 2
     */
    // public void setPipeline(int pipelineNumber) {
    // Number numObj = (Number) pipelineNumber;
    // table.getEntry("pipeline").setNumber(numObj);
    // }

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
