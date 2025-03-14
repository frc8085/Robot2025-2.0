// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Vector;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FakeConstants;
import frc.robot.Constants.DriveConstants.Direction;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.States.DriveState;
import frc.robot.utils.MAXSwerveModule;
import frc.robot.utils.SwerveUtils;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.geometry.Translation3d;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      CanIdConstants.kFrontLeftDrivingCanId,
      CanIdConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      CanIdConstants.kFrontRightDrivingCanId,
      CanIdConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      CanIdConstants.kRearLeftDrivingCanId,
      CanIdConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      CanIdConstants.kRearRightDrivingCanId,
      CanIdConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final Pigeon2 m_gyro = new Pigeon2(CanIdConstants.kGyroCanId);

  // This section was copied from 2024 code, I think for auto
  public Field2d field = new Field2d();

  // Copied from 6616 - PID Controller for orientation to supplied angle
  private final PIDController orientationController;

  // Odometry class for tracking robot pose
  // SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
  // DriveConstants.kDriveKinematics,
  // getYaw(),
  // getModulePositions(),
  // new Pose2d(0, 0, new Rotation2d(0)));
  SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(getGyroOrientation()),
      getModulePositions(), new Pose2d());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    // configurePathPlanner();

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::updatePose, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) ->

          driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
                                      // optionally outputs individual module feedforwards
          AutoConstants.PP_CONTROLLER,
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // from 6616
    orientationController = new PIDController(AutoConstants.ANGLE_PID.kP, AutoConstants.ANGLE_PID.kI,
        AutoConstants.ANGLE_PID.kD);
    orientationController.enableContinuousInput(-180, 180);

  }

  public void inputCameraPoses(String[] limelightNames) {
    for (String limelightName : limelightNames) {
      if (LimelightHelpers.getTV(limelightName)) {
        // Pose2d pose = limelight.getBotPoseBlue(limelightName);
        Pose2d pose = LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
        double[] limelightArrayData = { pose.getX(), pose.getY(),
            pose.getRotation().getRadians() };
        SmartDashboard.putNumberArray(limelightName, limelightArrayData);
        // System.out.println("Limelight pose: " + pose);
        // double visionTime = Timer.getFPGATimestamp() -
        // limelight.getVisionTime(limelightName);
        double visionTime = Timer.getFPGATimestamp() - LimelightHelpers.getLatency_Pipeline(limelightName) / 1000;
        // int tagCount = limelight.getTagCount(limelightName);
        int tagCount = LimelightHelpers.getTargetCount(limelightName);
        // int primaryId = limelight.getPrimaryId(limelightName);
        int primaryId = (int) LimelightHelpers.getFiducialID(limelightName);
        double distanceToTarget = LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getTranslation()
            .getDistance(new Translation3d());
        this.addVisionPose(pose, visionTime, tagCount, primaryId, distanceToTarget);
      }
    }
  }

  public void addVisionPose(Pose2d pose, double visionTime, int tagCount, int primaryId, double distanceToTarget) {

    double stdDev = 1.5;

    pose = new Pose2d(pose.getTranslation(), Rotation2d.fromDegrees(this.getGyroOrientation()));

    boolean primaryReef = false;
    if (6 <= primaryId && primaryId <= 11) {
      primaryReef = true;
    } else if (17 <= primaryId && primaryId <= 22) {
      primaryReef = true;
    }

    if (tagCount == 0) {
      return;
    }

    if (distanceToTarget > 3.0) {
      return;
    }

    if (primaryReef) {
      if (distanceToTarget <= 1.25) {
        stdDev = 0.1;
      } else if (distanceToTarget <= 1.75) {
        stdDev = 0.5;
      } else {
        return;
      }
    }

    // System.out.println("Adding vision measurement");

    this.m_odometry.addVisionMeasurement(
        // new Pose2d(pose.getTranslation(),
        // Rotation2d.fromDegrees(getGyroOrientation())),
        pose,
        visionTime,
        VecBuilder.fill(stdDev, stdDev, 10.0)
    // VecBuilder.fill(0.1, 0.1, 0.1)
    );
    return;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.updateWithTime(Timer.getFPGATimestamp(),
        Rotation2d.fromDegrees(getGyroOrientation()),
        this.getModulePositions());

    this.inputCameraPoses(new String[] { "limelight-blue", "limelight-yellow" });

    SmartDashboard.putNumber("robot heading", getHeading());
    SmartDashboard.putNumber("robot wrapped heading", getHeadingWrappedDegrees());
    SmartDashboard.putBoolean("fieldRelative", FakeConstants.fieldRelative);
    double[] poseData = { this.getPose().getX(), this.getPose().getY(), this.getPose().getRotation().getRadians() };
    SmartDashboard.putNumberArray("Robot Pose", poseData);
  }

  // Copying code from 6616 for limelight and orienting
  /**
   * Method to drive the robot while it adjusts to a specified orientation.
   *
   * @param xSpeed
   *                  Speed of the robot in the x direction (forward).
   * @param ySpeed
   *                  Speed of the robot in the y direction (sideways).
   * @param direction
   *                  Direction to orient front of robot towards.
   */
  public void driveAndOrient(double speed, double xSpeed, double ySpeed, Direction direction) {
    this.driveAndOrient(speed, xSpeed, ySpeed,
        SwerveUtils.normalizeAngle(SwerveUtils.directionToAngle(direction, this.getHeading())));
  }

  /**
   * Method to drive the robot while it adjusts to a specified orientation.
   *
   * @param xSpeed
   *                      Speed of the robot in the x direction (forward).
   * @param ySpeed
   *                      Speed of the robot in the y direction (sideways).
   * @param targetHeading
   *                      Target heading (angle) robot should face
   */
  public void driveAndOrient(double speed, double xSpeed, double ySpeed, double target) {
    double currentHeading = this.getHeading();
    double targetHeading = SwerveUtils.normalizeAngle(target);

    // The left stick controls translation of the robot.
    // Automatically turn to face the supplied heading
    this.drive(
        speed,
        xSpeed,
        ySpeed,
        this.orientationController.calculate(currentHeading, targetHeading),
        true);
  }

  // This next set of statements was copied from 2024 code
  public Rotation2d getYaw() {
    return (DriveConstants.kGyroReversed) ? Rotation2d.fromDegrees((360 - m_gyro.getYaw().getValueAsDouble()))
        : Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble());
  }

  /**
   * Drive in a robot relative direction.
   * Accepts ChassisSpeeds object for path planner integration.
   * 
   * @param robotRelativeSpeeds
   */
  private void driveRobotRelative(ChassisSpeeds speeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates((targetSpeeds));
    setModuleStates(targetStates);
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    };
  }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

  public void stop() {
    drive(0, 0, 0, 0, false);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose (x / y coordinates and rotation).
   */
  public Pose2d getPose() {
    // return m_odometry.getPoseMeters(); // this was using PoseEstimator
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Updates the odometry to the specified pose.
   *
   * @param pose
   *             The pose to which to set the odometry.
   */
  public void updatePose(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getGyroOrientation()),
        getModulePositions(),
        pose);
  }

  // // This was using the PoseEstimator
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        getYaw(),
        getModulePositions(),
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param speed         How much of the right trigger is pressed.
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double speed, double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    if (xSpeed == 0 && ySpeed == 0) {
      speed = 0;
    }

    double dElevatorSpeed = DriveConstants.kMaxSpeedMetersPerSecond
        - DriveConstants.kMinSpeedMetersPerSecondMaxElevatorHeight;

    // Convert the commanded speeds into the correct units(angle) for the drivetrain
    double joystickAngle = Math.atan2(ySpeed, xSpeed);

    // allows trigger to apply speed and converts an angle into x and y speeds.
    double vy = speed * Math.sin(joystickAngle);
    double vx = speed * Math.cos(joystickAngle);

    double maxDrivableSpeed = DriveState.elevatorMultiplier * dElevatorSpeed
        + DriveConstants.kMinSpeedMetersPerSecondMaxElevatorHeight;

    SmartDashboard.putNumber("Max drivable speed",
        maxDrivableSpeed);

    double xSpeedDelivered = vx * maxDrivableSpeed;
    double ySpeedDelivered = vy * maxDrivableSpeed;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.setYaw(addForAlliance());
  }

  // Inverts the joystick direction if on red alliance
  public double invertForAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      return -1;
    }
    return 1;
  }

  // Adds 180 degrees if on red alliance
  private double addForAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      return 180;
    }
    return 0;
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return SwerveUtils.normalizeAngle(getGyroOrientation());
  }
  // public double getHeading() {
  // return
  // Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()).getDegrees();
  // }

  // more from 6616
  /**
   * Drives the robot to achieve the specified offsets relative to a detected
   * AprilTag.
   * It computes the current forward and lateral distances to the tag using
   * limelightTy and limelightTx,
   * constructs a current relative pose, and then uses the path following
   * controller's
   * calculateRobotRelativeSpeeds method to determine the required robot-relative
   * speeds.
   *
   * @param desiredXOffset Desired forward distance (in meters) from the tag.
   * @param desiredYOffset Desired lateral distance (in meters) from the tag.
   * @param limelightTx    Horizontal offset from the tag (in degrees).
   * @param limelightTy    Vertical offset from the tag (in degrees).
   */
  public void driveToTagOffset(double speed, double desiredXOffset, double desiredYOffset, double limelightTx,
      double limelightTy) {
    // Calculate the current forward distance (x) from the tag using the vertical
    // offset.
    double currentXOffset = (AutoConstants.REEF_APRILTAG_HEIGHT - AutoConstants.LIMELIGHT_HEIGHT_METERS)
        / Math.tan(AutoConstants.LIMELIGHT_MOUNTING_ANGLE_RADIANS + Math.toRadians(limelightTy));

    // Calculate the current lateral distance (y) from the camera center using the
    // horizontal offset.
    double currentYOffset = currentXOffset * Math.tan(Math.toRadians(limelightTx));

    // Construct the current relative pose.
    // The rotation is set from limelightTx so that zero means the limelight is
    // directly facing the tag.
    Pose2d currentRelativePose = new Pose2d(currentXOffset, currentYOffset, Rotation2d.fromDegrees(limelightTx));

    // The desired relative pose has the desired offsets with zero rotation error
    // (directly facing the tag).
    Pose2d desiredRelativePose = new Pose2d(desiredXOffset, desiredYOffset, new Rotation2d(0));

    // Create a target trajectory state using the desired relative pose.
    PathPlannerTrajectoryState targetState = new PathPlannerTrajectoryState();
    targetState.pose = desiredRelativePose;

    // Calculate the robot-relative speeds using the path following controller.
    ChassisSpeeds robotRelativeSpeeds = AutoConstants.PP_CONTROLLER
        .calculateRobotRelativeSpeeds(currentRelativePose, targetState);

    double normalizedX = robotRelativeSpeeds.vxMetersPerSecond / ModuleConstants.kDriveWheelFreeSpeedRps;
    double normalizedY = robotRelativeSpeeds.vyMetersPerSecond / ModuleConstants.kDriveWheelFreeSpeedRps;
    double normalizedRot = robotRelativeSpeeds.omegaRadiansPerSecond / DriveConstants.kMaxAngularSpeed;

    // Command the drivetrain using field-relative control.
    drive(speed, normalizedX, normalizedY, normalizedRot, true);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getAngularVelocityXWorld().getValueAsDouble() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getHeadingWrappedDegrees() {
    return MathUtil.inputModulus(getHeading(), -180, 180);
  }

  public double getGyroOrientation() {
    return m_gyro.getYaw().getValueAsDouble() + DriveConstants.GYRO_OFFSET;
  }

}
