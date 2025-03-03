// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.States.DriveState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // private DriveState driveState;

  // The gyro sensor
  private final Pigeon2 m_gyro = new Pigeon2(CanIdConstants.kGyroCanId);

  // This section was copied from 2024 code, I think for auto
  public Field2d field = new Field2d();

  // SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
  // DriveConstants.kDriveKinematics,
  // getYaw(),
  // getModulePositions(),
  // new Pose2d());

  // private void configurePathPlanner() {

  // // Configure PathPlanner AutoBuilder
  // AutoBuilder.configureHolonomic(
  // this::getPose, // Robot pose supplier
  // this::resetOdometry, // Method to reset odometry (will be called if your auto
  // has a starting pose)
  // this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT
  // RELATIVE
  // this::driveRobotRelative, // Method that will drive the robot given ROBOT
  // RELATIVE ChassisSpeeds
  // new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should
  // likely live in your
  // // Constants class
  // new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
  // new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
  // 4.5, // Max module speed, in m/s
  // 0.4, // Drive base radius in meters. Distance from robot center to furthest
  // module.
  // new ReplanningConfig() // Default path replanning config. See the API for the
  // options here
  // ),
  // () -> {
  // // Boolean supplier that controls when the path will be mirrored for the red
  // // alliance
  // // This will flip the path being followed to the red side of the field.
  // // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

  // var alliance = DriverStation.getAlliance();
  // if (alliance.isPresent()) {
  // return alliance.get() == DriverStation.Alliance.Red;
  // }
  // return false;
  // },
  // this // Reference to this subsystem to set requirements
  // );
  // }
  // This ends the copied section

  // Odometry class for tracking robot pose
  SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      getYaw(),
      getModulePositions(),
      // Matching 2024 code
      // Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
      // new SwerveModulePosition[] {
      // m_frontLeft.getPosition(), m_frontRight.getPosition(),
      // m_rearLeft.getPosition(), m_rearRight.getPosition() },
      new Pose2d(0, 0, new Rotation2d(0)));

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    // configurePathPlanner();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    SmartDashboard.putNumber("Drive X", getPose().getX());
    SmartDashboard.putNumber("Drive Y", getPose().getY());
  }

  // This next set of statements was copied from 2024 code
  public Rotation2d getYaw() {
    return (DriveConstants.kGyroReversed) ? Rotation2d.fromDegrees((360 - m_gyro.getYaw().getValueAsDouble()))
        : Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble());
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

  // public void turn(double speed) {
  // double direction = speed > 0 ? 1 : -1;
  // double speedWithMinimum = Math.max(0.00, Math.abs(speed)) * direction;

  // drive(0, 0, 0, speedWithMinimum, true, false);
  // }

  // This ends the copied section

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    // return m_odometry.getPoseMeters();
    return m_odometry.getEstimatedPosition();
  }

  public void UpdatePoseFromCameras() {

  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  // public void resetOdometry(Pose2d pose) {
  // m_odometry.resetPosition(
  // Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
  // new SwerveModulePosition[] {
  // m_frontLeft.getPosition(),
  // m_frontRight.getPosition(),
  // m_rearLeft.getPosition(),
  // m_rearRight.getPosition()
  // },
  // pose);
  // }

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
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()).getDegrees();
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
}
