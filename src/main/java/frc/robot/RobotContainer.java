// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final CoralSubsystem m_CoralSubsystem = new CoralSubsystem();
  private final AlgaeSubsystem m_AlgaeSubsystem = new AlgaeSubsystem();
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperaterControllerPort);

  public void adjustJoystickValues() {
    double rawX = m_driverController.getLeftX();
    double rawY = m_driverController.getLeftY();

  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The right trigger controls the speed of the robot.
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(m_driverController.getRightTriggerAxis(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */

  private void configureButtonBindings() {

    // final Trigger climb = m_operatorController.povDown();

  private void configureButtonBindings(){
      //coral subsystem
    new JoystickButton(m_operatorController, Button.kX.value)
        .onTrue(new RunCommand(() -> m_CoralSubsystem.pickup(), m_CoralSubsystem))
        .onFalse(new RunCommand(() -> m_CoralSubsystem.stop(), m_CoralSubsystem));
    new JoystickButton(m_operatorController, Button.kY.value)
        .onTrue(new RunCommand(() -> m_CoralSubsystem.eject(), m_CoralSubsystem))
        .onFalse(new RunCommand(() -> m_CoralSubsystem.stop(), m_CoralSubsystem));
    new JoystickButton(m_operatorController, Button.kA.value)
        .whileTrue(new RunCommand(() -> m_AlgaeSubsystem.pickup(), m_AlgaeSubsystem))
        .onFalse(new InstantCommand(() -> m_AlgaeSubsystem.holdAlgae(), m_AlgaeSubsystem));
    new JoystickButton(m_operatorController, Button.kB.value)
        .onTrue(new RunCommand(() -> m_AlgaeSubsystem.eject(), m_AlgaeSubsystem))
        .onFalse(new RunCommand(() -> m_AlgaeSubsystem.stop(), m_AlgaeSubsystem));

    /*
     * Dpad Controls
     * Angle 0 = UP
     * Angle 90 = RIGHT
     * Angle 180 = DOWN
     * Angle 270 = LEFT
     */

    new POVButton(m_operatorController, 0)
        .onTrue(new RunCommand(() -> m_ClimberSubsystem.start(), m_ClimberSubsystem))
        .onFalse(new RunCommand(() -> m_ClimberSubsystem.stop(), m_ClimberSubsystem));
    new POVButton(m_operatorController, 180)
        .onTrue(new RunCommand(() -> m_ClimberSubsystem.reverse(), m_ClimberSubsystem))
        .onFalse(new RunCommand(() -> m_ClimberSubsystem.stop(), m_ClimberSubsystem));
  }
    
    .onTrue(new RunCommand(()->m_CoralSubsystem.eject(), m_CoralSubsystem))
    .onFalse(new RunCommand(()->m_CoralSubsystem.stop(), m_CoralSubsystem));

    //elevator subsystem
    new JoystickButton(m_operatorController, Button.kA.value)
    .onTrue(new RunCommand(()->m_ElevatorSubsystem.moveUp(), m_ElevatorSubsystem));
    new JoystickButton(m_operatorController, Button.kB.value)
    .onTrue(new RunCommand(()->m_ElevatorSubsystem.moveDown(), m_ElevatorSubsystem))
    .onFalse(new RunCommand(()->m_ElevatorSubsystem.stop(), m_ElevatorSubsystem));
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, 0, false));
  }
}
