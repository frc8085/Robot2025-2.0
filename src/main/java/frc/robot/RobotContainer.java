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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.EjectCoral;
import frc.robot.commands.PickUpCoralFromSource;
import frc.robot.commands.Windmill;
import frc.robot.commands.ZeroElevator;
import frc.robot.commands.ZeroPivot;
import frc.robot.commands.AutoAlignToAprilTagCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightReefSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class RobotContainer {
        // The robot's subsystems
        private final PivotSubsystem m_PivotArm = new PivotSubsystem();
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final CoralSubsystem m_CoralSubsystem = new CoralSubsystem();
        private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
        private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
        // private final AlgaeSubsystem m_AlgaeSubsystem = new AlgaeSubsystem();
        private final LimelightReefSubsystem m_LimelightReefSubsystem = new LimelightReefSubsystem();

        // The driver's controller
        CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
        CommandXboxController operatorController = new CommandXboxController(OIConstants.kOperaterControllerPort);

        public void adjustJoystickValues() {
                double rawX = driverController.getLeftX();
                double rawY = driverController.getLeftY();

        }

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();

                // Configure default commands
                m_robotDrive.setDefaultCommand(
                                // The right trigger controls the speed of the robot.
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> m_robotDrive.drive(
                                                                MathUtil.applyDeadband(
                                                                                driverController
                                                                                                .getRightTriggerAxis(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(driverController.getLeftY(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(driverController.getLeftX(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(driverController.getRightX(),
                                                                                OIConstants.kDriveDeadband),
                                                                true),
                                                m_robotDrive));

                // Smart Dashboard Buttons
                SmartDashboard.putData("Windmill Home",
                                new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                                Constants.Windmill.WindmillState.Home, false));

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

                // Manual Zero buttons for elevator and pivot
                final Trigger zeroElevator = operatorController.start();
                final Trigger zeroPivot = operatorController.back();

                // SwerveControllerCommand swerveControllerCommand = new
                // SwerveControllerCommand(
                // exampleTrajectory,
                // m_robotDrive::getPose, // Functional interface to feed supplier
                // DriveConstants.kDriveKinematics,
                // // Position controllers
                // new PIDController(AutoConstants.kPXController, 0, 0),
                // new PIDController(AutoConstants.kPYController, 0, 0),
                // thetaController,
                // m_robotDrive::setModuleStates,
                // m_robotDrive);

                // Zero elevator - carriage must be below stage 1 or it will zero where it is
                zeroElevator.onTrue(new ZeroElevator(m_ElevatorSubsystem));
                zeroPivot.onTrue(new ZeroPivot(m_PivotArm));

                // Reset heading of robot for field relative drive
                final Trigger zeroHeadingButton = driverController.start();
                zeroHeadingButton.onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

                // Driver operations
                final Trigger ejectCoral = driverController.b();
                final Trigger pickUpCoral = driverController.x();
                final Trigger ejectAlgae = driverController.a();
                final Trigger pickUpAlgae = driverController.y();
                final Trigger shootAlgaeLeft = driverController.leftBumper();
                final Trigger shootAlgaeRight = driverController.rightBumper();
                final Trigger raiseClimber = driverController.povUp();
                final Trigger lowerClimber = driverController.povDown();
                final Trigger intakeMotorsOff = driverController.back();

                final Trigger AutoAlign = driverController.povLeft();

                // commands that go with driver operations
                ejectCoral.onTrue(new EjectCoral(m_CoralSubsystem, m_ElevatorSubsystem, m_PivotArm));
                pickUpCoral.onTrue(new PickUpCoralFromSource(m_CoralSubsystem, m_ElevatorSubsystem, m_PivotArm));

                AutoAlign.onTrue(new AutoAlignToAprilTagCommand(m_robotDrive, m_LimelightReefSubsystem, 0));
                // ejectAlgae.onTrue(new ScoreAlgae(m_AlgaeSubsystem));
                // shootAlgaeLeft.onTrue(new ScoreAlgaeNetLeft(m_AlgaeSubsystem, m_ElevatorSubsystem, m_PivotArm));
                // shootAlgaeRight.onTrue(new ScoreAlgaeNetRight(m_AlgaeSubsystem, m_ElevatorSubsystem, m_PivotArm));
                // raiseClimber.onTrue(new RunCommand(() -> m_ClimberSubsystem.moveUp(),
                //                 m_ClimberSubsystem))
                //                 .onFalse(new RunCommand(() -> m_ClimberSubsystem.stop(),
                //                                 m_ClimberSubsystem));

                // lowerClimber.onTrue(new RunCommand(() -> m_ClimberSubsystem.moveDown(),
                //                 m_ClimberSubsystem))
                //                 .onFalse(new RunCommand(() -> m_ClimberSubsystem.stop(),
                //                                 m_ClimberSubsystem));
                // pickUpAlgae.whileTrue(new RunCommand(() -> m_AlgaeSubsystem.pickup(), m_AlgaeSubsystem))
                //                 .onFalse(new InstantCommand(m_AlgaeSubsystem::holdAlgae));
                // pickUpAlgae.whileTrue(new RunCommand(() -> algaeSubsystem.pickup(),
                // algaeSubsystem))
                // .onFalse(new PickUpAlgaeL3(algaeSubsystem, elevatorSubsystem,
                // pivotSubsystem));
                // intakeMotorsOff.onTrue(new IntakeMotorsOff(m_CoralSubsystem, m_AlgaeSubsystem));

                // Operator Controls
                // position controls
                final Trigger armHome = operatorController.leftBumper();
                final Trigger algaeGround = operatorController.povDown();
                final Trigger algaeReef2 = operatorController.povRight();
                final Trigger algaeReef3 = operatorController.povUp();
                final Trigger coralDropOff4 = operatorController.y();
                final Trigger coralDropOff3 = operatorController.x();
                final Trigger coralDropOff2 = operatorController.b();
                final Trigger coralDropOff1 = operatorController.a();
                final Trigger altPositionRight = operatorController.rightBumper();



                armHome.onTrue(new SequentialCommandGroup(new PrintCommand("arm button pressed"),
                                new Windmill(m_ElevatorSubsystem, m_PivotArm, Constants.Windmill.WindmillState.Home,
                                                false)));
                algaeGround.onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                Constants.Windmill.WindmillState.AlgaePickUpFloor, false));
                algaeReef2.onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                Constants.Windmill.WindmillState.AlgaePickUpReef2, false));
                algaeReef3.onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                Constants.Windmill.WindmillState.AlgaePickUpReef3, false));
                coralDropOff1.onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                Constants.Windmill.WindmillState.CoralDropOff1, false));
                coralDropOff2.onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                Constants.Windmill.WindmillState.CoralDropOff2, false));
                coralDropOff3.onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                Constants.Windmill.WindmillState.CoralDropOff3, false));
                coralDropOff4.onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                Constants.Windmill.WindmillState.CoralDropOff4, false));



                // alternate positions
                algaeGround.and(altPositionRight).onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                Constants.Windmill.WindmillState.AlgaePickUpFloorFlip, false));
                algaeReef2.and(altPositionRight).onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                Constants.Windmill.WindmillState.AlgaePickUpReef2Flip, false));
                algaeReef3.and(altPositionRight).onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                Constants.Windmill.WindmillState.AlgaePickUpReef3Flip, false));
                coralDropOff1.and(altPositionRight).onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                Constants.Windmill.WindmillState.CoralDropOff1, true));
                coralDropOff2.and(altPositionRight).onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                Constants.Windmill.WindmillState.CoralDropOff2, true));
                coralDropOff3.and(altPositionRight).onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                Constants.Windmill.WindmillState.CoralDropOff3, true));
                coralDropOff4.and(altPositionRight).onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                Constants.Windmill.WindmillState.CoralDropOff4, true));

                // Set Left Joystick for manual elevator/pivot movement
                final Trigger raiseElevator = operatorController.axisLessThan(1, -0.25);
                final Trigger lowerElevator = operatorController.axisGreaterThan(1, 0.25);
                final Trigger pivotClockwise = operatorController.axisGreaterThan(4, 0.25);
                final Trigger pivotCounterClockwise = operatorController.axisLessThan(4,
                                -0.25);

                // commands that go with manual elevator/pivot movement
                pivotClockwise
                                .onTrue(new InstantCommand(() -> m_PivotArm.start()))
                                .onFalse(new InstantCommand(
                                                () -> m_PivotArm.holdPivotArmManual()));
                pivotCounterClockwise.onTrue(new InstantCommand(() -> m_PivotArm.reverse()))
                                .onFalse(new InstantCommand(
                                                () -> m_PivotArm.holdPivotArmManual()));
                raiseElevator.whileTrue(
                                new InstantCommand(() -> m_ElevatorSubsystem.moveUp())
                                                .andThen(new WaitUntilCommand(
                                                                () -> m_ElevatorSubsystem.ElevatorRaiseLimitHit()))
                                                .andThen(new InstantCommand(() -> m_ElevatorSubsystem.holdHeight())))
                                .onFalse(new InstantCommand(
                                                () -> m_ElevatorSubsystem.holdHeight()));
                lowerElevator.whileTrue(
                                new InstantCommand(() -> m_ElevatorSubsystem.moveDown())
                                                .andThen(new WaitUntilCommand(
                                                                () -> m_ElevatorSubsystem.ElevatorLowerLimitHit()))
                                                .andThen(new InstantCommand(() -> m_ElevatorSubsystem.holdHeight())))
                                .onFalse(new InstantCommand(
                                                () -> m_ElevatorSubsystem.holdHeight()));

                // limelight stuff

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
