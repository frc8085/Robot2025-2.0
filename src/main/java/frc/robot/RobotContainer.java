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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.EjectCoral;
import frc.robot.commands.PickUpAlgaeL3;
import frc.robot.commands.PickUpCoral;
import frc.robot.commands.PickUpCoralFromSource;
import frc.robot.commands.ScoreAlgae;
import frc.robot.commands.ScoreAlgaeNetLeft;
import frc.robot.commands.ScoreAlgaeNetRight;
import frc.robot.commands.Windmill;
import frc.robot.commands.ZeroElevator;
import frc.robot.commands.ZeroPivot;
import frc.robot.commands.states.ToAlgaeNetLeftCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        private final PivotSubsystem pivotSubsystem = new PivotSubsystem();
        private final DriveSubsystem driveSubsystem = new DriveSubsystem();
        private final CoralSubsystem coralSubsystem = new CoralSubsystem();
        private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
        private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
        private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();

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
                driveSubsystem.setDefaultCommand(
                                // The right trigger controls the speed of the robot.
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> driveSubsystem.drive(
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
                                                                true, false),
                                                driveSubsystem));

                // Smart Dashboard Buttons
                SmartDashboard.putData("Windmill Home",
                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                Constants.Windmill.WindmillState.Home, false));

        }

        private void configureButtonBindings() {

                // Manual Zero buttons for elevator and pivot
                final Trigger zeroElevator = operatorController.start();
                final Trigger zeroPivot = operatorController.back();

                // Zero elevator - carriage must be below stage 1 or it will zero where it is
                zeroElevator.onTrue(new ZeroElevator(elevatorSubsystem));
                zeroPivot.onTrue(new ZeroPivot(pivotSubsystem));

                // Reset heading of robot for field relative drive
                final Trigger zeroHeadingButton = driverController.start();
                zeroHeadingButton.onTrue(new InstantCommand(() -> driveSubsystem.zeroHeading(), driveSubsystem));

                // Driver operations
                final Trigger ejectCoral = driverController.b();
                final Trigger pickUpCoral = driverController.x();
                final Trigger ejectAlgae = driverController.a();
                final Trigger shootAlgaeLeft = driverController.leftBumper();
                final Trigger shootAlgaeRight = driverController.rightBumper();
                final Trigger raiseClimber = driverController.povUp();
                final Trigger lowerClimber = driverController.povDown();

                // commands that go with driver operations
                ejectCoral.onTrue(new EjectCoral(coralSubsystem, elevatorSubsystem, pivotSubsystem));
                pickUpCoral.onTrue(new PickUpCoralFromSource(coralSubsystem, elevatorSubsystem, pivotSubsystem));
                ejectAlgae.onTrue(new ScoreAlgae(algaeSubsystem));
                shootAlgaeLeft.onTrue(new ScoreAlgaeNetLeft(algaeSubsystem, elevatorSubsystem, pivotSubsystem));
                shootAlgaeRight.onTrue(new ScoreAlgaeNetRight(algaeSubsystem, elevatorSubsystem, pivotSubsystem));
                raiseClimber.onTrue(new RunCommand(() -> climberSubsystem.moveUp(),
                                climberSubsystem))
                                .onFalse(new RunCommand(() -> climberSubsystem.stop(),
                                                climberSubsystem));

                lowerClimber.onTrue(new RunCommand(() -> climberSubsystem.moveDown(),
                                climberSubsystem))
                                .onFalse(new RunCommand(() -> climberSubsystem.stop(),
                                                climberSubsystem));

                // Operator Controls
                // position controls
                final Trigger armHome = operatorController.leftBumper();
                final Trigger algaeNet = operatorController.povLeft();
                final Trigger algaeGround = operatorController.povDown();
                final Trigger algaeReef2 = operatorController.povRight();
                final Trigger algaeReef3 = operatorController.povUp();

                armHome.onTrue(new Windmill(elevatorSubsystem, pivotSubsystem, Constants.Windmill.WindmillState.Home,
                                false));
                algaeNet.onTrue(new ToAlgaeNetLeftCommand(elevatorSubsystem, pivotSubsystem));
                algaeGround.onTrue(new Windmill(elevatorSubsystem, pivotSubsystem,
                                Constants.Windmill.WindmillState.AlgaePickUpFloor, false));
                algaeReef2.onTrue(new Windmill(elevatorSubsystem, pivotSubsystem,
                                Constants.Windmill.WindmillState.AlgaePickUpReef2, false));
                algaeReef3.onTrue(new Windmill(elevatorSubsystem, pivotSubsystem,
                                Constants.Windmill.WindmillState.AlgaePickUpReef3, false));

                final Trigger pickUpAlgae = operatorController.a();

                final Trigger altPositionRightIntake = operatorController.rightBumper();

                // Set Left Joystick for manual elevator/pivot movement
                final Trigger raiseElevator = operatorController.axisLessThan(1, -0.25);
                final Trigger lowerElevator = operatorController.axisGreaterThan(1, 0.25);
                final Trigger pivotClockwise = operatorController.axisGreaterThan(4, 0.25);
                final Trigger pivotCounterClockwise = operatorController.axisLessThan(4,
                                -0.25);

                // commands that go with manual elevator/pivot movement
                pivotClockwise
                                .whileTrue(new RunCommand(() -> pivotSubsystem.start(), pivotSubsystem))
                                .onFalse(new InstantCommand(
                                                () -> pivotSubsystem.keepPivot(pivotSubsystem.getCurrentRotation()),
                                                pivotSubsystem));
                pivotCounterClockwise.whileTrue(new RunCommand(() -> pivotSubsystem.reverse(), pivotSubsystem))
                                .onFalse(new InstantCommand(
                                                () -> pivotSubsystem.keepPivot(pivotSubsystem.getCurrentRotation()),
                                                pivotSubsystem));
                raiseElevator.whileTrue(
                                new InstantCommand(elevatorSubsystem::moveUp))
                                .onFalse(new InstantCommand(
                                                () -> elevatorSubsystem.keepHeight(
                                                                elevatorSubsystem.getCurrentMotorPosition())));
                lowerElevator.whileTrue(
                                new InstantCommand(elevatorSubsystem::moveDown))
                                .onFalse(new InstantCommand(
                                                () -> elevatorSubsystem.keepHeight(
                                                                elevatorSubsystem.getCurrentMotorPosition())));

                // coral subsystem

                // algae subsystem
                pickUpAlgae.whileTrue(new RunCommand(() -> algaeSubsystem.pickup(), algaeSubsystem))
                                .onFalse(new PickUpAlgaeL3(algaeSubsystem, elevatorSubsystem, pivotSubsystem));

                // Positions

                // final Trigger pickUpCoral = operatorController.x();
                // final Trigger armHome = operatorController.y();
                // final Trigger pickUpAlgae = operatorController.a();
                // final Trigger ejectAlgae = operatorController.b();

                // coralPickup.onTrue(new Windmill(elevatorSubsystem, pivotSubsystem,
                // Constants.Windmill.WindmillState.CoralPickup, false));
                // algaeGround.onTrue(new Windmill(elevatorSubsystem, pivotSubsystem,
                // Constants.Windmill.WindmillState.AlgaePickUpFloor, false));
                // pickUpAlgae.and(altPositionLeft).onTrue(new Windmill(elevatorSubsystem,
                // pivotSubsystem,
                // Constants.Windmill.WindmillState.CoralDropOff1, false));
                // pickUpAlgae.and(altPositionRightIntake).onTrue(new
                // Windmill(elevatorSubsystem, pivotSubsystem,
                // Constants.Windmill.WindmillState.CoralDropOff1, true));
                // ejectAlgae.and(altPositionLeft).onTrue(new Windmill(elevatorSubsystem,
                // pivotSubsystem,
                // Constants.Windmill.WindmillState.CoralDropOff2, false));
                // ejectAlgae.and(altPositionRightIntake).onTrue(new Windmill(elevatorSubsystem,
                // pivotSubsystem,
                // Constants.Windmill.WindmillState.CoralDropOff2, true));
                // pickUpCoral.and(altPositionLeft).onTrue(new Windmill(elevatorSubsystem,
                // pivotSubsystem,
                // Constants.Windmill.WindmillState.CoralDropOff3, false));
                // pickUpCoral.and(altPositionRightIntake).onTrue(new
                // Windmill(elevatorSubsystem, pivotSubsystem,
                // Constants.Windmill.WindmillState.CoralDropOff3, true));
                // armHome.and(altPositionLeft).onTrue(new Windmill(elevatorSubsystem,
                // pivotSubsystem,
                // Constants.Windmill.WindmillState.CoralDropOff4, false));
                // armHome.and(altPositionRightIntake).onTrue(new Windmill(elevatorSubsystem,
                // pivotSubsystem,
                // Constants.Windmill.WindmillState.CoralDropOff4, true));

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
                                driveSubsystem::getPose, // Functional interface to feed supplier
                                DriveConstants.kDriveKinematics,

                                // Position controllers
                                new PIDController(AutoConstants.kPXController, 0, 0),
                                new PIDController(AutoConstants.kPYController, 0, 0),
                                thetaController,
                                driveSubsystem::setModuleStates,
                                driveSubsystem);

                // Reset odometry to the starting pose of the trajectory.
                driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

                // Run path following command, then stop at the end.
                return swerveControllerCommand.andThen(() -> driveSubsystem.drive(0, 0, 0, 0, false, false));
        }
}
