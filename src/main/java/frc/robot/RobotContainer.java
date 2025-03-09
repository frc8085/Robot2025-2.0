// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DropCoral;
import frc.robot.commands.EjectCoral;
import frc.robot.commands.InitializePivotAndElevator;
import frc.robot.commands.LockPivotAndElevatorCommand;
import frc.robot.commands.PickUpAlgae;
import frc.robot.commands.PickUpAlgaeFromGround;
import frc.robot.commands.PickUpCoralFromSource;
import frc.robot.commands.Pivot;
import frc.robot.commands.RetractClimb;
import frc.robot.commands.EjectAlgae;
import frc.robot.commands.Windmill;
import frc.robot.commands.ZeroElevator;
import frc.robot.commands.autoCommands.*;
import frc.robot.commands.automated.AlignAndDriveBlue;
import frc.robot.commands.automated.AlignAndDriveYellow;
import frc.robot.commands.movement.AlignToAprilTagBlue;
import frc.robot.commands.movement.AlignToAprilTagYellow;
import frc.robot.commands.movement.AutoDriveMeters;
import frc.robot.commands.movement.AutoMoveForwardForTime;
import frc.robot.commands.movement.AutoMoveForwardForTimeFaster;
import frc.robot.commands.movement.AutoPositionLeftRight;
import frc.robot.commands.movement.DriveToReefBlue;
import frc.robot.commands.movement.DriveToReefYellow;
import frc.robot.commands.movement.NewAutoMoveOnReef;
import frc.robot.commands.scoring.ScoreAlgaeNetBlue;
import frc.robot.commands.scoring.ScoreAlgaeNetYellow;
import frc.robot.commands.scoring.ScoreCoralL4;
import frc.robot.commands.DeployClimb;
import frc.robot.commands.scoring.ScoreCoralL1;
import frc.robot.commands.scoring.ScoreCoralL2;
import frc.robot.commands.scoring.ScoreCoralL3;
import frc.robot.commands.scoring.ScoreCoralL4;
import frc.robot.commands.sequences.RemoveAlgaeL2;
import frc.robot.commands.sequences.RemoveAlgaeL3;
import frc.robot.commands.sequences.RemoveAlgaeL3ScoreL3;
import frc.robot.commands.PickUpAlgaeFromReef;
import frc.robot.commands.states.ToAlgaeGround;
import frc.robot.commands.states.ToAlgaeL2;
import frc.robot.commands.states.ToAlgaeL3;
import frc.robot.commands.states.ToCoralDropOff1;
import frc.robot.commands.states.ToCoralDropOff2;
import frc.robot.commands.states.ToCoralDropOff3;
import frc.robot.commands.states.ToCoralDropOff4;
import frc.robot.commands.states.ToCoralSource;
import frc.robot.commands.states.ToCoralSourceManual;
import frc.robot.commands.states.ToHomeCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
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
        private final LimelightSubsystem limelight = new LimelightSubsystem(driveSubsystem);

        private final SendableChooser<Command> autoChooser;
        protected SendableChooser<Alliance> allianceColor = new SendableChooser<>();

        private final Field2d field;

        // Register Named Commands for PathPlanner
        private void configureAutoCommands() {
                NamedCommands.registerCommand("InitializePE",
                                new InitializePivotAndElevator(pivotSubsystem, elevatorSubsystem));
                NamedCommands.registerCommand("AutoYCoral1",
                                new AutoCoral1(coralSubsystem, elevatorSubsystem, pivotSubsystem, true));
                NamedCommands.registerCommand("AutoYCoral2",
                                new AutoCoral2(coralSubsystem, elevatorSubsystem, pivotSubsystem, true));
                NamedCommands.registerCommand("AutoYCoral3",
                                new AutoCoral3(coralSubsystem, elevatorSubsystem, pivotSubsystem, true));
                NamedCommands.registerCommand("AutoYCoral4",
                                new AutoCoral4(coralSubsystem, elevatorSubsystem, pivotSubsystem, true));
                NamedCommands.registerCommand("AutoBCoral4",
                                new AutoCoral4(coralSubsystem, elevatorSubsystem, pivotSubsystem, false));
                NamedCommands.registerCommand("AutoCoralSource",
                                new AutoCoralSource(coralSubsystem, elevatorSubsystem, pivotSubsystem, false));
                NamedCommands.registerCommand("AutoYLimelightRight",
                                new AutoLimelightPosition(driveSubsystem, limelight, true, true));
                NamedCommands.registerCommand("AutoYLimelightLeft",
                                new AutoLimelightPosition(driveSubsystem, limelight, false, true));
                NamedCommands.registerCommand("AutoBLimelightRight",
                                new AutoLimelightPosition(driveSubsystem, limelight, true, false));
                NamedCommands.registerCommand("AutoBLimelightLeft",
                                new AutoLimelightPosition(driveSubsystem, limelight, false, false));
                NamedCommands.registerCommand("AutoYLimelight",
                                new AutoLimelight(driveSubsystem, limelight, true, true));
                NamedCommands.registerCommand("AutoBLimelight",
                                new AutoLimelight(driveSubsystem, limelight, false, false));
                NamedCommands.registerCommand("AutoYMoveForward",
                                new AutoMoveForwardForTime(driveSubsystem, limelight, true, 1));
                NamedCommands.registerCommand("AutoYMoveForwardFaster",
                                new AutoMoveForwardForTimeFaster(driveSubsystem, limelight, true, 1, 0.3));
                NamedCommands.registerCommand("MoveToYCoral1",
                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                Constants.Windmill.WindmillState.CoralDropOff1,
                                                true));
                NamedCommands.registerCommand("DropCoral",
                                new DropCoral(coralSubsystem, elevatorSubsystem, pivotSubsystem));
                NamedCommands.registerCommand("AutoBMoveForward",
                                new AutoMoveForwardForTime(driveSubsystem, limelight, false, 1));
                NamedCommands.registerCommand("WaitUntilSafeToMove",
                                new WaitUntilElevatorBelowSafeTravelHeight(elevatorSubsystem));
                NamedCommands.registerCommand("ZeroHeading",
                                new InstantCommand(() -> driveSubsystem.zeroHeading(), driveSubsystem));
                NamedCommands.registerCommand("AutoYMoveTowardSource",
                                new AutoPositionForTime(driveSubsystem, limelight, true, true, 0.5));
                NamedCommands.registerCommand("StartedMove", new PrintCommand("Startedmove"));
                NamedCommands.registerCommand("AutoYMoveRight", new NewAutoMoveOnReef(driveSubsystem, limelight, true));
                NamedCommands.registerCommand("AutoYMoveLeft", new NewAutoMoveOnReef(driveSubsystem, limelight, false));
                NamedCommands.registerCommand("AutoBMoveRight",
                                new NewAutoMoveOnReef(driveSubsystem, limelight, false));
                NamedCommands.registerCommand("AutoBMoveLeft", new NewAutoMoveOnReef(driveSubsystem, limelight, true));
                NamedCommands.registerCommand("IsElevatorInSafePosition", new AutoPrintElevatorSafe(elevatorSubsystem));
        }

        // The driver's controller
        CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
        CommandXboxController operatorController = new CommandXboxController(OIConstants.kOperaterControllerPort);
        GenericHID driverControllerRumble = driverController.getHID();
        GenericHID operatorControllerRumble = operatorController.getHID();

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

                // Register Named Commands for Pathplanner
                configureAutoCommands();

                // testing alternate drive
                // MoveCommand moveCommand = new MoveCommand(this.driveSubsystem,
                // driverController);
                // driveSubsystem.setDefaultCommand(moveCommand);

                // Configure default commands
                driveSubsystem.setDefaultCommand(
                                // The right trigger controls the speed of the robot.
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> driveSubsystem.drive(
                                                                MathUtil.applyDeadband(
                                                                                Math.pow(driverController
                                                                                                .getRightTriggerAxis(),
                                                                                                2),
                                                                                0),
                                                                -MathUtil.applyDeadband(driverController.getLeftY(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(driverController.getLeftX(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(
                                                                                Math.pow(driverController.getRightX(),
                                                                                                3),
                                                                                OIConstants.kTurnDeadband),
                                                                true),
                                                driveSubsystem));

                // Another option that allows you to specify the default auto by its name
                autoChooser = AutoBuilder.buildAutoChooser("Test Auto");

                SmartDashboard.putData("Auto Chooser", autoChooser);

                field = new Field2d();
                SmartDashboard.putData("Field", field);

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
                // final Trigger zeroElevator = operatorController.start();
                final Trigger zeroPivot = operatorController.start();

                // Zero elevator - carriage must be below stage 1 or it will zero where it is
                // zeroElevator.onTrue(new ZeroElevator(elevatorSubsystem));
                zeroPivot.onTrue(new InitializePivotAndElevator(pivotSubsystem, elevatorSubsystem));

                // Reset heading of robot for field relative drive
                final Trigger zeroHeadingButton = driverController.start();
                zeroHeadingButton.onTrue(new InstantCommand(() -> driveSubsystem.zeroHeading(), driveSubsystem));

                // // Limelight Buttons
                final Trigger limelightTrigger1 = driverController.x();
                final Trigger limelightTrigger2 = driverController.b();

                // limelightTrigger1.onTrue(
                // new ParallelRaceGroup(new WaitCommand(1), new
                // AlignToAprilTagBlue(driveSubsystem,
                // limelight)));

                // limelightTrigger2.onTrue(
                // new ParallelRaceGroup(new WaitCommand(4), new
                // AlignToAprilTagYellow(driveSubsystem,
                // limelight)));
                limelightTrigger1.onTrue(
                                new ParallelRaceGroup(new WaitCommand(4), new AlignAndDriveBlue(driveSubsystem,
                                                limelight)));

                limelightTrigger2.onTrue(
                                new ParallelRaceGroup(new WaitCommand(4), new AlignAndDriveYellow(driveSubsystem,
                                                limelight, elevatorSubsystem, pivotSubsystem)));

                // Driver operations
                final Trigger ejectCoral = driverController.a();
                final Trigger pickUpCoral = driverController.leftTrigger();
                final Trigger ejectAlgae = driverController.y();
                final Trigger shootAlgaeNetBlue = driverController.leftBumper();
                final Trigger shootAlgaeNetYellow = driverController.rightBumper();
                final Trigger raiseClimber = driverController.povRight();
                final Trigger lowerClimber = driverController.povLeft();
                final Trigger altButton = driverController.back();
                final Trigger left = driverController.povDown();
                final Trigger right = driverController.povUp();

                // commands that go with driver operations
                ejectCoral.onTrue(new EjectCoral(coralSubsystem, elevatorSubsystem,
                                pivotSubsystem));
                ejectCoral.and(altButton).onTrue(new DropCoral(coralSubsystem,
                                elevatorSubsystem, pivotSubsystem));
                pickUpCoral.onTrue(new PickUpCoralFromSource(coralSubsystem,
                                elevatorSubsystem, pivotSubsystem, false));
                pickUpCoral.and(altButton)
                                .onTrue(new PickUpCoralFromSource(coralSubsystem, elevatorSubsystem,
                                                pivotSubsystem,
                                                true));

                ejectAlgae.onTrue(new EjectAlgae(algaeSubsystem));
                shootAlgaeNetBlue.onTrue(new ScoreAlgaeNetBlue(algaeSubsystem,
                                elevatorSubsystem, pivotSubsystem,
                                coralSubsystem));
                shootAlgaeNetYellow.onTrue(new ScoreAlgaeNetYellow(algaeSubsystem,
                                elevatorSubsystem, pivotSubsystem,
                                coralSubsystem));

                left.onTrue(new AutoPositionLeftRight(driveSubsystem, limelight, false,
                                (limelight.hasTarget("limelight-yellow"))));
                right.onTrue(new AutoPositionLeftRight(driveSubsystem, limelight, true,
                                (limelight.hasTarget("limelight-yellow"))));
                raiseClimber.onTrue(new RunCommand(() -> climberSubsystem.moveUp(),
                                climberSubsystem))
                                .onFalse(new RunCommand(() -> climberSubsystem.stop(),
                                                climberSubsystem));
                lowerClimber.onTrue(new RunCommand(() -> climberSubsystem.moveDown(),
                                climberSubsystem))
                                .onFalse(new RunCommand(() -> climberSubsystem.stop(),
                                                climberSubsystem));

                // Operator Controls
                final Trigger manualCoral = operatorController.rightTrigger();
                final Trigger manualAlgae = operatorController.leftTrigger();

                manualCoral.onTrue(new SequentialCommandGroup(
                                new ToCoralSourceManual(elevatorSubsystem, pivotSubsystem, false),
                                new RunCommand(() -> coralSubsystem.pickup(), coralSubsystem)))
                                .onFalse(new SequentialCommandGroup(
                                                new InstantCommand(() -> coralSubsystem.stop(), coralSubsystem),
                                                new WaitCommand(0.25),
                                                new ToHomeCommand(elevatorSubsystem, pivotSubsystem, coralSubsystem)));

                manualAlgae.onTrue(new RunCommand(() -> algaeSubsystem.pickup(), algaeSubsystem))
                                .onFalse(new InstantCommand(algaeSubsystem::holdAlgae));

                // climber control
                final Trigger toggleClimber = operatorController.back();

                toggleClimber.toggleOnTrue(new ConditionalCommand(
                                new DeployClimb(climberSubsystem),
                                new RetractClimb(climberSubsystem),
                                climberSubsystem::climberAtHomePosition));

                toggleClimber.toggleOnTrue(new ConditionalCommand(
                                new LockPivotAndElevatorCommand(elevatorSubsystem,
                                                pivotSubsystem).withTimeout(15)
                                                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming),
                                new ToHomeCommand(elevatorSubsystem, pivotSubsystem, coralSubsystem),
                                climberSubsystem::climberAtHomePosition));

                // position controls
                final Trigger home = operatorController.leftBumper();
                final Trigger algaeGround = operatorController.povDown();
                final Trigger algaeReef2 = operatorController.povRight();
                final Trigger algaeReef3 = operatorController.povUp();
                final Trigger algaeProcessor = operatorController.povLeft();
                final Trigger coralDropOff4 = operatorController.y();
                final Trigger coralDropOff3 = operatorController.x();
                final Trigger coralDropOff2 = operatorController.b();
                final Trigger coralDropOff1 = operatorController.a();
                final Trigger altPositionRight = operatorController.rightBumper();

                // initializeInputs.onTrue(new ResetOperatorInputs());

                home.onTrue(new Windmill(elevatorSubsystem, pivotSubsystem, Constants.Windmill.WindmillState.Home,
                                false));

                algaeGround.onTrue(new PickUpAlgaeFromGround(algaeSubsystem, elevatorSubsystem, pivotSubsystem));

                algaeReef2.onTrue(new SequentialCommandGroup(new ToAlgaeL2(elevatorSubsystem, pivotSubsystem, false),
                                new PickUpAlgae(algaeSubsystem), new WaitCommand(.25),
                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                Constants.Windmill.WindmillState.Home,
                                                false)));
                algaeReef3.onTrue(new SequentialCommandGroup(new ToAlgaeL3(elevatorSubsystem, pivotSubsystem, false),
                                new PickUpAlgae(algaeSubsystem), new WaitCommand(.25),
                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                Constants.Windmill.WindmillState.AlgaeHoldHeight,
                                                false)));

                algaeProcessor.onTrue(new ToAlgaeGround(elevatorSubsystem, pivotSubsystem));

                coralDropOff1.onTrue(new ScoreCoralL1(elevatorSubsystem, pivotSubsystem, coralSubsystem, false));

                coralDropOff2.onTrue(new ScoreCoralL2(elevatorSubsystem, pivotSubsystem, coralSubsystem, false));

                coralDropOff3.onTrue(new ScoreCoralL3(elevatorSubsystem, pivotSubsystem, coralSubsystem, false));

                coralDropOff4.onTrue(new ScoreCoralL4(elevatorSubsystem, pivotSubsystem, coralSubsystem, false));

                algaeReef2.and(altPositionRight)
                                .onTrue(new SequentialCommandGroup(
                                                new ToAlgaeL2(elevatorSubsystem, pivotSubsystem, true),
                                                new PickUpAlgae(algaeSubsystem), new WaitCommand(.25),
                                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                                Constants.Windmill.WindmillState.Home,
                                                                true)));
                algaeReef3.and(altPositionRight).onTrue(
                                new SequentialCommandGroup(new ToAlgaeL3(elevatorSubsystem, pivotSubsystem, true),
                                                new PickUpAlgae(algaeSubsystem), new WaitCommand(.25),
                                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                                Constants.Windmill.WindmillState.AlgaeHoldHeight,
                                                                true)));

                coralDropOff1.and(altPositionRight)
                                .onTrue(new ScoreCoralL1(elevatorSubsystem, pivotSubsystem, coralSubsystem, true));
                coralDropOff2.and(altPositionRight)
                                .onTrue(new ScoreCoralL2(elevatorSubsystem, pivotSubsystem, coralSubsystem, true));
                coralDropOff3.and(altPositionRight)
                                .onTrue(new ScoreCoralL3(elevatorSubsystem, pivotSubsystem, coralSubsystem, true));
                coralDropOff4.and(altPositionRight)
                                .onTrue(new ScoreCoralL4(elevatorSubsystem, pivotSubsystem, coralSubsystem, true));

                // Set Left Joystick for manual elevator/pivot movement
                final Trigger raiseElevator = operatorController.axisLessThan(1, -0.25);
                final Trigger lowerElevator = operatorController.axisGreaterThan(1, 0.25);
                final Trigger pivotClockwise = operatorController.axisGreaterThan(4, 0.25);
                final Trigger pivotCounterClockwise = operatorController.axisLessThan(4,
                                -0.25);

                // commands that go with manual elevator/pivot movement
                pivotClockwise
                                .onTrue(new InstantCommand(pivotSubsystem::start, pivotSubsystem))
                                .onFalse(new InstantCommand(pivotSubsystem::holdPivotArmManual, pivotSubsystem));
                pivotCounterClockwise.onTrue(new InstantCommand(pivotSubsystem::reverse, pivotSubsystem))
                                .onFalse(new InstantCommand(pivotSubsystem::holdPivotArmManual, pivotSubsystem));
                raiseElevator.whileTrue(
                                new InstantCommand(elevatorSubsystem::moveUp, elevatorSubsystem)
                                                .andThen(new WaitUntilCommand(
                                                                () -> elevatorSubsystem.ElevatorRaiseLimitHit()))
                                                .andThen(new InstantCommand(elevatorSubsystem::holdHeight,
                                                                elevatorSubsystem)))
                                .onFalse(new InstantCommand(elevatorSubsystem::holdHeight, elevatorSubsystem));
                lowerElevator.whileTrue(
                                new InstantCommand(elevatorSubsystem::moveDown, elevatorSubsystem)
                                                .andThen(new WaitUntilCommand(
                                                                () -> elevatorSubsystem.ElevatorLowerLimitHit()))
                                                .andThen(new InstantCommand(elevatorSubsystem::holdHeight,
                                                                elevatorSubsystem)))
                                .onFalse(new InstantCommand(elevatorSubsystem::holdHeight,
                                                elevatorSubsystem));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        // // Create config for trajectory
        // TrajectoryConfig config = new TrajectoryConfig(
        // AutoConstants.kMaxSpeedMetersPerSecond,
        // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // // Add kinematics to ensure max speed is actually obeyed
        // .setKinematics(DriveConstants.kDriveKinematics);

        // // An example trajectory to follow. All units in meters.
        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // // Start at the origin facing the +X direction
        // new Pose2d(0, 0, new Rotation2d(0)),
        // // Pass through these two interior waypoints, making an 's' curve path
        // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // // End 3 meters straight ahead of where we started, facing forward
        // new Pose2d(3, 0, new Rotation2d(0)),
        // config);

        // var thetaController = new ProfiledPIDController(
        // AutoConstants.kPThetaController, 0, 0,
        // AutoConstants.kThetaControllerConstraints);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // SwerveControllerCommand swerveControllerCommand = new
        // SwerveControllerCommand(
        // exampleTrajectory,
        // driveSubsystem::getPose, // Functional interface to feed supplier
        // DriveConstants.kDriveKinematics,

        // // Position controllers
        // new PIDController(AutoConstants.kPXController, 0, 0),
        // new PIDController(AutoConstants.kPYController, 0, 0),
        // thetaController,
        // driveSubsystem::setModuleStates,
        // driveSubsystem);

        // // Run path following command, then stop at the end.
        // return swerveControllerCommand.andThen(() -> driveSubsystem.drive(0, 0, 0, 0,
        // false));
        // }

        public Command rumbleDriverCommand() {
                return new RunCommand(() -> rumbleDriverCtrl()).withTimeout(2).finallyDo(() -> stopRumbleDriverCtrl());
        }

        public void rumbleDriverCtrl() {
                driverControllerRumble.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
                operatorControllerRumble.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
        }

        public void stopRumbleDriverCtrl() {
                driverControllerRumble.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
                operatorControllerRumble.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
        }

}
