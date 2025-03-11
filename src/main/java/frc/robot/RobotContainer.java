// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.manipulator.coral.*;
import frc.robot.commands.windmill.Windmill;
import frc.robot.commands.autoCommands.*;
import frc.robot.commands.movement.AutoMoveForwardForTime;
import frc.robot.commands.movement.AutoMoveForwardForTimeFaster;
import frc.robot.commands.movement.NewAutoMoveOnReef;
import frc.robot.commands.windmill.InitializePivotAndElevator;
import frc.robot.commands.drivetrain.SwerveDriveTeleop;
import frc.robot.io.IO;
import frc.robot.subsystems.*;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems

        public static final PivotSubsystem pivot = new PivotSubsystem();
        public static final DriveSubsystem drivetrain = new DriveSubsystem();
        public static final CoralSubsystem coral = new CoralSubsystem();
        public static final ElevatorSubsystem elevator = new ElevatorSubsystem();
        public static final ClimberSubsystem climber = new ClimberSubsystem();
        public static final AlgaeSubsystem algae = new AlgaeSubsystem();
        public static final LimelightSubsystem limelight = new LimelightSubsystem();

        private final SendableChooser<Command> autoChooser;
        protected SendableChooser<Alliance> allianceColor = new SendableChooser<>();

        private final Field2d field;

        // Register Named Commands for PathPlanner
        private void configureAutoCommands() {
                NamedCommands.registerCommand("InitializePE",
                                new InitializePivotAndElevator(this.pivot, this.elevator));
                NamedCommands.registerCommand("AutoYCoral1",
                                new AutoCoral1(this.coral, this.elevator, this.pivot,
                                                this.drivetrain,
                                                true));
                NamedCommands.registerCommand("AutoYCoral2",
                                new AutoCoral2(this.coral, this.elevator, this.pivot,
                                                this.drivetrain,
                                                true));
                NamedCommands.registerCommand("AutoYCoral3",
                                new AutoCoral3(this.coral, this.elevator, this.pivot,
                                                this.drivetrain,
                                                true));
                NamedCommands.registerCommand("AutoYCoral4",
                                new AutoCoral4(this.coral, this.elevator, this.pivot,
                                                this.drivetrain,
                                                true));
                NamedCommands.registerCommand("AutoBCoral4",
                                new AutoCoral4(this.coral, this.elevator, this.pivot,
                                                this.drivetrain,
                                                false));
                NamedCommands.registerCommand("AutoCoralSource",
                                new AutoCoralSource(this.coral, this.elevator,
                                                this.pivot, false));
                NamedCommands.registerCommand("AutoYLimelightRight",
                                new AutoLimelightPosition(this.drivetrain, this.limelight, true,
                                                true));
                NamedCommands.registerCommand("AutoYLimelightLeft",
                                new AutoLimelightPosition(this.drivetrain, this.limelight, false,
                                                true));
                NamedCommands.registerCommand("AutoBLimelightRight",
                                new AutoLimelightPosition(this.drivetrain, this.limelight, true,
                                                false));
                NamedCommands.registerCommand("AutoBLimelightLeft",
                                new AutoLimelightPosition(this.drivetrain, this.limelight, false,
                                                false));
                NamedCommands.registerCommand("AutoYLimelight",
                                new AutoLimelight(this.drivetrain, this.limelight, true, true));
                NamedCommands.registerCommand("AutoBLimelight",
                                new AutoLimelight(this.drivetrain, this.limelight, false, false));
                NamedCommands.registerCommand("AutoYMoveForward",
                                new AutoMoveForwardForTime(this.drivetrain, this.limelight, true,
                                                1));
                NamedCommands.registerCommand("AutoYMoveForwardFaster",
                                new AutoMoveForwardForTimeFaster(this.drivetrain, this.limelight,
                                                true, 1, 0.3));
                NamedCommands.registerCommand("MoveToYCoral1",
                                new Windmill(this.elevator, this.pivot,
                                                Constants.Windmill.WindmillState.CoralDropOff1,
                                                true));
                NamedCommands.registerCommand("DropCoral",
                                new DropCoral(this.coral, this.elevator, this.pivot,
                                                this.drivetrain));
                NamedCommands.registerCommand("AutoBMoveForward",
                                new AutoMoveForwardForTime(this.drivetrain, this.limelight, false,
                                                1));
                NamedCommands.registerCommand("WaitUntilSafeToMove",
                                new WaitUntilElevatorBelowSafeTravelHeight(this.elevator));
                NamedCommands.registerCommand("ZeroHeading",
                                new InstantCommand(() -> this.drivetrain.zeroHeading(),
                                                this.drivetrain));
                NamedCommands.registerCommand("AutoYMoveTowardSource",
                                new AutoPositionForTime(this.drivetrain, this.limelight, true,
                                                true, 0.5));
                NamedCommands.registerCommand("StartedMove", new PrintCommand("Startedmove"));
                NamedCommands.registerCommand("AutoYMoveRight",
                                new NewAutoMoveOnReef(this.drivetrain, this.limelight, true));
                NamedCommands.registerCommand("AutoYMoveLeft",
                                new NewAutoMoveOnReef(this.drivetrain, this.limelight, false));
                NamedCommands.registerCommand("AutoBMoveRight",
                                new NewAutoMoveOnReef(this.drivetrain, this.limelight, false));
                NamedCommands.registerCommand("AutoBMoveLeft",
                                new NewAutoMoveOnReef(this.drivetrain, this.limelight, true));
                NamedCommands.registerCommand("IsElevatorInSafePosition",
                                new AutoPrintElevatorSafe(this.elevator));
                NamedCommands.registerCommand("AutoAlgaeL2",
                                new AutoAlgaeL2(this.drivetrain, this.coral,
                                                this.algae, this.elevator,
                                                this.pivot));
        }

        // The driver's controller
        // CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
        // CommandXboxController operatorController = new CommandXboxController(OIConstants.kOperaterControllerPort);
        // GenericHID driverControllerRumble = driverController.getHID();
        // GenericHID operatorControllerRumble = operatorController.getHID();

        // public void adjustJoystickValues() {
        //         double rawX = driverController.getLeftX();
        //         double rawY = driverController.getLeftY();

        // }

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();

                // Register Named Commands for Pathplanner
                configureAutoCommands();

                // testing alternate drive
                // MoveCommand moveCommand = new MoveCommand(this.this.drivetrain,
                // driverController);
                // this.drivetrain.setDefaultCommand(moveCommand);

                // Configure default commands
                this.drivetrain.setDefaultCommand(
                        // IMPLEMENT DEFAULT COMMAND
                        new SwerveDriveTeleop(this.drivetrain)
                        );
                                // The right trigger controls the speed of the robot.
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                
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

                IO io = new IO();

                io.init();

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
        // this.drivetrain::getPose, // Functional interface to feed supplier
        // DriveConstants.kDriveKinematics,

        // // Position controllers
        // new PIDController(AutoConstants.kPXController, 0, 0),
        // new PIDController(AutoConstants.kPYController, 0, 0),
        // thetaController,
        // this.drivetrain::setModuleStates,
        // this.drivetrain);

        // // Run path following command, then stop at the end.
        // return swerveControllerCommand.andThen(() ->
        // this.drivetrain.drive(0, 0, 0, 0,
        // false));
        // }

        // public Command rumbleDriverCommand() {
        //         return new RunCommand(() -> rumbleDriverCtrl()).withTimeout(2).finallyDo(() -> stopRumbleDriverCtrl());
        // }

        // public void rumbleDriverCtrl() {
        //         driverControllerRumble.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
        //         operatorControllerRumble.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
        // }

        // public void stopRumbleDriverCtrl() {
        //         driverControllerRumble.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
        //         operatorControllerRumble.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
        // }

}
