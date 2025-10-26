// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.autoCommands.*;
// import frc.robot.commands.windmill.InitializePivotAndElevator;
import frc.robot.commands.drivetrain.SwerveDriveTeleop;
// import frc.robot.commands.scoring.ScoreCoralL4WithWheelLock;
import frc.robot.io.IO;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector.EndEffectorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Limelight.LimelightSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems

        public final PivotSubsystem pivot = new PivotSubsystem();
        public final DriveSubsystem drivetrain = new DriveSubsystem();
        public final EndEffectorSubsystem endEffector = new EndEffectorSubsystem();
        public final ElevatorSubsystem elevator = new ElevatorSubsystem();
        public final ClimberSubsystem climber = new ClimberSubsystem();
        // public final AlgaeSubsystem algae = new AlgaeSubsystem();
        public final LimelightSubsystem limelight = new LimelightSubsystem();
        public final IntakeSubsystem intake = new IntakeSubsystem();

        private final SendableChooser<Command> autoChooser;
        protected SendableChooser<Alliance> allianceColor = new SendableChooser<>();

        private final Field2d field;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();

                // Configure default commands
                this.drivetrain.setDefaultCommand(
                                // IMPLEMENT DEFAULT COMMAND
                                new SwerveDriveTeleop(this.drivetrain));

                // Another option that allows you to specify the default auto by its name
                autoChooser = AutoBuilder.buildAutoChooser("CenterBarge");
                autoChooser.addOption("CenterBarge",
                                new ChoreoCenterL4(this.drivetrain, this.pivot, this.elevator,
                                                this.endEffector,
                                                this.intake));
                autoChooser.addOption("CenterL1", new ChoreoCenterL1(this.drivetrain, this.pivot, this.elevator,
                                this.endEffector, this.intake));
                autoChooser.addOption("OppoBarge", new ChoreoOppoL4(this.drivetrain, this.pivot, this.elevator,
                                this.endEffector, this.intake));
                autoChooser.addOption("OppoBargeL1Slow", new ChoreoOppoL1Slow(this.drivetrain,
                                this.pivot, this.elevator,
                                this.endEffector, this.intake));
                autoChooser.addOption("OppoBargeL1Fast",
                                new ChoreoOppoL1Fast(this.drivetrain, this.pivot, this.elevator,
                                                this.endEffector, this.intake));

                SmartDashboard.putData("Auto Chooser", autoChooser);

                field = new Field2d();
                SmartDashboard.putData("Field", field);

                DataLogManager.start();
                DataLogManager.logNetworkTables(true);
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

                io.init(this);

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        // public Command rumbleDriverCommand() {
        // return new RunCommand(() -> rumbleDriverCtrl()).withTimeout(2).finallyDo(()
        // -> stopRumbleDriverCtrl());
        // }

        // public void rumbleDriverCtrl() {
        // driverControllerRumble.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
        // operatorControllerRumble.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
        // }

        // public void stopRumbleDriverCtrl() {
        // driverControllerRumble.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
        // operatorControllerRumble.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
        // }
}