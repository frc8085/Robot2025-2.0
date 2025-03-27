// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.autoCommands.ChoreoAutoCenterBarge;
import frc.robot.commands.drivetrain.SwerveDriveTeleop;
import frc.robot.io.IO;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
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

        public final PivotSubsystem pivot = new PivotSubsystem();
        public final DriveSubsystem drivetrain = new DriveSubsystem();
        public final CoralSubsystem coral = new CoralSubsystem();
        public final ElevatorSubsystem elevator = new ElevatorSubsystem();
        public final ClimberSubsystem climber = new ClimberSubsystem();
        public final AlgaeSubsystem algae = new AlgaeSubsystem();
        public final LimelightSubsystem limelight = new LimelightSubsystem();

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

                // Preload trajectories for Choreo

                // private Set<String> listFilesUsingFilesList(String dir) throws IOException {
                // try (Stream<Path> stream = Files.list(Paths.get(dir))) {
                // return stream
                // .filter(file -> !Files.isDirectory(file))
                // .map(Path::getFileName)
                // .map(Path::toString)
                // .collect(Collectors.toSet());
                // }
                // }
                // private Map<String, Optional<Trajectory<SwerveSample>>> loadTrajectories() {
                // Set<String> trajNames;
                // try {
                // if (Robot.isReal()) {
                // trajNames = listFilesUsingFilesList("/home/lvuser/deploy/choreo");
                // } else {
                // trajNames = listFilesUsingFilesList("src/main/deploy/choreo");
                // }
                // } catch (IOException e) {
                // DriverStation.reportError("Invalid Directory! Trajectories failed to load!",
                // true);
                // return null;
                // }
                // return trajNames.stream().collect(Collectors.toMap(
                // entry -> entry.replace(".traj", ""),
                // entry -> Choreo.loadTrajectory(entry.replace(".traj", ""))));
                // }
                // trajMap = loadTrajectories();

                // Another option that allows you to specify the default auto by its name
                autoChooser = AutoBuilder.buildAutoChooser("Test Auto");
                autoChooser.addOption("CenterBarge", new ChoreoAutoCenterBarge(this.drivetrain));

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
