package frc.robot.commands.autoCommands;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.SwerveDriveChoreoFollow;
import frc.robot.subsystems.DriveSubsystem;

public class AutoOppoToL4 extends SequentialCommandGroup {
    public AutoOppoToL4(DriveSubsystem driveSubsystem) {

        addCommands(
                new SwerveDriveChoreoFollow(
                        driveSubsystem,
                        Choreo.<SwerveSample>loadTrajectory("OppoBargeToReef22"), true),
                new SwerveDriveChoreoFollow(
                        driveSubsystem,
                        Choreo.<SwerveSample>loadTrajectory("Reef22ToSource"), false),
                new SwerveDriveChoreoFollow(
                        driveSubsystem,
                        Choreo.<SwerveSample>loadTrajectory("SourceToReef17L"), false),
                new SwerveDriveChoreoFollow(
                        driveSubsystem,
                        Choreo.<SwerveSample>loadTrajectory("Reef17LToSource"), false),
                new SwerveDriveChoreoFollow(
                        driveSubsystem,
                        Choreo.<SwerveSample>loadTrajectory("SourceToReef17R"), false));
    }
}