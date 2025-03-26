package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.drivetrain.SwerveDriveChoreoFollow;
import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import java.util.Optional;

public class ChoreoAutoCenterBarge extends SequentialCommandGroup {
    public ChoreoAutoCenterBarge(DriveSubsystem driveSubsystem) {

        Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("CenterBargeToReef21");

        addCommands(
                new SwerveDriveChoreoFollow(
                        driveSubsystem,
                        trajectory, true));
    }
}