package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.drivetrain.SwerveDriveChoreoFollow;
import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import java.util.Optional;

public class ChoreoAutoSimple extends SequentialCommandGroup {
    public ChoreoAutoSimple(DriveSubsystem driveSubsystem) {

        Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("SIMPLEAUTO1");

        addCommands(
                new SwerveDriveChoreoFollow(
                        driveSubsystem,
                        trajectory, true));
    }
}