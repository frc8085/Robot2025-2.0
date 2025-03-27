package frc.robot.commands.autoCommands;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.SwerveDriveChoreoFollow;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ChoreoAutoOppoToL4 extends SequentialCommandGroup {
        public ChoreoAutoOppoToL4(DriveSubsystem driveSubsystem, AlgaeSubsystem algaeSubsystem,
                        ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem,
                        CoralSubsystem coralSubsystem) {

                Optional<Trajectory<SwerveSample>> path1 = Choreo.loadTrajectory("OppoBargeToReef22");
                Optional<Trajectory<SwerveSample>> path2 = Choreo.loadTrajectory("Reef22ToSource");
                Optional<Trajectory<SwerveSample>> path3 = Choreo.loadTrajectory("SourceToReef17L");
                Optional<Trajectory<SwerveSample>> path4 = Choreo.loadTrajectory("Reef17LToSource");
                Optional<Trajectory<SwerveSample>> path5 = Choreo.loadTrajectory("SourceToReef17R");

                addCommands(
                                new SwerveDriveChoreoFollow(
                                                driveSubsystem,
                                                path1, true),
                                new AutoScoreCoralL4(algaeSubsystem, elevatorSubsystem, pivotSubsystem, coralSubsystem,
                                                driveSubsystem,
                                                isScheduled()),
                                new AutoToHomeCommand(elevatorSubsystem, pivotSubsystem),
                                new AutoWaitUntilElevatorBelowSafeTravelHeight(elevatorSubsystem),
                                new ParallelDeadlineGroup(
                                                new AutoCoralPickup(elevatorSubsystem, pivotSubsystem, coralSubsystem),
                                                new SwerveDriveChoreoFollow(
                                                                driveSubsystem,
                                                                path2, false)),
                                new SwerveDriveChoreoFollow(
                                                driveSubsystem,
                                                path3, false),
                                new AutoScoreCoralL4(algaeSubsystem, elevatorSubsystem, pivotSubsystem, coralSubsystem,
                                                driveSubsystem,
                                                isScheduled()),
                                new SwerveDriveChoreoFollow(
                                                driveSubsystem,
                                                path4, false),
                                new SwerveDriveChoreoFollow(
                                                driveSubsystem,
                                                path5, false),
                                new AutoScoreCoralL4(algaeSubsystem, elevatorSubsystem, pivotSubsystem, coralSubsystem,
                                                driveSubsystem,
                                                isScheduled()));
        }
}