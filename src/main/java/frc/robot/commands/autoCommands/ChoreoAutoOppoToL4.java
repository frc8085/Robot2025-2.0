package frc.robot.commands.autoCommands;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
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

                addCommands(
                                new SwerveDriveChoreoFollow(
                                                driveSubsystem,
                                                Choreo.<SwerveSample>loadTrajectory("OppoBargeToReef22"), true),
                                new AutoScoreCoralL4(algaeSubsystem, elevatorSubsystem, pivotSubsystem, coralSubsystem,
                                                driveSubsystem,
                                                isScheduled()),
                                new AutoToHomeCommand(elevatorSubsystem, pivotSubsystem),
                                new AutoWaitUntilElevatorBelowSafeTravelHeight(elevatorSubsystem),
                                new ParallelDeadlineGroup(
                                                new AutoCoralPickup(elevatorSubsystem, pivotSubsystem, coralSubsystem),
                                                new SwerveDriveChoreoFollow(
                                                                driveSubsystem,
                                                                Choreo.<SwerveSample>loadTrajectory("Reef22ToSource"),
                                                                false)),
                                new SwerveDriveChoreoFollow(
                                                driveSubsystem,
                                                Choreo.<SwerveSample>loadTrajectory("SourceToReef17L"), false),
                                new AutoScoreCoralL4(algaeSubsystem, elevatorSubsystem, pivotSubsystem, coralSubsystem,
                                                driveSubsystem,
                                                isScheduled()),
                                new SwerveDriveChoreoFollow(
                                                driveSubsystem,
                                                Choreo.<SwerveSample>loadTrajectory("Reef17LToSource"), false),
                                new SwerveDriveChoreoFollow(
                                                driveSubsystem,
                                                Choreo.<SwerveSample>loadTrajectory("SourceToReef17R"), false),
                                new AutoScoreCoralL4(algaeSubsystem, elevatorSubsystem, pivotSubsystem, coralSubsystem,
                                                driveSubsystem,
                                                isScheduled()));
        }
}