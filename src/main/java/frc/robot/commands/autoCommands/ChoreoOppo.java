package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Windmill.WindmillState;
import frc.robot.commands.drivetrain.SwerveDriveChoreoFollow;
import frc.robot.commands.intake.PickupCoral;
import frc.robot.commands.states.ScoreReef;
import frc.robot.commands.states.TestHandoff;
import frc.robot.commands.states.ToCoralDropOff;
import frc.robot.commands.windmill.Windmill;
import frc.robot.commands.windmill.elevator.ZeroElevator;
import frc.robot.subsystems.Algae.AlgaeSubsystem;
import frc.robot.subsystems.Coral.CoralSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector.EndEffectorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import java.util.Optional;

public class ChoreoOppo extends SequentialCommandGroup {
        public ChoreoOppo(DriveSubsystem driveSubsystem, PivotSubsystem pivotSubsystem,
                        ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem,
                        IntakeSubsystem intakeSubsystem) {

                Optional<Trajectory<SwerveSample>> path1 = Choreo.loadTrajectory("OppoBargeToReef22");
                Optional<Trajectory<SwerveSample>> path2 = Choreo.loadTrajectory("Reef22ToSource");
                Optional<Trajectory<SwerveSample>> path3 = Choreo.loadTrajectory("SourceToReef17L");
                // Optional<Trajectory<SwerveSample>> path4 =
                // Choreo.loadTrajectory("Reef20ToScoreBarge");
                // Optional<Trajectory<SwerveSample>> path5 =
                // Choreo.loadTrajectory("ScoreBargeToSource");

                addCommands(
                                new SequentialCommandGroup(
                                                new ParallelCommandGroup(new ZeroElevator(elevatorSubsystem),
                                                                new InstantCommand(intakeSubsystem::zeroIntake)),
                                                new ParallelCommandGroup(
                                                                new SwerveDriveChoreoFollow(driveSubsystem, path1,
                                                                                true),
                                                                new ToCoralDropOff(elevatorSubsystem, pivotSubsystem,
                                                                                intakeSubsystem,
                                                                                endEffectorSubsystem,
                                                                                WindmillState.CoralDropOff4, true)),
                                                new ScoreReef(elevatorSubsystem, pivotSubsystem, endEffectorSubsystem,
                                                                intakeSubsystem),
                                                new WaitCommand(.5),
                                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                                WindmillState.CoralScoreHome, true),
                                                new ParallelRaceGroup(
                                                                new PickupCoral(intakeSubsystem),
                                                                new SwerveDriveChoreoFollow(driveSubsystem, path2,
                                                                                false)),
                                                new ParallelCommandGroup(
                                                                new SwerveDriveChoreoFollow(driveSubsystem, path3,
                                                                                false),
                                                                new SequentialCommandGroup(
                                                                                new WaitUntilCommand(
                                                                                                intakeSubsystem::hasCoralCentered),
                                                                                new TestHandoff(elevatorSubsystem,
                                                                                                pivotSubsystem,
                                                                                                intakeSubsystem,
                                                                                                endEffectorSubsystem),
                                                                                new ToCoralDropOff(
                                                                                                elevatorSubsystem,
                                                                                                pivotSubsystem,
                                                                                                intakeSubsystem,
                                                                                                endEffectorSubsystem,
                                                                                                WindmillState.CoralDropOff4,
                                                                                                true)))));
                // ));

        }
}
