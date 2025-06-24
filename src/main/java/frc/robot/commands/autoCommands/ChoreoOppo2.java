package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.util.function.FloatSupplier;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Windmill.WindmillState;
import frc.robot.commands.drivetrain.SwerveDriveChoreoFollow;
import frc.robot.commands.intake.PickupCoral;
import frc.robot.commands.states.ScoreReef;
import frc.robot.commands.states.TestHandoff;
import frc.robot.commands.states.TestHandoffAuto;
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

public class ChoreoOppo2 extends SequentialCommandGroup {
        public ChoreoOppo2(DriveSubsystem driveSubsystem, PivotSubsystem pivotSubsystem,
                        ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem,
                        IntakeSubsystem intakeSubsystem) {

                Optional<Trajectory<SwerveSample>> path0_5 = Choreo.loadTrajectory("OppoBargeToReef221");
                Optional<Trajectory<SwerveSample>> path1 = Choreo.loadTrajectory("OppoBargeToReef222");
                Optional<Trajectory<SwerveSample>> path2 = Choreo.loadTrajectory("Reef22ToSource");
                Optional<Trajectory<SwerveSample>> path3 = Choreo.loadTrajectory("SourceToReef17L");
                Optional<Trajectory<SwerveSample>> path4 = Choreo.loadTrajectory("Reef17toBack");
                // Optional<Trajectory<SwerveSample>> path4 =
                // Choreo.loadTrajectory("Reef20ToScoreBarge");
                // Optional<Trajectory<SwerveSample>> path5 =
                // Choreo.loadTrajectory("ScoreBargeToSource");

                addCommands(
                                new SequentialCommandGroup(
                                                new InstantCommand(endEffectorSubsystem::stop),
                                                new ParallelCommandGroup(
                                                                new SwerveDriveChoreoFollow(driveSubsystem, path0_5,
                                                                                true),
                                                                new InstantCommand(intakeSubsystem::zeroIntake)),
                                                new ZeroElevator(elevatorSubsystem),
                                                new ParallelCommandGroup(
                                                                new SequentialCommandGroup(new WaitCommand(0.5),
                                                                                new SwerveDriveChoreoFollow(
                                                                                                driveSubsystem, path1,
                                                                                                false)),
                                                                new ToCoralDropOff(elevatorSubsystem, pivotSubsystem,
                                                                                intakeSubsystem,
                                                                                endEffectorSubsystem,
                                                                                WindmillState.CoralDropOff4, true)),
                                                new WaitCommand(.5),
                                                new ScoreReef(elevatorSubsystem, pivotSubsystem, endEffectorSubsystem,
                                                                intakeSubsystem),
                                                new WaitCommand(.5),
                                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                                WindmillState.CoralScoreHome, true),
                                                // added this to speed up and get the arm positioned properly
                                                // while robot is moving to pickup coral,
                                                // it should also have the arm going to home position so it's ready to
                                                // handoff the coral,
                                                // this should make the handoff cleaner
                                                new ParallelCommandGroup(
                                                                new SequentialCommandGroup(
                                                                                new WaitCommand(2),
                                                                                new Windmill(elevatorSubsystem,
                                                                                                pivotSubsystem,
                                                                                                WindmillState.Home,
                                                                                                false)),
                                                                new ParallelRaceGroup(
                                                                                new PickupCoral(intakeSubsystem),
                                                                                new SwerveDriveChoreoFollow(
                                                                                                driveSubsystem, path2,
                                                                                                false))),
                                                new ParallelCommandGroup(
                                                                new SwerveDriveChoreoFollow(driveSubsystem, path3,
                                                                                false),
                                                                new SequentialCommandGroup(
                                                                                new ConditionalCommand(
                                                                                                new InstantCommand(),
                                                                                                new PickupCoral(intakeSubsystem),
                                                                                                intakeSubsystem::hasCoralCentered),
                                                                                // I don't think the waituntil command
                                                                                // is necessary here, because i don't
                                                                                // think pickup coral will end until the
                                                                                // coral is centered
                                                                                // new WaitUntilCommand(
                                                                                // intakeSubsystem::hasCoralCentered),
                                                                                new WaitCommand(1.25)),
                                                                // new TestHandoff(elevatorSubsystem,
                                                                // pivotSubsystem,
                                                                // intakeSubsystem,
                                                                // endEffectorSubsystem))
                                                                // )),
                                                                // new ToCoralDropOff(
                                                                // elevatorSubsystem,
                                                                // pivotSubsystem,
                                                                // intakeSubsystem,
                                                                // endEffectorSubsystem,
                                                                // WindmillState.CoralDropOff4,
                                                                // true),
                                                                // new WaitCommand(2.5), // 2 seconds wait to get to
                                                                // position, 0.5 seconds
                                                                // // to wait for wobble
                                                                // new ScoreReef(elevatorSubsystem,
                                                                // pivotSubsystem,
                                                                // endEffectorSubsystem,
                                                                // intakeSubsystem),
                                                                // new WaitCommand(.5),
                                                                new SwerveDriveChoreoFollow(
                                                                                driveSubsystem, path4,
                                                                                false))));
                // ))));
        }
}
