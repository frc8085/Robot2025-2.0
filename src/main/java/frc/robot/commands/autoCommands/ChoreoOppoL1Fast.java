package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Windmill.WindmillState;
import frc.robot.commands.drivetrain.SwerveDriveChoreoFollow;
import frc.robot.commands.intake.DumpCoral;
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

import java.util.EventListener;
import java.util.Optional;
import java.util.Map;

public class ChoreoOppoL1Fast extends SequentialCommandGroup {
        public ChoreoOppoL1Fast(DriveSubsystem driveSubsystem, PivotSubsystem pivotSubsystem,
                        ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem,
                        IntakeSubsystem intakeSubsystem) {

                // pivot arm up
                // 1) Start at barge
                // 2) Drive to L1
                // 3) Drop off Coral at L1
                // 4) Drive to new coral
                // Drop intake
                // drive into coral
                // 6) drive back to L1
                // 7) drop off coral at L1
                // 8) repeat steps 4-7

                Optional<Trajectory<SwerveSample>> path1 = Choreo.loadTrajectory("1All1");
                Optional<Trajectory<SwerveSample>> path2 = Choreo.loadTrajectory("1All2-3");

                addCommands(
                                new SequentialCommandGroup(
                                                new InstantCommand(intakeSubsystem::zeroIntake),

                                                new SwerveDriveChoreoFollow(driveSubsystem, path1, true),
                                                new DumpCoral(intakeSubsystem),
                                                new ParallelCommandGroup(
                                                                new SwerveDriveChoreoFollow(driveSubsystem, path2,
                                                                                false),

                                                                new SequentialCommandGroup(
                                                                                new WaitCommand(1),
                                                                                new ParallelDeadlineGroup(
                                                                                                new WaitCommand(2),
                                                                                                new PickupCoral(intakeSubsystem)),
                                                                                new WaitCommand(.75),
                                                                                new PrintCommand("DumpCoral"),
                                                                                new DumpCoral(intakeSubsystem),
                                                                                new WaitCommand(0.5),
                                                                                new PrintCommand("start pickup coral"),
                                                                                new ParallelDeadlineGroup(
                                                                                                new WaitCommand(2),
                                                                                                new PrintCommand(
                                                                                                                "PickupCoral"),
                                                                                                new PickupCoral(intakeSubsystem)))),
                                                new ParallelCommandGroup(
                                                                new DumpCoral(intakeSubsystem),
                                                                new ZeroElevator(elevatorSubsystem))));

        }
}
