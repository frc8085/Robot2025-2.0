package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Windmill.WindmillState;
import frc.robot.commands.drivetrain.SwerveDriveChoreoFollow;
import frc.robot.commands.intake.DumpCoral;
import frc.robot.commands.intake.PickupCoral;
import frc.robot.commands.states.ScoreReef;
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

public class ChoreoL1 extends SequentialCommandGroup {
        public ChoreoL1(DriveSubsystem driveSubsystem, PivotSubsystem pivotSubsystem,
                        ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem,
                        IntakeSubsystem intakeSubsystem) {

                Optional<Trajectory<SwerveSample>> path1 = Choreo.loadTrajectory("CenterL1");
                addCommands(
                                new SequentialCommandGroup(

                                                new InstantCommand(intakeSubsystem::zeroIntake),

                                                new SwerveDriveChoreoFollow(driveSubsystem, path1,
                                                                true),
                                                new DumpCoral(intakeSubsystem),
                                                new ZeroElevator(elevatorSubsystem)
                                // new ParallelRaceGroup(
                                // new PickupCoral(intakeSubsystem),
                                // new SwerveDriveChoreoFollow(driveSubsystem, path2,
                                // false)),
                                // new SwerveDriveChoreoFollow(driveSubsystem, path3, false)));
                                ));

        }
}
