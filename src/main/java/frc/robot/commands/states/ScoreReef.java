package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector.EndEffectorSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.commands.windmill.Windmill;
import frc.robot.Constants.TuningModeConstants;
import frc.robot.Constants.Windmill.WindmillState;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.commands.endEffector.RunEndEffector;

public class ScoreReef extends Command {

    public static enum ReefLevel {
        None,
        One,
        Two,
        Three,
        Four,
    }

    private ElevatorSubsystem elevatorSubsystem;
    private PivotSubsystem pivotSubsystem;
    private EndEffectorSubsystem endEffectorSubsystem;

    public ScoreReef(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem,
            EndEffectorSubsystem endEffectorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;
    }

    @Override
    public void initialize() {
        // get current hight and pivot angle

        ParallelCommandGroup commands = new ParallelCommandGroup();

        SequentialCommandGroup eject = new SequentialCommandGroup(
                new InstantCommand(() -> this.endEffectorSubsystem.drop()),
                new WaitCommand(0.5),
                new InstantCommand(() -> this.endEffectorSubsystem.stop()));

        ReefLevel reefLevel = elevatorSubsystem.getReefLevel();

        boolean mirrored = pivotSubsystem.reefMirrored();

        if (reefLevel == ReefLevel.None) {
            commands.addCommands(
                    eject);
        } else if (reefLevel == ReefLevel.Two) {
            commands.addCommands(
                    eject,
                    new Windmill(this.elevatorSubsystem, this.pivotSubsystem, WindmillState.CoralScore2, mirrored));
        } else if (reefLevel == ReefLevel.Three) {
            commands.addCommands(
                    eject,
                    new Windmill(this.elevatorSubsystem, this.pivotSubsystem, WindmillState.CoralScore3, mirrored));
        } else if (reefLevel == ReefLevel.Four) {
            commands.addCommands(
                    eject,
                    new Windmill(this.elevatorSubsystem, this.pivotSubsystem, WindmillState.CoralScore4, mirrored));
        }

        commands.schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}