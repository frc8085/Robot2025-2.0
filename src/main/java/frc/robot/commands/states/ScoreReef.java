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
import frc.robot.subsystems.Intake.IntakeSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.commands.windmill.Windmill;
import frc.robot.commands.windmill.WindmillSlow;
import frc.robot.Constants.TuningModeConstants;
import frc.robot.Constants.Windmill.WindmillState;
import frc.robot.subsystems.Intake.IntakeConstants;
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
    private IntakeSubsystem intakeSubsystem;

    public ScoreReef(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem,
            EndEffectorSubsystem endEffectorSubsystem, IntakeSubsystem intakeSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        // get current height and pivot angle

        ParallelCommandGroup commands = new ParallelCommandGroup();

        SequentialCommandGroup eject = new SequentialCommandGroup(
                new WaitCommand(.1),
                new InstantCommand(() -> this.endEffectorSubsystem.drop())
        // new WaitCommand(0.5)
        // new InstantCommand(() -> this.endEffectorSubsystem.stop())
        );

        ReefLevel reefLevel = elevatorSubsystem.getReefLevel();

        boolean mirrored = pivotSubsystem.reefMirrored();

        if (mirrored) {
            if (reefLevel == ReefLevel.None) {
                commands.addCommands(
                        eject);
            } else if (reefLevel == ReefLevel.Two) {
                commands.addCommands(
                        new InstantCommand(
                                () -> this.intakeSubsystem.setDeployRotation(IntakeConstants.kIntakeScoreAngle)),
                        new Windmill(this.elevatorSubsystem, this.pivotSubsystem, WindmillState.CoralScore2, mirrored),
                        eject);
            } else if (reefLevel == ReefLevel.Three) {
                commands.addCommands(
                        new Windmill(this.elevatorSubsystem, this.pivotSubsystem, WindmillState.CoralScore3, mirrored),
                        eject);
            } else if (reefLevel == ReefLevel.Four) {
                commands.addCommands(
                        new WindmillSlow(this.elevatorSubsystem, this.pivotSubsystem, WindmillState.CoralScore4,
                                mirrored),
                        eject);
            }
        } else {
            if (reefLevel == ReefLevel.None) {
                commands.addCommands(
                        eject);
            } else if (reefLevel == ReefLevel.Two) {
                commands.addCommands(
                        new Windmill(this.elevatorSubsystem, this.pivotSubsystem, WindmillState.CoralLeftScore2,
                                false),
                        eject);
            } else if (reefLevel == ReefLevel.Three) {
                commands.addCommands(
                        new Windmill(this.elevatorSubsystem, this.pivotSubsystem, WindmillState.CoralLeftScore3,
                                false),
                        eject);
            } else if (reefLevel == ReefLevel.Four) {
                commands.addCommands(
                        new WindmillSlow(this.elevatorSubsystem, this.pivotSubsystem, WindmillState.CoralLeftScore4,
                                false),
                        eject);
            }
        }

        commands.schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}