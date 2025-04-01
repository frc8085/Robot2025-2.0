package frc.robot.commands.states;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
        One,
        Two,
        Three,
        Four,
    }

    private ElevatorSubsystem elevatorSubsystem;
    private PivotSubsystem pivotSubsystem;
    private EndEffectorSubsystem endEffectorSubsystem;
    private ReefLevel reefLevel;
    private boolean mirrored;

    ScoreReef(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem, EndEffectorSubsystem endEffectorSubsystem, ReefLevel level, boolean mirrored) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.reefLevel = level;
        this.mirrored = mirrored;
    }

    @Override
    public void initialize() {
        // get current hight and pivot angle
        double currentElevatorHeight = elevatorSubsystem.getCurrentMotorPosition();
        Rotation2d currentPivotAngle = pivotSubsystem.getCurrentRotation();

        ParallelCommandGroup commands = new ParallelCommandGroup();

        switch (this.reefLevel) {
            case One:
                commands.addCommands(
                    new RunEndEffector(this.endEffectorSubsystem, mirrored)
                );
                break;
            case Two:
                commands.addCommands(
                    new Windmill(this.elevatorSubsystem, this.pivotSubsystem, WindmillState.CoralScore2, mirrored)
                );
            case Three:
                commands.addCommands(
                    new Windmill(this.elevatorSubsystem, this.pivotSubsystem, WindmillState.CoralScore3, mirrored)
                );
            case Four:
                commands.addCommands(
                    new Windmill(this.elevatorSubsystem, this.pivotSubsystem, WindmillState.CoralScore4, mirrored)
                );
                break;
        }

        commands.schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}