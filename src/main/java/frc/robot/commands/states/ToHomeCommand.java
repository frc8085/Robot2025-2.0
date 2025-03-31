package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.commands.windmill.Windmill;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class ToHomeCommand extends ParallelCommandGroup {
    public ToHomeCommand(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem) {
        addCommands(
                new PrintCommand("Move to Home"),
                new Windmill(elevatorSubsystem, pivotSubsystem,
                        Constants.Windmill.WindmillState.Home, false));
    }
}