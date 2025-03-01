package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.commands.Windmill;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ToHomeCommand extends ParallelCommandGroup {
    public ToHomeCommand(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem,
            CoralSubsystem coralSubsystem) {
        addCommands(
                // Switch to a transition state

                // Turn off Coral Motor
                new RunCommand(() -> coralSubsystem.stop(), coralSubsystem).withTimeout(0.25),

                // go home
                new Windmill(elevatorSubsystem, pivotSubsystem,
                        Constants.Windmill.WindmillState.Home, false));

        // Switch to target state.
    }
}