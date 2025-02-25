package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Windmill.WindmillState;
import frc.robot.commands.Windmill;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ToHomeCommand extends SequentialCommandGroup {
    public ToHomeCommand(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem,
            CoralSubsystem coralSubsystem) {
        addCommands(
                // Switch to a transition state

                // Turn off Coral Motor
                new InstantCommand(coralSubsystem::stop),

                // go home
                new Windmill(elevatorSubsystem, pivotSubsystem,
                        Constants.Windmill.WindmillState.Home, false));

        // Switch to target state.
    }
}