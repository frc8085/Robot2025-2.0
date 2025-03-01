package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.commands.Windmill;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ToClimb extends SequentialCommandGroup {
    public ToClimb(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem,
            ClimberSubsystem climberSubsystem) {
        addCommands(
                // Switch to a transition state
                // Maybe turn off all the motors

                // Check safety
                new Windmill(elevatorSubsystem, pivotSubsystem, Constants.Windmill.WindmillState.Climb, false),
                new RunCommand(() -> climberSubsystem.moveUp(), climberSubsystem).withTimeout(2),
                new InstantCommand(climberSubsystem::stop));
        // Switch to target state.
    }
}