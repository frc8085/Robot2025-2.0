package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.ClimberSubsystem;

public class DeployClimb extends SequentialCommandGroup {
    public DeployClimb(
            ClimberSubsystem climberSubsystem) {
        addCommands(
                // Switch to a transition state
                // Maybe turn off all the motors

                // Check safety
                // new Windmill(elevatorSubsystem, pivotSubsystem,
                // Constants.Windmill.WindmillState.Climb, false),
                new RunCommand(() -> climberSubsystem.moveUp(), climberSubsystem).withTimeout(2),
                new InstantCommand(climberSubsystem::stop));
        // Switch to target state.
    }
}
