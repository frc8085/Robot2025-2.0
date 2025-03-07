package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ElevatorSubsystem;

public class WaitUntilElevatorBelowSafeTravelHeight extends SequentialCommandGroup {
        public WaitUntilElevatorBelowSafeTravelHeight(ElevatorSubsystem elevatorSubsystem) {
                addCommands(
                                new WaitUntilCommand(elevatorSubsystem::elevatorBelowSafeTravelHeight));
        }
}