package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutoWaitUntilElevatorBelowSafeTravelHeight extends SequentialCommandGroup {
        public AutoWaitUntilElevatorBelowSafeTravelHeight(ElevatorSubsystem elevatorSubsystem) {
                addCommands(
                                new WaitUntilCommand(elevatorSubsystem::elevatorBelowSafeTravelHeight),
                                new PrintCommand("Safe to move"));
        }
}