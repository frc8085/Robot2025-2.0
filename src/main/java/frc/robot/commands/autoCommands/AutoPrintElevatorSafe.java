package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutoPrintElevatorSafe extends SequentialCommandGroup {
    public AutoPrintElevatorSafe(ElevatorSubsystem elevator) {
        new PrintCommand(String.format("", elevator.elevatorBelowSafeTravelHeight()));
    }
}
