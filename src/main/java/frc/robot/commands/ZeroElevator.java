package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ElevatorSubsystem;

public class ZeroElevator extends
        SequentialCommandGroup {

    public ZeroElevator(ElevatorSubsystem elevatorSubsystem) {
        addCommands(
                new InstantCommand(elevatorSubsystem::moveUp),
                new WaitUntilCommand(elevatorSubsystem::ElevatorZeroLimitHit),
                new InstantCommand(elevatorSubsystem::stop),
                new InstantCommand(elevatorSubsystem::zero));
    }
}