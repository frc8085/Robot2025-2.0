package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.states.ToHomeCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class DropCoral extends SequentialCommandGroup {
        public DropCoral(
                        CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem) {
                addCommands(
                                new PrintCommand("Coral DropOff Started"),
                                new RunCommand(() -> coralSubsystem.drop(), coralSubsystem).withTimeout(1),
                                new InstantCommand(coralSubsystem::stop),
                                // new WaitCommand(0.25),
                                new ToHomeCommand(elevatorSubsystem, pivotSubsystem, coralSubsystem));

        }
}
