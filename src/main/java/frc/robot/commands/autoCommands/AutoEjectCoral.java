package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.states.ToHomeCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class AutoEjectCoral extends SequentialCommandGroup {
        public AutoEjectCoral(
                        CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem) {
                addCommands(
                                new PrintCommand("Coral Eject Started"),
                                new RunCommand(() -> coralSubsystem.eject(), coralSubsystem).withTimeout(.5),
                                new InstantCommand(coralSubsystem::stop),
                                new ToHomeCommand(elevatorSubsystem, pivotSubsystem, coralSubsystem));

        }
}
