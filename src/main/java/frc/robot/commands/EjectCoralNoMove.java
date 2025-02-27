package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class EjectCoralNoMove extends SequentialCommandGroup {
        public EjectCoralNoMove(
                        CoralSubsystem coralSubsystem) {
                addCommands(
                                new PrintCommand("Coral Eject Started"),
                                new InstantCommand(coralSubsystem::eject),
                                new WaitCommand(1),
                                new InstantCommand(coralSubsystem::stop));

        }
}
