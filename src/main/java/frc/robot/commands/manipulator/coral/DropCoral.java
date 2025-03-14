package frc.robot.commands.manipulator.coral;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.states.ToHomeCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class DropCoral extends SequentialCommandGroup {
        public DropCoral(
                        CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem, DriveSubsystem driveSubsystem) {
                addCommands(
                                new PrintCommand("Coral Drop Started"),
                                new RunCommand(() -> coralSubsystem.drop(), coralSubsystem).withTimeout(1),
                                new InstantCommand(coralSubsystem::stop),
                                new ToHomeCommand(elevatorSubsystem, pivotSubsystem));
        }
}
