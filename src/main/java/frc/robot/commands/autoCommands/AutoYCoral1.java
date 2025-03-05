package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.DropCoral;
import frc.robot.commands.states.ToCoralDropOff1;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class AutoYCoral1 extends SequentialCommandGroup {
        public AutoYCoral1(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem, boolean mirror) {
                addCommands(
                                new ToCoralDropOff1(elevatorSubsystem, pivotSubsystem, mirror),
                                new ParallelCommandGroup(
                                                new SequentialCommandGroup(new WaitUntilCommand(
                                                                elevatorSubsystem::elevatorAtCoralDropOff1Height),
                                                                new PrintCommand("At Elevator Drop Off")),
                                                new SequentialCommandGroup(new WaitUntilCommand(
                                                                () -> pivotSubsystem.pivotAtCoralDropOffAngle(mirror)),
                                                                new PrintCommand("At Pivot Drop Off"))),
                                new DropCoral(coralSubsystem, elevatorSubsystem, pivotSubsystem));
        }
}