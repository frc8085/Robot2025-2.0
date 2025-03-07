package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.EjectCoral;
import frc.robot.commands.states.ToCoralDropOff4;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class AutoCoral4 extends SequentialCommandGroup {
        public AutoCoral4(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem, boolean yellow) {
                addCommands(
                                new ToCoralDropOff4(elevatorSubsystem, pivotSubsystem, yellow),
                                new ParallelCommandGroup(
                                                new SequentialCommandGroup(new WaitUntilCommand(
                                                                elevatorSubsystem::elevatorAtCoralDropOff4Height),
                                                                new PrintCommand("At Elevator Drop Off")),
                                                new SequentialCommandGroup(new WaitUntilCommand(
                                                                () -> pivotSubsystem.pivotAtCoral4DropOffAngle(yellow)),
                                                                new PrintCommand("At Pivot Drop Off"))),
                                new WaitCommand(0.5),
                                new EjectCoral(coralSubsystem, elevatorSubsystem, pivotSubsystem));
        }
}