package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.EjectCoral;
import frc.robot.commands.states.ToCoralDropOff3;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class AutoCoral3 extends SequentialCommandGroup {
        public AutoCoral3(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem, boolean yellow) {
                addCommands(
                                new ToCoralDropOff3(elevatorSubsystem, pivotSubsystem, yellow),
                                new ParallelCommandGroup(
                                                new SequentialCommandGroup(new WaitUntilCommand(
                                                                elevatorSubsystem::elevatorAtCoralDropOff3Height),
                                                                new PrintCommand("At Elevator Drop Off")),
                                                new SequentialCommandGroup(new WaitUntilCommand(
                                                                () -> pivotSubsystem.pivotAtCoralDropOffAngle(yellow)),
                                                                new PrintCommand("At Pivot Drop Off"))),
                                new EjectCoral(coralSubsystem, elevatorSubsystem, pivotSubsystem));
        }
}