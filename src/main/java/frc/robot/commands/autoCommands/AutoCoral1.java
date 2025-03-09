package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.DropCoral;
import frc.robot.commands.states.ToCoralDropOff1;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class AutoCoral1 extends SequentialCommandGroup {
        public AutoCoral1(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem, DriveSubsystem driveSubsystem, boolean yellow) {
                addCommands(
                                new ToCoralDropOff1(elevatorSubsystem, pivotSubsystem, yellow),
                                new ParallelCommandGroup(
                                                new SequentialCommandGroup(new WaitUntilCommand(
                                                                elevatorSubsystem::elevatorAtCoralDropOff1Height),
                                                                new PrintCommand("At Elevator Drop Off")),
                                                new SequentialCommandGroup(new WaitUntilCommand(
                                                                () -> pivotSubsystem.pivotAtCoralDropOffAngle(yellow)),
                                                                new PrintCommand("At Pivot Drop Off"))),
                                new WaitCommand(0.5),
                                new DropCoral(coralSubsystem, elevatorSubsystem, pivotSubsystem, driveSubsystem));
        }
}