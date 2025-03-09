package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.EjectCoral;
import frc.robot.commands.states.ToCoralDropOff2;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class AutoCoral2 extends SequentialCommandGroup {
        public AutoCoral2(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem, DriveSubsystem driveSubsystem, boolean yellow) {
                addCommands(
                                new ToCoralDropOff2(elevatorSubsystem, pivotSubsystem, yellow),
                                new ParallelCommandGroup(
                                                new SequentialCommandGroup(new WaitUntilCommand(
                                                                elevatorSubsystem::elevatorAtCoralDropOff2Height),
                                                                new PrintCommand("At Elevator Drop Off")),
                                                new SequentialCommandGroup(new WaitUntilCommand(
                                                                () -> pivotSubsystem.pivotAtCoralDropOffAngle(yellow)),
                                                                new PrintCommand("At Pivot Drop Off"))),
                                new WaitCommand(0.5),
                                new EjectCoral(coralSubsystem, elevatorSubsystem, pivotSubsystem, driveSubsystem));
        }
}