package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.manipulator.coral.*;
import frc.robot.commands.states.ToCoralDropOff4;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ScoreCoralL4 extends SequentialCommandGroup {
        public ScoreCoralL4(
                        ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem, CoralSubsystem coralSubsystem, DriveSubsystem driveSubsystem,
                        boolean yellow) {
                addCommands(
                                new ToCoralDropOff4(elevatorSubsystem, pivotSubsystem, yellow),
                                new WaitUntilCommand(elevatorSubsystem::elevatorAtCoralDropOff4Height),
                                new WaitUntilCommand(() -> pivotSubsystem.pivotAtCoral4DropOffAngle(yellow)),
                                new WaitCommand(.5),
                                new EjectCoral(coralSubsystem, elevatorSubsystem, pivotSubsystem));
        }
}
