package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.states.ToCoralDropOff2;
import frc.robot.commands.manipulator.coral.*;

import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ScoreCoralL2 extends SequentialCommandGroup {
        public ScoreCoralL2(
                        ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem, CoralSubsystem coralSubsystem, boolean yellow) {
                addCommands(
                                new ToCoralDropOff2(elevatorSubsystem, pivotSubsystem, yellow),
                                new WaitUntilCommand(elevatorSubsystem::elevatorAtCoralDropOff2Height),
                                new WaitUntilCommand(() -> pivotSubsystem.pivotAtCoralDropOffAngle(yellow)),
                                new WaitCommand(0.25),
                                new EjectCoral(coralSubsystem, elevatorSubsystem, pivotSubsystem));
        }

}
