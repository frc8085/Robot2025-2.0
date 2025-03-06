package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.EjectCoral;
import frc.robot.commands.states.ToCoralDropOff1;
import frc.robot.commands.states.ToHomeCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ScoreCoralL1 extends SequentialCommandGroup {
        public ScoreCoralL1(
                        ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem, CoralSubsystem coralSubsystem, boolean yellow) {
                addCommands(
                                new ToCoralDropOff1(elevatorSubsystem, pivotSubsystem, yellow),
                                new WaitUntilCommand(elevatorSubsystem::elevatorAtCoralDropOff1Height),
                                new WaitUntilCommand(() -> pivotSubsystem.pivotAtCoralDropOffAngle(yellow)),
                                new EjectCoral(coralSubsystem, elevatorSubsystem, pivotSubsystem),
                                new WaitCommand(0.25),
                                new ToHomeCommand(elevatorSubsystem, pivotSubsystem,
                                                coralSubsystem));
        }

}
