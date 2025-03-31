package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.windmill.Windmill;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector.EndEffectorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.Constants.Windmill.WindmillState;
import frc.robot.commands.endEffector.Handoff;

public class ToCoralDropOff extends SequentialCommandGroup {
        public ToCoralDropOff(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem,
                        IntakeSubsystem intakeSubsystem, EndEffectorSubsystem endEffectorSubsystem, WindmillState state,
                        boolean yellow) {
                addCommands(
                                new Windmill(elevatorSubsystem, pivotSubsystem, WindmillState.CoralHandoff, false),
                                new Handoff(intakeSubsystem, endEffectorSubsystem));
                if (yellow) {
                        addCommands(
                                        new PrintCommand("Move to Y Coral Drop Off 1"),
                                        new Windmill(elevatorSubsystem, pivotSubsystem,
                                                        state,
                                                        true));
                } else {
                        addCommands(
                                        new PrintCommand("Move to B Coral Drop Off 1"),
                                        new Windmill(elevatorSubsystem, pivotSubsystem,
                                                        state,
                                                        false));
                }

        }
}