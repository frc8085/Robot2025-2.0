package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.windmill.Windmill;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector.EndEffectorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.Constants.Windmill.WindmillState;
import frc.robot.commands.endEffector.Handoff;

public class TestHandoff extends SequentialCommandGroup {
        public TestHandoff(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem,
                        IntakeSubsystem intakeSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
                addCommands(
                                new ConditionalCommand(new SequentialCommandGroup(
                                                new PrintCommand("Performing Coral Handoff"),
                                                // new ParallelCommandGroup(
                                                // new Windmill(elevatorSubsystem, pivotSubsystem,
                                                // WindmillState.CoralHandoff, false),
                                                // new RetractIntake(intakeSubsystem)),
                                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                                WindmillState.CoralHandoff, false),
                                                new WaitCommand(2),
                                                new Handoff(intakeSubsystem, endEffectorSubsystem)),
                                                new WaitCommand(0),
                                                () -> intakeSubsystem.hasCoralCentered()));
        }
}