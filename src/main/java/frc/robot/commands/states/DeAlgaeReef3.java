package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.windmill.Windmill;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector.EndEffectorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.Constants.Windmill.WindmillState;
import frc.robot.commands.endEffector.Algae;
import frc.robot.commands.endEffector.Handoff;

public class DeAlgaeReef3 extends SequentialCommandGroup {
        public DeAlgaeReef3(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem,
                        EndEffectorSubsystem endEffectorSubsystem) {
                addCommands(
                                new SequentialCommandGroup(
                                                new PrintCommand("Performing Algae Pickup"),
                                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                                WindmillState.AlgaePickUpReef3, false),
                                                new PrintCommand("Move to Reef3"),
                                                new Algae(endEffectorSubsystem)));
        }
}