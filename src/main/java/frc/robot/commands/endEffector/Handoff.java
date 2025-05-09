package frc.robot.commands.endEffector;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.EndEffector.EndEffectorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import frc.robot.commands.intake.EjectHandoffCoral;

public class Handoff extends SequentialCommandGroup {
        public Handoff(
                        IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem,
                        EndEffectorSubsystem endEffectorSubsystem) {

                addCommands(
                                new ParallelCommandGroup(
                                                // new InstantCommand(() -> intakeSubsystem.enableOuterRollers()),
                                                new PrintCommand("Handoff 1"),
                                                new InstantCommand(endEffectorSubsystem::pickup),
                                                new PrintCommand("Handoff 2")),
                                new WaitUntilCommand(pivotSubsystem::pivotAtHomeAngle),
                                new PrintCommand("Handoff 3"),
                                new EjectHandoffCoral(intakeSubsystem),
                                new PrintCommand("Handoff 4"),
                                new InstantCommand(() -> endEffectorSubsystem.stop()));
        }
}