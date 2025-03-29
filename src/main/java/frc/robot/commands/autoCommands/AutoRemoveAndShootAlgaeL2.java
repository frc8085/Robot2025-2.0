package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.windmill.Windmill;
import frc.robot.commands.manipulator.algae.PickUpAlgae;
import frc.robot.commands.manipulator.coral.DropCoral;
import frc.robot.commands.manipulator.coral.EjectCoral;
import frc.robot.commands.states.ToAlgaeL2;
import frc.robot.subsystems.Algae.AlgaeSubsystem;
import frc.robot.subsystems.Coral.CoralSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem;

public class AutoRemoveAndShootAlgaeL2 extends SequentialCommandGroup {
        public AutoRemoveAndShootAlgaeL2(ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem, AlgaeSubsystem algaeSubsystem, CoralSubsystem coralSubsystem) {
                addCommands(
                                new ToAlgaeL2(elevatorSubsystem, pivotSubsystem, false),
                                new ParallelRaceGroup(new PickUpAlgae(algaeSubsystem),
                                                new SequentialCommandGroup(
                                                                new WaitCommand(5),
                                                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                                                Constants.Windmill.WindmillState.CoralDropOff1,
                                                                                false),
                                                                new WaitCommand(1),
                                                                new DropCoral(coralSubsystem, elevatorSubsystem,
                                                                                pivotSubsystem)),
                                                new WaitCommand(10)),
                                new WaitCommand(.25),
                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                Constants.Windmill.WindmillState.Home, false),
                                new ParallelRaceGroup(new WaitUntilCommand(() -> pivotSubsystem
                                                .pivotAtHomeAngle()), new WaitCommand(.5)),
                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                Constants.Windmill.WindmillState.CoralDropOff3, false),
                                new ParallelCommandGroup(
                                                new WaitUntilCommand(
                                                                () -> pivotSubsystem.pivotAtCoralDropOffAngle(false)),
                                                new WaitUntilCommand(() -> elevatorSubsystem
                                                                .elevatorAtCoralDropOff3Height())),
                                new EjectCoral(coralSubsystem, elevatorSubsystem, pivotSubsystem));

        }
}
