package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.windmill.Windmill;
import frc.robot.commands.manipulator.algae.PickUpAlgaeCurrent;
import frc.robot.commands.states.ToAlgaeL3;
import frc.robot.subsystems.Algae.AlgaeSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem;

public class AutoRemoveAlgaeL3 extends SequentialCommandGroup {
        public AutoRemoveAlgaeL3(ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem, AlgaeSubsystem algaeSubsystem) {
                addCommands(
                                new ToAlgaeL3(elevatorSubsystem, pivotSubsystem, false),
                                new PickUpAlgaeCurrent(algaeSubsystem),
                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                Constants.Windmill.WindmillState.Home, false));
        }
}
