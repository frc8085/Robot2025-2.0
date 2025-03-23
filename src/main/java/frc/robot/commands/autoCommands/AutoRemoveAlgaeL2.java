package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.windmill.Windmill;
import frc.robot.commands.manipulator.algae.PickUpAlgae;
import frc.robot.commands.manipulator.algae.PickUpAlgaeCurrent;
import frc.robot.commands.states.ToAlgaeL2;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class AutoRemoveAlgaeL2 extends SequentialCommandGroup {
        public AutoRemoveAlgaeL2(ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem, AlgaeSubsystem algaeSubsystem) {
                addCommands(
                                new ToAlgaeL2(elevatorSubsystem, pivotSubsystem, false),
                                new PickUpAlgaeCurrent(algaeSubsystem),
                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                Constants.Windmill.WindmillState.Home, false));
        }
}
