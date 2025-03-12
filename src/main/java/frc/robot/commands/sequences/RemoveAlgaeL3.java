package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.manipulator.algae.PickUpAlgaeCurrent;
import frc.robot.commands.windmill.Windmill;
import frc.robot.commands.states.ToAlgaeL3;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class RemoveAlgaeL3 extends SequentialCommandGroup {
        public RemoveAlgaeL3(ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem, AlgaeSubsystem algaeSubsystem,
                        boolean yellow) {
                addCommands(
                                new ToAlgaeL3(elevatorSubsystem, pivotSubsystem, yellow),
                                new PickUpAlgaeCurrent(algaeSubsystem),
                                new WaitCommand(.25),
                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                Constants.Windmill.WindmillState.AlgaeHoldHeight, yellow),
                                new WaitUntilCommand(() -> pivotSubsystem
                                                .pivotAtHomeAngle()),
                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                Constants.Windmill.WindmillState.CoralDropOff3, yellow));

        }
}
