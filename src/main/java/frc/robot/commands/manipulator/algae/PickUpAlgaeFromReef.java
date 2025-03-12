package frc.robot.commands.manipulator.algae;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.states.ToAlgaeL2;
import frc.robot.commands.states.ToAlgaeL3;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.commands.windmill.Windmill;

public class PickUpAlgaeFromReef extends SequentialCommandGroup {
        public PickUpAlgaeFromReef(
                        AlgaeSubsystem algaeSubsystem, ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem, boolean level, boolean yellow) {

                // level true= L3, false = L2; yellow true = right, false = blue/left
                if (level) {
                        if (yellow) {
                                addCommands(
                                                new ToAlgaeL3(elevatorSubsystem, pivotSubsystem, true),
                                                new PickUpAlgae(algaeSubsystem),
                                                new WaitCommand(.25),
                                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                                Constants.Windmill.WindmillState.CoralDropOff3,
                                                                false));
                        } else {
                                addCommands(
                                                new ToAlgaeL3(elevatorSubsystem, pivotSubsystem, false),
                                                new PickUpAlgae(algaeSubsystem),
                                                new WaitCommand(.25),
                                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                                Constants.Windmill.WindmillState.CoralDropOff3,
                                                                false));

                        }
                } else {
                        if (yellow) {
                                addCommands(
                                                new ToAlgaeL2(elevatorSubsystem, pivotSubsystem, true),
                                                new PickUpAlgae(algaeSubsystem),
                                                new WaitCommand(.25),
                                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                                Constants.Windmill.WindmillState.AlgaeHoldHeight,
                                                                false));

                        } else {
                                addCommands(
                                                new ToAlgaeL2(elevatorSubsystem, pivotSubsystem, false),
                                                new PickUpAlgae(algaeSubsystem));

                        }

                }
        }
}
