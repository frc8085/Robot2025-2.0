package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.PickUpAlgae;
import frc.robot.commands.Windmill;
import frc.robot.commands.states.ToAlgaeL2;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class AutoRemoveAlgaeL2 extends SequentialCommandGroup {
        public AutoRemoveAlgaeL2(ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem, AlgaeSubsystem algaeSubsystem,
                        boolean yellow) {
                addCommands(
                                new PickUpAlgae(algaeSubsystem),
                                new WaitCommand(.25),
                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                Constants.Windmill.WindmillState.Home,
                                                false));

        }
}