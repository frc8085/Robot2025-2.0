package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class PickUpCoralFromSource extends SequentialCommandGroup {
        public PickUpCoralFromSource(
                        CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem, boolean altButton) {
                if (altButton) {
                        addCommands(
                                        new Windmill(elevatorSubsystem, pivotSubsystem,
                                                        Constants.Windmill.WindmillState.CoralPickupAlternate, false),
                                        new PickUpCoral(coralSubsystem),
                                        new WaitCommand(0.25),
                                        new Windmill(elevatorSubsystem, pivotSubsystem,
                                                        Constants.Windmill.WindmillState.Home, false));

                } else {
                        addCommands(
                                        new Windmill(elevatorSubsystem, pivotSubsystem,
                                                        Constants.Windmill.WindmillState.CoralPickupHigher, false),
                                        new PickUpCoral(coralSubsystem),
                                        new WaitCommand(0.25),
                                        new Windmill(elevatorSubsystem, pivotSubsystem,
                                                        Constants.Windmill.WindmillState.Home, false));
                }
        }
}
