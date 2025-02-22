package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Windmill.WindmillState;
import frc.robot.commands.states.ToCoralPickUp;
import frc.robot.commands.states.ToHomeCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class PickUpCoralFromSource extends SequentialCommandGroup {
        public PickUpCoralFromSource(
                        CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem) {
                addCommands(
                                new ToCoralPickUp(elevatorSubsystem, pivotSubsystem, WindmillState.Home),
                                new PickUpCoral(coralSubsystem),
                                new WaitCommand(0.25),
                                new ToHomeCommand(elevatorSubsystem, pivotSubsystem, WindmillState.CoralPickup));

        }
}
