package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// if command is interrupted before coral is picked up, kill command
public class PickUpCoral extends SequentialCommandGroup {

        public PickUpCoral(
                        CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem) {
                addCommands(
                                // new Windmill(elevatorSubsystem, pivotSubsystem,
                                //                 Constants.Windmill.WindmillState.CoralPickup,
                                //                 false),
                                new InstantCommand(coralSubsystem::pickup),
                                new ParallelRaceGroup(
                                                new WaitUntilCommand(coralSubsystem::isCoralDetected),
                                                new WaitCommand(4)),
                                new InstantCommand(coralSubsystem::stop),
                                new WaitCommand(0.25),
                                new Windmill(elevatorSubsystem, pivotSubsystem, Constants.Windmill.WindmillState.Home,
                                                false));

        }
}
