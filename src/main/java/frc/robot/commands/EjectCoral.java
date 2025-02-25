package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class EjectCoral extends SequentialCommandGroup {
        public EjectCoral(
                        CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem) {
                addCommands(

                                new InstantCommand(coralSubsystem::eject),
                                new WaitCommand(1),
                                new InstantCommand(coralSubsystem::stop),
                                new WaitCommand(0.25));
                                // new Windmill(elevatorSubsystem, pivotSubsystem, Constants.Windmill.WindmillState.Home,
                                //                 false));

        }
}
