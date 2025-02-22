package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// if command is interrupted before coral is picked up, kill command
public class PickUpAlgaeL3 extends SequentialCommandGroup {

        public PickUpAlgaeL3(
                        AlgaeSubsystem algaeSubsystem, ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem) {
                addCommands(
                                new InstantCommand(algaeSubsystem::holdAlgae),
                                new ParallelCommandGroup(
                                                new InstantCommand(() -> elevatorSubsystem
                                                                .setPos(ElevatorConstants.kElevatorReef3Height - 20)),
                                                new InstantCommand(() -> pivotSubsystem
                                                                .setPos(Rotation2d.fromDegrees(-40)))));
        }
}
