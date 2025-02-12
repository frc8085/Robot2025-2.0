package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

//Copied and pasted from 2023; still a work in progress, ignore errors
public class MoveToTravel extends SequentialCommandGroup {
        public MoveToTravel(ElevatorSubsystem m_extension)
        {
            addCommands(
                new InstantCommand(() -> m_extension.keepPositionInches(
                    ElevatorConstants.kElevatorPositionInchesFullyRetracted)));
        }
}