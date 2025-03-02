package frc.robot.commands.scoring;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.AlgaeLevel;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ToAlgaeScore extends SequentialCommandGroup {
    public ToAlgaeScore(AlgaeSubsystem algae, PivotSubsystem pivot) {
        switch (RobotContainer.algaeLevel) {
            // Add the specific commands in states in here.
            case TWO:
                addCommands(

                );
                break;
            case THREE:
                addCommands(

                );
                break;
            case NONE:
                break;
            case UNDECIDED:
                addCommands(
                        // Waits until a level is given and will rerun this current command
                        // TODO: Add a way to exit this command in case you don't want to score or
                        // change your mind
                        new WaitUntilCommand(
                                new BooleanSupplier() {
                                    @Override
                                    public boolean getAsBoolean() {
                                        return RobotContainer.algaeLevel != AlgaeLevel.UNDECIDED;
                                    }
                                }),
                        new ToAlgaeScore(algae, pivot));
                break;
        }
    }
}