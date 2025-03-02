package frc.robot.commands.scoring;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.AlgaeLevel;
import frc.robot.RobotContainer.CoralLevel;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ToCoralScore extends SequentialCommandGroup {
    public ToCoralScore(CoralSubsystem coral, PivotSubsystem pivot) {
        // Logic structure Assuming blue limelight
        boolean blue = true;

        /*
         * TODO: Implemment the logic to deduct which side of the robot you are scoring
         * on.
         * 
         * Couple ways to do this:
         * 1. Keep track of the last limelight that saw something and the distance.
         * Both cameras will potentially see a tag, but we should know which one is in
         * front of us because it's closer
         * 
         * 2. Keep track of the closest tag within a reasonable time frame.
         * Look at the tag ID to know where the robot is and which direction the robot
         * should be facing
         * Then look at the robot's gyro information to see if it is facing the right
         * direction
         */

        switch (RobotContainer.scoreDirection) {
            case LEFT:
                if (!blue && RobotContainer.algaeLevel != AlgaeLevel.NONE) {
                    RobotContainer.coralLevel = CoralLevel.NONE;
                }
                break;
            case RIGHT:
                if (blue && RobotContainer.algaeLevel != AlgaeLevel.NONE) {
                    RobotContainer.coralLevel = CoralLevel.NONE;
                }
                break;
            // You shouldn't really get to this case.
            case UNDECIDED:
                break;
        }

        scoreCoral(coral, pivot);
    }

    public void scoreCoral(CoralSubsystem coral, PivotSubsystem pivot) {
        switch (RobotContainer.coralLevel) {
            case ONE:
                addCommands(

                );
                break;
            case TWO:
                addCommands(

                );
                break;
            case THREE:
                addCommands(

                );
                break;
            case FOUR:
                break;
            case NONE:
                break;

            default:
                addCommands(
                        // Waits until a level is given and will rerun this current command
                        // TODO: Add a way to exit this command in case you don't want to score for some
                        // reason.
                        new WaitUntilCommand(
                                new BooleanSupplier() {
                                    @Override
                                    public boolean getAsBoolean() {
                                        return RobotContainer.coralLevel != CoralLevel.UNDECIDED;
                                    }
                                }),
                        new ToCoralScore(coral, pivot));
                break;
        }
    }
}