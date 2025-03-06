package frc.robot.commands.scoring;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.AlgaeLevel;
import frc.robot.RobotContainer.CoralLevel;
import frc.robot.commands.DropCoral;
import frc.robot.commands.states.ToCoralDropOff1;
import frc.robot.commands.states.ToCoralDropOff2;
import frc.robot.commands.states.ToCoralDropOff3;
import frc.robot.commands.states.ToCoralDropOff4;
import frc.robot.commands.states.ToHomeCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ToCoralScoreBlue extends SequentialCommandGroup {
    public ToCoralScoreBlue(CoralSubsystem coral, ElevatorSubsystem elevator, PivotSubsystem pivot,
            DriveSubsystem drive) {

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

        scoreCoral(coral, elevator, pivot, drive);
    }

    public void scoreCoral(CoralSubsystem coral, ElevatorSubsystem elevator, PivotSubsystem pivot,
            DriveSubsystem drive) {

        switch (RobotContainer.coralLevel) {
            case ONE:
                addCommands(
                        new ToCoralDropOff1(elevator, pivot, false),
                        new ParallelCommandGroup(
                                new WaitUntilCommand(elevator::elevatorAtCoralDropOff1Height),
                                new WaitUntilCommand(() -> pivot.pivotAtCoralDropOffAngle(false))),
                        new DropCoral(coral, elevator, pivot),
                        new ToHomeCommand(elevator, pivot, coral));

                break;
            case TWO:
                addCommands(
                        new ToCoralDropOff2(elevator, pivot, false),
                        new ParallelCommandGroup(
                                new WaitUntilCommand(elevator::elevatorAtCoralDropOff2Height),
                                new WaitUntilCommand(() -> pivot.pivotAtCoralDropOffAngle(false))),
                        new DropCoral(coral, elevator, pivot),
                        new ToHomeCommand(elevator, pivot, coral));

                break;
            case THREE:
                addCommands(
                        new ToCoralDropOff3(elevator, pivot, false),
                        new ParallelCommandGroup(
                                new WaitUntilCommand(elevator::elevatorAtCoralDropOff3Height),
                                new WaitUntilCommand(() -> pivot.pivotAtCoralDropOffAngle(false))),
                        new DropCoral(coral, elevator, pivot),
                        new ToHomeCommand(elevator, pivot, coral));
                break;
            case FOUR:
                addCommands(
                        new ToCoralDropOff4(elevator, pivot, false),
                        new ParallelCommandGroup(
                                new WaitUntilCommand(elevator::elevatorAtCoralDropOff4Height),
                                new WaitUntilCommand(() -> pivot.pivotAtCoral4DropOffAngle(false))),
                        new DropCoral(coral, elevator, pivot),
                        new ToHomeCommand(elevator, pivot, coral));

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
                        new ToCoralScoreBlue(coral, elevator, pivot, drive));
                break;
        }
    }

}
