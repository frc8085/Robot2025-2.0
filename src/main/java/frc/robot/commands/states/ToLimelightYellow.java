package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Windmill;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

// Used to make sure that the coral flags are not blocking the april tag / camera
public class ToLimelightYellow extends SequentialCommandGroup {
    public ToLimelightYellow(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem) {
        addCommands(
                new Windmill(elevatorSubsystem, pivotSubsystem,
                        Constants.Windmill.WindmillState.LimelightYellowTarget,
                        false));
    }

}
