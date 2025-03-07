package frc.robot.commands.automated;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.movement.AlignToAprilTagYellow;
import frc.robot.commands.movement.DriveToReefYellow;
import frc.robot.commands.states.ToLimelightYellow;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class AlignAndDriveYellow extends SequentialCommandGroup {
    public AlignAndDriveYellow(DriveSubsystem drive, LimelightSubsystem limelight, ElevatorSubsystem elevator,
            PivotSubsystem pivot) {
        addCommands(
                new AlignToAprilTagYellow(drive, limelight),
                new ToLimelightYellow(elevator, pivot),
                new DriveToReefYellow(drive, limelight),
                new AlignToAprilTagYellow(drive, limelight));

    }

}
