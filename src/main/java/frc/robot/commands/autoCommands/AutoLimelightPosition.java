package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.movement.AlignToAprilTagYellow;
import frc.robot.commands.movement.AutoPositionLeftRight;
import frc.robot.commands.movement.DriveToReefYellow;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoLimelightPosition extends SequentialCommandGroup {
    public AutoLimelightPosition(DriveSubsystem drive, LimelightSubsystem limelight, boolean right, boolean yellow) {
        addCommands(
                new DriveToReefYellow(drive, limelight),
                new AlignToAprilTagYellow(drive, limelight),
                new AutoPositionLeftRight(drive, limelight, right, yellow));
    }

}
