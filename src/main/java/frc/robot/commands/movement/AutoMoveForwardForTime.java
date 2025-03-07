package frc.robot.commands.movement;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoMoveForwardForTime extends SequentialCommandGroup {
    public AutoMoveForwardForTime(DriveSubsystem drive, LimelightSubsystem limelight, boolean yellow, double time) {
        if (yellow) {
            addCommands(
                    new ParallelDeadlineGroup(
                            new WaitCommand(time),
                            new AutoDriveMeters(drive, 0, -1, 0.2))); // move forward for yellow;
        } else {
            addCommands(
                    new ParallelDeadlineGroup(
                            new WaitCommand(time),
                            new AutoDriveMeters(drive, 0, 1, 0.2))); // move forward for blue;

        }
    }
}