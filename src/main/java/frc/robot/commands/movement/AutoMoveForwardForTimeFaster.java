package frc.robot.commands.movement;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoMoveForwardForTimeFaster extends SequentialCommandGroup {
    public AutoMoveForwardForTimeFaster(DriveSubsystem drive, LimelightSubsystem limelight, boolean yellow,
            double time, double speed) {
        // negative sideways numbers moves it towards the yellow side

        if (yellow) {
            addCommands(
                    new ParallelDeadlineGroup(
                            new WaitCommand(time),
                            new AutoDriveMeters(drive, 0, -1, speed))); // move forward for yellow;
        } else {
            addCommands(
                    new ParallelDeadlineGroup(
                            new WaitCommand(time),
                            new AutoDriveMeters(drive, 0, 1, speed))); // move forward for blue;

        }
    }
}