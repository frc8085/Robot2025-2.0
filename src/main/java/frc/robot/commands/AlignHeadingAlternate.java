package frc.robot.commands;

import java.lang.annotation.Target;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AlignHeadingAlternate extends Command {
    private final DriveSubsystem drive;
    private final LimelightSubsystem limelight;

    private final int tagid;
    private final int desiredAngle;

    private final int tolerance = 3;

    public boolean hasTarget = true;

    public AlignHeadingAlternate(DriveSubsystem drive, LimelightSubsystem limelight, boolean yellow) {
        this.drive = drive;
        this.limelight = limelight;

        if (yellow) {
            tagid = limelight.getID("limelight-yellow");
        } else {
            tagid = limelight.getID("limelight-blue");
        }

        switch (tagid) {
            case 7:
                desiredAngle = 90;
                break;
            case 8:
                desiredAngle = 150;
                break;
            default:
                desiredAngle = 0;
                break;
        }

        addRequirements(drive);
    }

    @Override
    public void execute() {

        if ((drive.getHeading() - desiredAngle) <= tolerance) {
            drive.drive(0.5, 0, 0, 0.3, true);
        }

    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        // probably a better way to do this
        if (hasTarget) {
            return true;
        } else {
            return true;
        }
    }
}
