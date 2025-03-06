package frc.robot.commands;

import java.lang.annotation.Target;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AlignHeading extends Command {
    private final DriveSubsystem drive;
    private final LimelightSubsystem limelight;

    private final double xOffset;

    private final double maxSpeed = 0.5;
    private final double maxRotationSpeed = 0.4;

    private final double base = 0; // tune
    private final double thetaFinal = 25;
    private final double tolerance = 3;

    public boolean hasTarget = true;

    public AlignHeading(DriveSubsystem drive, LimelightSubsystem limelight) {
        this.drive = drive;
        this.limelight = limelight;

        xOffset = limelight.getX("limelight-left");

        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (!limelight.hasTarget("limelight-left")) {
            hasTarget = false;
            return;
        }

        // Retrieve AprilTag pose data

        double xAbs = Math.abs(xOffset);
        double baseAbs = xAbs * base;

        double thetaInitial = Math.atan(xAbs / baseAbs);
        if ((thetaInitial - thetaFinal) < tolerance) {
            if (xOffset < 0) {
                drive.drive(maxSpeed, 0, 0, -0.3, false);
            } else {
                drive.drive(maxSpeed, 0, 0, 0.3, false);
            }
        } else {
            return;
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
