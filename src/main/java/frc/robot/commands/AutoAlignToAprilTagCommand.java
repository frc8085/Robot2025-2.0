package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightReefSubsystem;

/**
 * AutoAlignToAprilTagCommand uses vision data from the LimelightReefSubsystem
 * to align the robot with an AprilTag target by applying both rotation and translation.
 */
public class AutoAlignToAprilTagCommand extends Command {

    private final DriveSubsystem driveSubsystem;
    private final LimelightReefSubsystem limelightSubsystem;

    // Tuning constants (adjust these as needed)
    private final double kDesiredArea = 2.0;       // Desired target area when at the correct distance
    private final double kAreaTolerance = 0.1;       // Acceptable error in target area (translation)
    private final double kTxTolerance = 1.0;         // Acceptable horizontal offset (degrees) (rotation)

    private final double kP_translation = 0.5;       // Proportional gain for translation speed
    private final double kP_rotation = 0.02;         // Proportional gain for rotation correction

    /**
     * Constructs a new AutoAlignToAprilTagCommand.
     *
     * @param driveSubsystem     the drivetrain subsystem.
     * @param limelightSubsystem the vision subsystem providing fiducial data.
     */
    public AutoAlignToAprilTagCommand(DriveSubsystem driveSubsystem, LimelightReefSubsystem limelightSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        addRequirements(driveSubsystem, limelightSubsystem);
    }

    @Override
    public void initialize() {
        // Optionally reset sensors or configure vision settings if needed.
    }

    @Override
    public void execute() {
        // If no valid target is found, stop all motion.
        if (!limelightSubsystem.getTV()) {
            driveSubsystem.drive(0, 0, 0, 0, false);
            return;
        }

        // Retrieve vision data from the closest fiducial.
        double xError = limelightSubsystem.getClosestTX();       // Horizontal error (in degrees)
        double currentArea = limelightSubsystem.getClosestTA();    // Current target area

        // Compute the translation error (positive error means the target is too small/far)
        double areaError = kDesiredArea - currentArea;
        double translationSpeed = kP_translation * areaError;      // Forward/backward speed (can be positive or negative)

        // Compute the rotational correction (negative sign to steer in the proper direction)
        double rotationCorrection = -kP_rotation * xError;

        // Command the drive subsystem:
        // We drive along the robot’s forward axis (xSpeed = 1.0, ySpeed = 0.0) and apply rotation.
        // The drive() method multiplies the first parameter (translationSpeed) by the max speed.
        driveSubsystem.drive(translationSpeed, 1.0, 0.0, rotationCorrection, false);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends or is interrupted.
        driveSubsystem.drive(0, 0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        // If no target is seen, we don't finish (or you might choose to cancel).
        if (!limelightSubsystem.getTV()) {
            return false;
        }
        double xError = limelightSubsystem.getClosestTX();
        double currentArea = limelightSubsystem.getClosestTA();
        double areaError = Math.abs(kDesiredArea - currentArea);

        // Finish if both the rotation and translation errors are within acceptable tolerances.
        return (Math.abs(xError) < kTxTolerance) && (areaError < kAreaTolerance);
    }
}
