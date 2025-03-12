package frc.robot.commands.drivetrain;

import java.util.List;
import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTag;

public class SwerveDriveTargetReef extends Command {

    private DriveSubsystem driveSubsystem;
    private boolean isLeft;
    private Pose2d targetPose;
    private double kXYP = 0;
    private double kXYI = 0;
    private double kXYD = 0;
    private double kRotP = 0;
    private double kRotI = 0;
    private double kRotD = 0;
    // x pid, y pid, and rotation pid

    private PIDController xPid, yPid = new PIDController(kXYP, kXYI, kXYD, 0.02);
    private PIDController rotPid = new PIDController(kRotP, kRotI, kRotD, 0.02);

    List<AprilTag> tagPoses = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField().getTags();

    public SwerveDriveTargetReef(DriveSubsystem driveSubsystem, boolean isLeft) {
        this.driveSubsystem = driveSubsystem;
        this.isLeft = isLeft;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = this.driveSubsystem.getPose();
        // an array of the 16 apriltag poses of the reef
        List<Pose2d> reefPoses = new ArrayList<Pose2d>();
        this.tagPoses.forEach(tag -> {
            int id = tag.ID;
            if ((6 <= id && id <= 11) || (17 <= id && id <= 22)) {
                // if the apriltag is a reef pose
                reefPoses.add(tag.pose.toPose2d());
            }
        });

        Pose2d focusPoseAprilTag = currentPose.nearest(reefPoses);

        Pose2d relativePose;
        // if the target is on the left side of the robot
        if (this.isLeft) {
            relativePose = new Pose2d();
        } else {
            relativePose = new Pose2d();
        }

        this.targetPose = new Pose2d(relativePose.getTranslation().plus(focusPoseAprilTag.getTranslation()),
                relativePose.getRotation().plus(focusPoseAprilTag.getRotation()));

        // add the poses together *

    }

    @Override
    public void execute() {
        // run the Pid controllers using the targetpose we generated earlier

        // use the current pose of the robot as the input
        Pose2d currentPose = this.driveSubsystem.getPose();

        double vx = this.xPid.calculate(currentPose.getTranslation().getX(), targetPose.getTranslation().getX());
        double vy = this.yPid.calculate(currentPose.getTranslation().getY(), targetPose.getTranslation().getY());
        double vrot = this.rotPid.calculate(currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians());

        double speedVal = Math.sqrt(vx * vx + vy * vy);
        double forward = vx / speedVal;
        double Right = vy / speedVal;
        double Rotation = vrot;

        this.driveSubsystem.drive(
                speedVal,
                forward,
                Right,
                Rotation,
                true);
    }

    @Override
    public boolean isFinished() {
        // return if the Pid controller is at the target pose
        return this.xPid.atSetpoint() && this.yPid.atSetpoint() && this.rotPid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {

    }

}