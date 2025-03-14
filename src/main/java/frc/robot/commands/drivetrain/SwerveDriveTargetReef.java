package frc.robot.commands.drivetrain;

import java.util.List;
import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;

public class SwerveDriveTargetReef extends Command {

    private DriveSubsystem driveSubsystem;
    private boolean isLeft;
    private Pose2d targetPose;
    private double kXP = .5; // 0.5
    private double kXI = 0;
    private double kXD = 0; // 0.1
    private double kYP = .5; // 0.5
    private double kYI = 0;
    private double kYD = 0; // 0.1
    private double kRotP = 1.5; // 3
    private double kRotI = 0;
    private double kRotD = 0.0; // 0.5

    private PIDController xPid = new PIDController(kXP, kXI, kXD, 0.02);
    private PIDController yPid = new PIDController(kYP, kYI, kYD, 0.02);
    private PIDController rotPid = new PIDController(kRotP, kRotI, kRotD, 0.02);

    List<AprilTag> tagPoses = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField().getTags();

    public SwerveDriveTargetReef(DriveSubsystem driveSubsystem, boolean isLeft) {
        this.rotPid.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));
        this.driveSubsystem = driveSubsystem;
        this.isLeft = isLeft;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Starting Target Reef");
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
            relativePose = AutoConstants.leftReefAlignPose;
        } else {
            relativePose = AutoConstants.rightReefAlignPose;
        }
        // need to rotate the relative pose by the rotation of the focusPoseAprilTag
        Pose2d newRelativePose = relativePose.rotateAround(new Translation2d(), focusPoseAprilTag.getRotation());

        this.targetPose = new Pose2d(newRelativePose.getTranslation().plus(focusPoseAprilTag.getTranslation()),
                newRelativePose.getRotation());

        double[] targetPoseArray = { this.targetPose.getTranslation().getX(), this.targetPose.getTranslation().getY(),
                this.targetPose.getRotation().getRadians() };

        SmartDashboard.putNumberArray("Target Align Pose", targetPoseArray);
        // this.rotPid.reset(currentPose.getRotation().getDegrees());
        this.rotPid.reset();
        this.xPid.reset();
        this.yPid.reset();

        this.yPid.setTolerance(0.05);
    }

    public double boundAngle(double degrees) {
        degrees = degrees % 360;
        if (degrees > 180) {
            degrees -= 360;
        }
        return degrees;
    }

    @Override
    public void execute() {
        // run the Pid controllers using the targetpose we generated earlier

        // use the current pose of the robot as the input
        Pose2d currentPose = this.driveSubsystem.getPose();

        double currentRotation = this.boundAngle(currentPose.getRotation().getDegrees());
        double targetRotation = this.boundAngle(this.targetPose.getRotation().getDegrees());

        double vx = this.xPid.calculate(currentPose.getTranslation().getX(), targetPose.getTranslation().getX());
        double vy = this.yPid.calculate(currentPose.getTranslation().getY(),
                targetPose.getTranslation().getY());
        double vrot = this.rotPid.calculate(Math.toRadians(currentRotation), Math.toRadians(targetRotation));
        double speedVal = Math.sqrt(vx * vx + vy * vy);
        // double forward = vx / speedVal;
        double forward = Math.pow(vx, 3);
        // double right = vy / speedVal;
        double right = Math.pow(vy, 3);
        double rotation = vrot;

        // double rotError = Math.abs(this.rotPid.getPositionError());
        this.driveSubsystem.drive(
                speedVal,
                forward,
                right,
                rotation,
                true);
    }

    @Override
    public boolean isFinished() {
        // return if the Pid controller is at the target pose
        return this.xPid.atSetpoint() && this.yPid.atSetpoint() && this.rotPid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Target Reef Completed");
    }

}