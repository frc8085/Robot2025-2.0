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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;
import edu.wpi.first.math.MathUtil;
import frc.robot.io.Keymap.Controllers;
import frc.robot.Constants.OIConstants;

public class SwerveDriveTargetReef extends Command {

    private DriveSubsystem driveSubsystem;
    private boolean isLeft;
    private Pose2d targetPose;
    private double kXP = 0.75; // 0.5
    private double kXI = 0;
    private double kXD = 0.1; // 0.1
    private double kYP = 0.75; // 0.5
    private double kYI = 0;
    private double kYD = 0.1; // 0.1
    private double kRotP = 1.7; // 3
    private double kRotI = 0;
    private double kRotD = 0.25; // 0.5

    private PIDController xPid = new PIDController(kXP, kXI, kXD, 0.02);
    private PIDController yPid = new PIDController(kYP, kYI, kYD, 0.02);
    private PIDController rotPid = new PIDController(kRotP, kRotI, kRotD, 0.02);

    List<AprilTag> tagPoses = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField().getTags();
    List<Pose2d> reefPoses = new ArrayList<Pose2d>();

    public SwerveDriveTargetReef(DriveSubsystem driveSubsystem, boolean isLeft) {
        this.driveSubsystem = driveSubsystem;
        this.isLeft = isLeft;

        this.tagPoses.forEach(tag -> {
            int id = tag.ID;
            if ((6 <= id && id <= 11) || (17 <= id && id <= 22)) {
                // if the apriltag is a reef pose
                this.reefPoses.add(tag.pose.toPose2d());
            }
        });

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Starting Target Reef");
        Pose2d currentPose = this.driveSubsystem.getPose();

        Pose2d focusPoseAprilTag = currentPose.nearest(reefPoses);

        Pose2d relativePose;

        // driver input whether left or right reef
        if (this.isLeft) {
            relativePose = AutoConstants.leftReefAlignPose;
        } else {
            relativePose = AutoConstants.rightReefAlignPose;
        }

        // need to rotate the relative pose by the rotation of the focusPoseAprilTag
        Pose2d newRelativePose = relativePose.rotateAround(new Translation2d(), focusPoseAprilTag.getRotation());

        Rotation2d rotationBlue = AutoConstants.rotationBlue.rotateBy(focusPoseAprilTag.getRotation());
        Rotation2d rotationYellow = AutoConstants.rotationYellow.rotateBy(focusPoseAprilTag.getRotation());

        // rotate left or right depending on the closer angle
        double currentAngle = this.boundAngle(currentPose.getRotation().getDegrees());
        double angleBlue = this.boundAngle(rotationBlue.getDegrees());
        double angleYellow = this.boundAngle(rotationYellow.getDegrees());

        // TODO: fix rotation yellow, currently set to blue only
        newRelativePose = new Pose2d(newRelativePose.getTranslation(), rotationBlue);

        // if (Math.abs(currentAngle - angleBlue) < Math.abs(currentAngle -
        // angleYellow)) {
        // newRelativePose = new Pose2d(newRelativePose.getTranslation(), rotationBlue);
        // } else {
        // newRelativePose = new Pose2d(newRelativePose.getTranslation(),
        // rotationYellow);
        // }

        this.targetPose = new Pose2d(newRelativePose.getTranslation().plus(focusPoseAprilTag.getTranslation()),
                newRelativePose.getRotation());

        double[] targetPoseArray = { this.targetPose.getTranslation().getX(), this.targetPose.getTranslation().getY(),
                this.targetPose.getRotation().getRadians() };

        SmartDashboard.putNumberArray("Target Align Pose", targetPoseArray);
        // this.rotPid.reset(currentPose.getRotation().getDegrees());
        // this.rotPid.reset();
        this.xPid.reset();
        this.yPid.reset();
        this.rotPid.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));

        this.yPid.setTolerance(0.05);
        this.rotPid.setTolerance(0.5);
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
        double vrot = this.rotPid.calculate(Math.toRadians(currentRotation),
                Math.toRadians(targetRotation));
        double speedVal = Math.sqrt(vx * vx + vy * vy);
        // double forward = vx / speedVal;

        double forward = 0;
        double right = 0;
        if (Math.abs(this.rotPid.getError()) < 7) {
            forward = vx;
            // double right = vy / speedVal;
            right = vy;
        }

        double rotation = vrot;
        // double rotation = 0;

        // user enabled rotation
        // double rotation =
        // MathUtil.applyDeadband(-Math.pow(Controllers.driverController.getRightX(),
        // 3),
        // OIConstants.kTurnDeadband);

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
        return this.xPid.atSetpoint() && this.yPid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Target Reef Completed");
    }

}