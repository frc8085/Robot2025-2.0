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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;

public class SwerveDriveAlignBarge extends Command {

    private DriveSubsystem driveSubsystem;
    private Pose2d targetPose;
    private double kXYP = 0.35;
    private double kXYI = 0;
    private double kXYD = 0;
    private double kRotP = 0.01;
    private double kRotI = 0;
    private double kRotD = 0;
    // x pid, y pid, and rotation pid

    private PIDController xPid = new PIDController(kXYP, kXYI, kXYD, 0.02);
    private PIDController yPid = new PIDController(kXYP, kXYI, kXYD, 0.02);
    private PIDController rotPid = new PIDController(kRotP, kRotI, kRotD, 0.02);

    public SwerveDriveAlignBarge(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // get the current pose of the robot
        Pose2d currentPose = this.driveSubsystem.getPose();
        // Enter in the target X -- this should be the X of the current pose
        double targetX = 0;
        // This is the final rotation we want to be at
        Rotation2d targetRot = Rotation2d.fromDegrees(30);
        // We also want to go to a fixed Y - how does it work for each field side
        // (blue/red)
        this.targetPose = new Pose2d(targetX, currentPose.getTranslation().getY(), targetRot);

        double[] targetPoseArray = { this.targetPose.getTranslation().getX(), this.targetPose.getTranslation().getY(),
                this.targetPose.getRotation().getRadians() };

        SmartDashboard.putNumberArray("Target Align Pose", targetPoseArray);
        this.rotPid.reset();
        this.xPid.reset();
        this.yPid.reset();
        this.rotPid.enableContinuousInput(-180, 180);

    }

    @Override
    public void execute() {
        // run the Pid controllers using the targetpose we generated earlier

        // use the current pose of the robot as the input
        Pose2d currentPose = this.driveSubsystem.getPose();

        double vx = this.xPid.calculate(currentPose.getTranslation().getX(), targetPose.getTranslation().getX());
        // double vy = this.yPid.calculate(currentPose.getTranslation().getY(),
        // targetPose.getTranslation().getY());
        double vrot = this.rotPid.calculate(currentPose.getRotation().getDegrees(),
                targetPose.getRotation().getDegrees());

        // double speedVal = Math.sqrt(vx * vx + vy * vy);
        double speedVal = Math.abs(vx);
        // double forward = -vy / speedVal;
        double forward = 0;
        double right = vx / speedVal;
        double rotation = vrot;

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

    }

}