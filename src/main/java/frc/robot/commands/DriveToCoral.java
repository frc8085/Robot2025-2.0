package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class DriveToCoral extends Command {
    DriveSubsystem drive;
    LimelightSubsystem limelight;
    PIDController xPid; // Moves left and right
    PIDController yPid; // Moves forward and back
    double maxSpeed = 0.5;

    double[] rhat = { 0, 0 }; // Unit vector in the correct direction.

    // TO BE TUNED:
    // double theta = 0; //Angle of the reef.
    double kPX = 20;
    double kIX = 0;
    double kDX = 0;
    double kPY = 50;
    double kIY = 0;
    double kDY = 0;
    double tolerance = 0.01;
    double xTarget = -11;
    double yTarget = 7.46;

    public DriveToCoral(DriveSubsystem drive, LimelightSubsystem limelight) {
        this.drive = drive;
        this.limelight = limelight;

        xPid = new PIDController(kPX, kIX, kDX);
        xPid.setTolerance(tolerance);
        yPid = new PIDController(kPY, kIY, kDY);
        yPid.setTolerance(tolerance);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        double ty = limelight.getY("limelight-left");
        xTarget += -3.67 * ty + 9.06; // Heuristic equation we found
        xPid.setSetpoint(xTarget);
        yPid.setSetpoint(yTarget);
    }

    @Override
    public void execute() {

        double tx = limelight.getX("limelight-left");
        double ty = limelight.getY("limelight-left");

        double xSpeed = maxSpeed * -xPid.calculate(tx);
        // double ySpeed = maxSpeed * yPid.calculate(ty);
        double ySpeed = 0;
        double speed = Math.hypot(xSpeed, ySpeed);

        if (!limelight.hasTarget("limelight-left")) {
            speed = 0;
        }

        drive.drive(speed, xSpeed, ySpeed, 0, false);
    }

    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, 0, true);
    }

    public boolean isFinished() {
        return xPid.atSetpoint() && yPid.atSetpoint();
    }

}