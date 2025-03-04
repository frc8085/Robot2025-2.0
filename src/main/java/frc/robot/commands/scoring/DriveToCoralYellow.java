package frc.robot.commands.scoring;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class DriveToCoralYellow extends Command {
    DriveSubsystem drive;
    LimelightSubsystem limelight;
    PIDController xPid; // Moves left and right
    PIDController yPid; // Moves forward and back
    double maxSpeed = 1;

    double[] rhat = { 0, 0 }; // Unit vector in the correct direction.

    // TO BE TUNED:
    // double theta = 0; //Angle of the reef.
    double kPX = 0.03;
    double kIX = 0;
    double kDX = 0;
    double kPY = 0.06;
    double kIY = 0;
    double kDY = 0;
    double tolerance = 1;
    double xTarget = -5.56;
    double yTarget = 12.57;

    public DriveToCoralYellow(DriveSubsystem drive, LimelightSubsystem limelight) {
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
        yPid.setSetpoint(yTarget);
        // X setpoint changes with distance, so we update it in execute
    }

    @Override
    public void execute() {
        double tx = limelight.getX("limelight-right");
        double ty = limelight.getY("limelight-right");

        xTarget = -2.31 * ty + 22.5; // Heuristic equation we found
        xPid.setSetpoint(xTarget);

        double xSpeed = maxSpeed * -xPid.calculate(tx);
        double ySpeed = maxSpeed * yPid.calculate(ty);

        // If we got to the correct x, stop moving in that direction.
        if (xPid.atSetpoint()) {
            xSpeed = 0;
        }

        double speed = Math.hypot(xSpeed, ySpeed);

        if (!limelight.hasTarget("limelight-right")) {
            speed = 0;
        }

        drive.drive(speed, xSpeed, ySpeed, 0, false);

        SmartDashboard.putNumber("X Error", tx - xPid.getSetpoint());
        SmartDashboard.putNumber("y Error", ty - yPid.getSetpoint());

    }

    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, 0, true);
    }

    public boolean isFinished() {
        // TODO: Add a condition that allows the driver/operator to exit this command.
        return ((xPid.atSetpoint() && yPid.atSetpoint()) || targetLost());
    }

    public boolean targetLost() {
        return (!limelight.hasTarget("limelight-right"));
    }

}
