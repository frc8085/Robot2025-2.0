package frc.robot.commands.movement;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Limelight.LimelightSubsystem;

public class AlignToAprilTagYellow extends Command {
    DriveSubsystem drive;
    LimelightSubsystem limelight;
    private boolean lostTarget;
    PIDController turnPID; // Adjusts turn PID
    double maxTurnSpeed = .15;

    double[] rhat = { 0, 0 }; // Unit vector in the correct direction.

    // TO BE TUNED:
    // double theta = 0; //Angle of the reef.
    double kPTurn = .1;
    double kITurn = 0;
    double kDTurn = 0;
    double tolerance = 2;
    double target;

    public AlignToAprilTagYellow(DriveSubsystem drive, LimelightSubsystem limelight) {
        this.drive = drive;
        this.limelight = limelight;

        turnPID = new PIDController(kPTurn, kITurn, kDTurn);
        turnPID.setTolerance(tolerance);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        int id = limelight.getID("limelight-yellow");
        switch (id) {
            case 6:
                id = 30;
                break;
            case 7:
                id = 90;
                break;
            case 8:
                id = 150;
                break;
            case 9:
                id = -150;
                break;
            case 10:
                id = 90;
                break;
            case 11:
                id = -30;
                break;
            case 17:
                id = 150;
                break;
            case 18:
                id = 90;
                break;
            case 19:
                id = 30;
                break;
            case 20:
                id = -30;
                break;
            case 21:
                id = -90;
                break;
            case 22:
                id = -150;
                break;
        }
        target = id;
        turnPID.setSetpoint(target);
        lostTarget = false;

        SmartDashboard.putNumber("target", target);
    }

    // X setpoint changes with distance, so we update it in execute

    @Override
    public void execute() {
        double heading = drive.getHeading();
        double angularSpeed = maxTurnSpeed * turnPID.calculate(heading);

        // If we got to the correct x, stop moving in that direction.
        if (turnPID.atSetpoint()) {
            angularSpeed = 0;
        }

        // if (!limelight.hasTarget("limelight-yellow")) {
        // angularSpeed = 0;
        // lostTarget = true;
        // }

        drive.drive(0, 0, 0, angularSpeed, false);

        SmartDashboard.putNumber("Angular Error", heading - turnPID.getSetpoint());

    }

    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, 0, false);

    }

    public boolean isFinished() {
        // TODO: Add a condition that allows the driver/operator to exit this command.
        return ((drive.getHeading() >= target - tolerance) && (drive.getHeading() <= target + tolerance));
    }

}