package frc.robot.commands.movement;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AngleToBarge extends Command {
    DriveSubsystem drive;
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

    public AngleToBarge(DriveSubsystem drive) {
        this.drive = drive;

        turnPID = new PIDController(kPTurn, kITurn, kDTurn);
        turnPID.setTolerance(tolerance);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        target = 30;
        turnPID.setSetpoint(target);
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
        drive.drive(0, 0, 0, 0, true);
    }

    public boolean isFinished() {
        // TODO: Add a condition that allows the driver/operator to exit this command.
        return ((drive.getHeading() >= target - tolerance) && (drive.getHeading() <= target + tolerance));
    }

}