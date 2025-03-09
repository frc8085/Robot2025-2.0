package frc.robot.commands.movement;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveFieldRelative extends Command {
    DriveSubsystem drive;

    public DriveFieldRelative(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.drive(0, 0, 0, 0, true);
    }

    @Override
    public void execute() {

    }

    public void end(boolean interrupted) {
    }

    public boolean isFinished() {
        // TODO: Add a condition that allows the driver/operator to exit this command.
        return true;
    }

}