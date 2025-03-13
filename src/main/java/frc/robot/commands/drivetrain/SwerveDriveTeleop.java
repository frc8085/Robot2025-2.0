package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.io.Keymap;
import frc.robot.io.Keymap.Controllers;

public class SwerveDriveTeleop extends Command {

    DriveSubsystem driveSubsystem;

    public SwerveDriveTeleop(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double speedVal = Math.pow(Controllers.driverController.getRightTriggerAxis(), 2);

        double invert = this.driveSubsystem.invertForAlliance();

        double leftX = -Controllers.driverController.getLeftX() * invert;
        double leftY = -Controllers.driverController.getLeftY() * invert;
        double rightX = -Math.pow(Controllers.driverController.getRightX(), 3);

        this.driveSubsystem.drive(
                speedVal,
                leftY, // forward-backward
                leftX, // left-right
                rightX, // rotation
                true);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}