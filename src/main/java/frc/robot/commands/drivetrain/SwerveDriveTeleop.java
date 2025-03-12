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

        double leftX = Controllers.driverController.getLeftX();
        double leftY = -Controllers.driverController.getLeftY();
        double rightX = -Math.pow(Controllers.driverController.getRightX(), 3);

        this.driveSubsystem.drive(
                speedVal,
                leftX,
                leftY,
                rightX,
                true);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}