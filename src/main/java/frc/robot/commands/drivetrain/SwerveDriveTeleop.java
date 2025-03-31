package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.io.Keymap.Controllers;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class SwerveDriveTeleop extends Command {

    DriveSubsystem driveSubsystem;

    public SwerveDriveTeleop(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Normal Drive Started");
    }

    @Override
    public void execute() {
        double speedVal = MathUtil.applyDeadband(Math.pow(Controllers.driverController.getRightTriggerAxis(), 2), 0);

        double invert = this.driveSubsystem.invertForAlliance();

        double leftX = MathUtil.applyDeadband(-Controllers.driverController.getLeftX() * invert,
                OIConstants.kDriveDeadband);
        double leftY = MathUtil.applyDeadband(-Controllers.driverController.getLeftY() * invert,
                OIConstants.kDriveDeadband);
        double rightX = MathUtil.applyDeadband(-Math.pow(Controllers.driverController.getRightX(), 3),
                OIConstants.kTurnDeadband);

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