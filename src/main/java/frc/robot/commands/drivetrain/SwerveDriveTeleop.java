package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.io.Keymap;

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

        double speedVal = Math.pow(Keymap.Layout.driverRightTrigger, 2);


        double leftX = -Keymap.Layout.driverLeftX;
        double leftY = -Keymap.Layout.driverLeftY;
        double rightX = -Math.pow(Keymap.Layout.driverRightX, 3);

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