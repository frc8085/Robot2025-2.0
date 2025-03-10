package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.io.Keymap;

// driveSubsystem.drive(
//                                                                 MathUtil.applyDeadband(
//                                                                                 Math.pow(driverController
//                                                                                                 .getRightTriggerAxis(),
//                                                                                                 2),
//                                                                                 0),
//                                                                 -MathUtil.applyDeadband(driverController.getLeftY(),
//                                                                                 OIConstants.kDriveDeadband),
//                                                                 -MathUtil.applyDeadband(driverController.getLeftX(),
//                                                                                 OIConstants.kDriveDeadband),
//                                                                 -MathUtil.applyDeadband(
//                                                                                 Math.pow(driverController.getRightX(),
//                                                                                                 3),
//                                                                                 OIConstants.kTurnDeadband),
//                                                                 FakeConstants.fieldRelative)

public class SwerveDriveTeleop extends Command {

    DriveSubsystem driveSubsystem;

    Keymap keymap = new Keymap();

    SwerveDriveTeleop(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        double triggerVal = Math.pow(Keymap.layout, 2);

        this.driveSubsystem.drive(
                Math.pow(Keymap.layout.driveLeftX, 2),
                -Keymap.driverController.getLeftY(),
                -Keymap.driverController.getLeftX(),
                -Math.pow(Keymap.driverController.getRightX(), 3),
                true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}