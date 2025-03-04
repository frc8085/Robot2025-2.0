package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants.Direction;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Command for moving the robot using the swerve drive modules and an x-box
 * controller
 */
public class MoveCommand extends Command {
        private CommandXboxController driverController;
        private DriveSubsystem driveSubsystem;
        private boolean fieldRelative = true;

        /**
         * Command for moving the robot using the swerve drive modules and an x-box
         * controller
         * D-PAD controls the robot at 20% speed in robot-oriented mode
         * Left Joystick controls the robot at variable speeds (0-100%) in
         * field-oriented mode
         * - Cubes driver input for greater control at low speeds
         * Right Joystick controls the robot orientation / heading (direction it is
         * facing)
         * Right Trigger + A/B/X/Y -> Robot automically orients to the corresponding
         * field-oriented heading
         * - Works while moving and continues to orient until button is released
         * 
         * @param drivetrain
         *                         The drive subsystem.
         * @param driverController
         *                         The xbox controller for the robot
         */
        public MoveCommand(DriveSubsystem drivetrain, CommandXboxController driverController) {
                this.driveSubsystem = drivetrain;
                this.driverController = driverController;
                addRequirements(this.driveSubsystem);
        }

        @Override
        public void execute() {
                // Trigger mappings for fine-tuned robot-oriented adjustments using the d-pad
                if (driverController.povLeft().getAsBoolean()) {
                        driveSubsystem.drive(1, 0, .2,
                                        -MathUtil.applyDeadband(driverController.getRightX(),
                                                        OIConstants.kDriveDeadband),
                                        false);
                } else if (driverController.povRight().getAsBoolean()) {
                        driveSubsystem.drive(1, 0, -.2,
                                        -MathUtil.applyDeadband(driverController.getRightX(),
                                                        OIConstants.kDriveDeadband),
                                        false);
                } else if (driverController.povUp().getAsBoolean()) {
                        driveSubsystem.drive(1, .2, 0,
                                        -MathUtil.applyDeadband(driverController.getRightX(),
                                                        OIConstants.kDriveDeadband),
                                        false);
                } else if (driverController.povDown().getAsBoolean()) {
                        driveSubsystem.drive(1, -0.2, 0,
                                        -MathUtil.applyDeadband(driverController.getRightX(),
                                                        OIConstants.kDriveDeadband),
                                        false);
                } else if (driverController.povUpLeft().getAsBoolean()) {
                        driveSubsystem.drive(1, .14, .14,
                                        -MathUtil.applyDeadband(driverController.getRightX(),
                                                        OIConstants.kDriveDeadband),
                                        false);
                } else if (driverController.povUpRight().getAsBoolean()) {
                        driveSubsystem.drive(1, .14, -.14,
                                        -MathUtil.applyDeadband(driverController.getRightX(),
                                                        OIConstants.kDriveDeadband),
                                        false);
                } else if (driverController.povDownLeft().getAsBoolean()) {
                        driveSubsystem.drive(1, -.14, .14,
                                        -MathUtil.applyDeadband(driverController.getRightX(),
                                                        OIConstants.kDriveDeadband),
                                        false);
                } else if (driverController.povDownRight().getAsBoolean()) {
                        driveSubsystem.drive(1, -.14, -.14,
                                        -MathUtil.applyDeadband(driverController.getRightX(),
                                                        OIConstants.kDriveDeadband),
                                        false);
                } else { // Joystick / field-oriented based movement
                        double yMovement = driverController.getLeftY();
                        double xMovement = driverController.getLeftX();
                }
                // Trigger mappings for field oriented driving while automically orienting to a
                // supplied direction
                if (driverController.leftTrigger().getAsBoolean() &&
                                driverController.x().getAsBoolean()) {
                        this.driveSubsystem.driveAndOrient(
                                        MathUtil.applyDeadband(
                                                        Math.pow(driverController
                                                                        .getRightTriggerAxis(),
                                                                        2),
                                                        OIConstants.kDriveDeadband),

                                        -MathUtil.applyDeadband(driverController.getLeftY(),
                                                        OIConstants.kDriveDeadband),
                                        -MathUtil.applyDeadband(driverController.getLeftX(),
                                                        OIConstants.kDriveDeadband),
                                        Direction.RIGHT);
                } else if (driverController.leftTrigger().getAsBoolean()
                                && driverController.y().getAsBoolean()) {
                        this.driveSubsystem.driveAndOrient(
                                        MathUtil.applyDeadband(
                                                        Math.pow(driverController
                                                                        .getRightTriggerAxis(),
                                                                        2),
                                                        OIConstants.kDriveDeadband),

                                        -MathUtil.applyDeadband(driverController.getLeftY(),
                                                        OIConstants.kDriveDeadband),
                                        -MathUtil.applyDeadband(driverController.getLeftX(),
                                                        OIConstants.kDriveDeadband),
                                        Direction.BACKWARD);
                } else if (driverController.leftTrigger().getAsBoolean()
                                && driverController.b().getAsBoolean()) {
                        this.driveSubsystem.driveAndOrient(
                                        MathUtil.applyDeadband(
                                                        Math.pow(driverController
                                                                        .getRightTriggerAxis(),
                                                                        2),
                                                        OIConstants.kDriveDeadband),

                                        -MathUtil.applyDeadband(driverController.getLeftY(),
                                                        OIConstants.kDriveDeadband),
                                        -MathUtil.applyDeadband(driverController.getLeftX(),
                                                        OIConstants.kDriveDeadband),
                                        Direction.LEFT);
                } else if (driverController.leftTrigger().getAsBoolean()
                                && driverController.a().getAsBoolean()) {
                        this.driveSubsystem.driveAndOrient(
                                        MathUtil.applyDeadband(
                                                        Math.pow(driverController
                                                                        .getRightTriggerAxis(),
                                                                        2),
                                                        OIConstants.kDriveDeadband),
                                        -MathUtil.applyDeadband(driverController.getLeftY(),
                                                        OIConstants.kDriveDeadband),
                                        -MathUtil.applyDeadband(driverController.getLeftX(),
                                                        OIConstants.kDriveDeadband),
                                        Direction.FORWARD);
                } else {
                        // Default joystick controlled swerve
                        // The left stick controls translation of the robot.
                        // Turning is controlled by the X axis of the right stick.
                        this.driveSubsystem.drive(
                                        MathUtil.applyDeadband(
                                                        Math.pow(driverController
                                                                        .getRightTriggerAxis(),
                                                                        2),
                                                        OIConstants.kDriveDeadband),

                                        -MathUtil.applyDeadband(driverController.getLeftY(),
                                                        OIConstants.kDriveDeadband),
                                        -MathUtil.applyDeadband(driverController.getLeftX(),
                                                        OIConstants.kDriveDeadband),
                                        -MathUtil.applyDeadband(
                                                        Math.pow(driverController.getRightX(),
                                                                        3),
                                                        OIConstants.kDriveDeadband),
                                        fieldRelative);
                }
        }

        @Override
        public boolean isFinished() {
                return false;
        }

        /*
         * getFieldRelative - returns the robot drive mode (field or robot oriented)
         */
        public boolean getFieldRelative() {
                return this.fieldRelative;
        }

        /*
         * toggleFieldRelative - toggles the drive mode of the robot
         * 
         * Can be used to dynamically switch between robot-oriented and field-oriented
         * modes
         */
        public void toggleFieldRelative() {
                this.fieldRelative = !this.fieldRelative;
        }

        /*
         * setFieldRelative - sets the drive mode of the robot
         * 
         * Can be used to dynamically set field-oriented or robot oriented drive modes
         */
        public void setFieldRelative(boolean fieldRelative) {
                this.fieldRelative = fieldRelative;
        }
}
