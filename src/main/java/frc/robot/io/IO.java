package frc.robot.io;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.jni.WPIMathJNI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.windmill.*;
// import frc.robot.commands.scoring.*;
// import frc.robot.commands.sequences.RemoveAlgaeL2;
// import frc.robot.commands.sequences.RemoveAlgaeL3;
// import frc.robot.commands.manipulator.coral.*;
// import frc.robot.commands.manipulator.algae.*;
// import frc.robot.commands.movement.*;
// import frc.robot.commands.states.*;
import frc.robot.commands.climber.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.endEffector.*;
import frc.robot.commands.windmill.elevator.*;
import frc.robot.commands.windmill.pivot.Pivot;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.Constants.Windmill.WindmillState;

import frc.robot.commands.states.ToCoralDropOff;
import frc.robot.commands.states.ToHomeCommand;
import frc.robot.commands.states.DeAlgaeReef2;
import frc.robot.commands.states.DeAlgaeReef3;
import frc.robot.commands.states.ScoreReef;
import frc.robot.commands.states.TestHandoff;

public class IO {

        public void init(RobotContainer robotContainer) {

                // Additional Buttons to allow for alternate button pushes
                // final Trigger altButtonDriver = Keymap.Layout.driverRightBumper;
                final Trigger scoreLeft = Keymap.Layout.operatorLeftBumper;
                final Trigger scoreRight = Keymap.Layout.operatorRightBumper;

                // Initialization
                final Trigger zeroHeadingButton = Keymap.Layout.driverStartButton;

                // final Trigger zeroElevator = operatorController.start();
                final Trigger zeroElevator = Keymap.Layout.operatorStartButton;
                final Trigger limelightTrigger1 = Keymap.Layout.driverXButton;
                final Trigger limelightTrigger2 = Keymap.Layout.driverBButton;

                // Driver operations
                final Trigger scoreCoral = Keymap.Layout.driverAButton;
                final Trigger pickupCoral = Keymap.Layout.operatorDownButton;
                final Trigger ejectAlgae = Keymap.Layout.driverYButton;
                final Trigger lockWheels = Keymap.Layout.driverXButton;
                final Trigger shootAlgaeNetBlue = Keymap.Layout.driverLeftBumper;
                // final Trigger left = Keymap.Layout.driverDownButton;
                // final Trigger right = Keymap.Layout.driverUpButton;
                final Trigger gorobotrelative = Keymap.Controllers.driverController.leftStick();
                final Trigger raiseClimber = Keymap.Layout.driverRightButton;
                final Trigger lowerClimber = Keymap.Layout.driverLeftButton;
                final Trigger goSlow = Keymap.Layout.driverBackButton;

                // Operator Controls
                final Trigger intakeCoral = Keymap.Layout.operatorRightTriggerButton;
                final Trigger dumpCoral = Keymap.Layout.driverLeftTriggerButton;
                final Trigger altdumpCoral = Keymap.Layout.operatorLeftTriggerButton;
                final Trigger toggleClimber = Keymap.Layout.operatorBackButton;

                // Operator Set Position Controls
                // final Trigger algaeGround = Keymap.Layout.operatorDownButton;
                final Trigger algaeReef2 = Keymap.Layout.operatorRightButton;
                final Trigger coralHandOff = Keymap.Layout.operatorUpButton;
                // final Trigger coralEject = Keymap.Layout.operatorDownButton;
                // final Trigger algaeProcessor = Keymap.Layout.operatorLeftButton;
                final Trigger coralDropOff4 = Keymap.Layout.operatorYButton;
                final Trigger coralDropOff3 = Keymap.Layout.operatorXButton;
                final Trigger coralDropOff2 = Keymap.Layout.operatorBButton;
                final Trigger coralDropOff1 = Keymap.Layout.operatorAButton;
                final Trigger algaeReef3 = Keymap.Layout.operatorLeftButton;
                // final Trigger intakeHalf = Keymap.Layout.operatorLeftButton;

                // Set Left Joystick for manual elevator/pivot movement
                final Trigger raiseElevator = Keymap.Controllers.operatorController.axisLessThan(1, -0.25);
                final Trigger lowerElevator = Keymap.Controllers.operatorController.axisGreaterThan(1, 0.25);
                final Trigger pivotClockwise = Keymap.Controllers.operatorController.axisGreaterThan(4, 0.25);
                final Trigger pivotCounterClockwise = Keymap.Controllers.operatorController.axisLessThan(4, -0.25);

                // Initialization
                // Zero elevator - carriage must be below stage 1 or it will zero where it is
                zeroElevator.onTrue(new ParallelCommandGroup(new ZeroElevator(robotContainer.elevator),
                                new InstantCommand(robotContainer.intake::zeroIntake)));
                // zeroArm.onTrue(new ZeroElevator(robotContainer.elevator));
                // zeroArm.and(altButtonOperator)
                // .onTrue(new InitializePivotAndElevator(robotContainer.pivot,
                // robotContainer.elevator));

                // Reset heading of robot for field relative drive
                zeroHeadingButton.onTrue(new InstantCommand(() -> robotContainer.drivetrain.zeroHeading(),
                                robotContainer.drivetrain));

                // // Limelight Buttons

                limelightTrigger1.onTrue(
                                new SwerveDriveTargetReef(robotContainer.drivetrain, true)).onFalse(
                                                new SwerveDriveTeleop(robotContainer.drivetrain));
                limelightTrigger2.onTrue(
                                new SwerveDriveTargetReef(robotContainer.drivetrain, false)).onFalse(
                                                new SwerveDriveTeleop(robotContainer.drivetrain));

                // gorobotrelative.onTrue(new InstantCommand(() -> {
                // if (FakeConstants.fieldRelative) {
                // FakeConstants.fieldRelative = false;
                // } else {
                // FakeConstants.fieldRelative = true;
                // }
                // }));

                // goSlow.toggleOnTrue(new
                // SwerveDriveTeleopRoboRelativeSlow(robotContainer.drivetrain)).toggleOnFalse(
                // new SwerveDriveTeleop(robotContainer.drivetrain));

                // intakeHalf.onTrue(
                // new InstantCommand(() -> robotContainer.intake
                // .setDeployRotation(Rotation2d.fromRotations(18)),
                // robotContainer.intake));
                intakeCoral.onTrue(new SequentialCommandGroup(
                                new Windmill(robotContainer.elevator, robotContainer.pivot,
                                                WindmillState.Home,
                                                false),
                                new InstantCommand(() -> robotContainer.endEffector.stop()),
                                new PickupCoral(robotContainer.intake))
                // .andThen(new Handoff(robotContainer.intake, robotContainer.endEffector))
                ).onFalse(new RetractIntake(robotContainer.intake)
                // .andThen(new Handoff(robotContainer.intake, robotContainer.endEffector))
                );

                pickupCoral.whileTrue(new RunCommand(robotContainer.endEffector::pickup))
                                .onFalse(new InstantCommand(robotContainer.endEffector::stop));
                dumpCoral.onTrue(new DumpCoral(robotContainer.intake))
                                .onFalse(new RetractIntake(robotContainer.intake));
                altdumpCoral.onTrue(new DumpCoral(robotContainer.intake))
                                .onFalse(new RetractIntake(robotContainer.intake));

                // coralDropOff1.onTrue(new Pivot(robotContainer.pivot,
                // Rotation2d.fromDegrees(0)));
                // coralDropOff2.onTrue(new Pivot(robotContainer.pivot,
                // Rotation2d.fromDegrees(90)));
                // coralDropOff3.onTrue(new Pivot(robotContainer.pivot,
                // Rotation2d.fromDegrees(180)));
                // coralDropOff4.onTrue(new Pivot(robotContainer.pivot,
                // Rotation2d.fromDegrees(270)));

                // coralDropOff1.and(scoreLeft).onTrue(
                // new ToCoralDropOff(robotContainer.elevator, robotContainer.pivot,
                // robotContainer.intake,
                // robotContainer.endEffector, WindmillState.CoralDropOff1, false))
                // .onFalse(new Windmill(robotContainer.elevator, robotContainer.pivot,
                // WindmillState.Home, false));

                // scoreCoral.and(coralDropOff1).and(scoreLeft)
                // .onTrue(new ScoreReef(robotContainer.elevator, robotContainer.pivot,
                // robotContainer.endEffector, ScoreReef.ReefLevel.One, false));

                // coralDropOff1.and(scoreRight).onTrue(
                // new ToCoralDropOff(robotContainer.elevator, robotContainer.pivot,
                // robotContainer.intake,
                // robotContainer.endEffector, WindmillState.CoralDropOff1, true))
                // .onFalse(new Windmill(robotContainer.elevator, robotContainer.pivot,
                // WindmillState.Home, false));

                // scoreCoral.and(coralDropOff1).and(scoreRight)
                // .onTrue(new ScoreReef(robotContainer.elevator, robotContainer.pivot,
                // robotContainer.endEffector, ScoreReef.ReefLevel.One, true));

                scoreCoral.onTrue(new ScoreReef(
                                robotContainer.elevator, robotContainer.pivot, robotContainer.endEffector,
                                robotContainer.intake))
                                .onFalse(new ConditionalCommand(
                                                new SequentialCommandGroup(new WaitCommand(.5),
                                                                new Windmill(robotContainer.elevator,
                                                                                robotContainer.pivot,
                                                                                WindmillState.CoralScoreHome, true)),
                                                new SequentialCommandGroup(new WaitCommand(.5),
                                                                new Windmill(robotContainer.elevator,
                                                                                robotContainer.pivot,
                                                                                WindmillState.CoralScoreHome, true)),
                                                robotContainer.pivot::reefMirrored).andThen(new WaitCommand(.25))
                                                .andThen(new ParallelCommandGroup(
                                                                new InstantCommand(robotContainer.endEffector::stop),
                                                                new Windmill(robotContainer.elevator,
                                                                                robotContainer.pivot,
                                                                                WindmillState.Home, false))));

                coralDropOff2.and(scoreLeft).onTrue(new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                                new InstantCommand(
                                                                () -> robotContainer.intake.setDeployRotation(
                                                                                IntakeConstants.kIntakeScoreAngle)),
                                                new Elevator(robotContainer.elevator,
                                                                ElevatorConstants.kElevatorCoralDropOff2Height)),
                                new WaitUntilCommand(() -> robotContainer.elevator
                                                .elevatorAtCoralDropOff2Height()),
                                new ToCoralDropOff(robotContainer.elevator, robotContainer.pivot,
                                                robotContainer.intake, robotContainer.endEffector,
                                                WindmillState.CoralLeftDropOff2, false)));
                // .onFalse(new Windmill(robotContainer.elevator, robotContainer.pivot,
                // WindmillState.Home, false));

                coralDropOff2.and(scoreRight).onTrue(new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                                new InstantCommand(
                                                                () -> robotContainer.intake.setDeployRotation(
                                                                                IntakeConstants.kIntakeScoreAngle)),
                                                new Elevator(robotContainer.elevator,
                                                                ElevatorConstants.kElevatorCoralDropOff2Height)),
                                new WaitUntilCommand(() -> robotContainer.elevator
                                                .elevatorAtCoralDropOff2Height()),
                                new ToCoralDropOff(robotContainer.elevator, robotContainer.pivot,
                                                robotContainer.intake, robotContainer.endEffector,
                                                WindmillState.CoralDropOff2, true)));
                // .onFalse(new Windmill(robotContainer.elevator, robotContainer.pivot,
                // WindmillState.Home, false));

                coralDropOff3.and(scoreLeft).onTrue(new ToCoralDropOff(robotContainer.elevator, robotContainer.pivot,
                                robotContainer.intake, robotContainer.endEffector,
                                WindmillState.CoralLeftDropOff3, false));
                // .onFalse(new Windmill(robotContainer.elevator, robotContainer.pivot,
                // WindmillState.Home, false));

                coralDropOff3.and(scoreRight).onTrue(new ToCoralDropOff(robotContainer.elevator, robotContainer.pivot,
                                robotContainer.intake, robotContainer.endEffector, WindmillState.CoralDropOff3, true));
                // .onFalse(new Windmill(robotContainer.elevator, robotContainer.pivot,
                // WindmillState.Home, false));

                coralDropOff4.and(scoreLeft).onTrue(new SequentialCommandGroup(
                                new Elevator(robotContainer.elevator, ElevatorConstants.kElevatorCoralDropOff4Height),
                                new WaitUntilCommand(() -> robotContainer.elevator.elevatorAtCoralDropOff4Height()),
                                new ToCoralDropOff(robotContainer.elevator, robotContainer.pivot,
                                                robotContainer.intake, robotContainer.endEffector,
                                                WindmillState.CoralLeftDropOff4, false)));
                // .onFalse(new Windmill(robotContainer.elevator, robotContainer.pivot,
                // WindmillState.Home, false));

                coralDropOff4.and(scoreRight).onTrue(new SequentialCommandGroup(
                                new Elevator(robotContainer.elevator, ElevatorConstants.kElevatorCoralDropOff4Height),
                                new WaitUntilCommand(() -> robotContainer.elevator.elevatorAtCoralDropOff4Height()),
                                new ToCoralDropOff(robotContainer.elevator, robotContainer.pivot,
                                                robotContainer.intake, robotContainer.endEffector,
                                                WindmillState.CoralDropOff4, true)));
                // .onFalse(new Windmill(robotContainer.elevator, robotContainer.pivot,
                // WindmillState.Home, false));

                coralHandOff.onTrue(new TestHandoff(robotContainer.elevator, robotContainer.pivot,
                                robotContainer.intake, robotContainer.endEffector));

                // coralEject.onTrue(new ScoreReef(robotContainer.elevator,
                // robotContainer.pivot,
                // robotContainer.endEffector, robotContainer.intake)).debounce(0.5)
                // .onFalse(new SequentialCommandGroup(
                // new Windmill(robotContainer.elevator, robotContainer.pivot,
                // WindmillState.Home,
                // false),
                // new InstantCommand(() -> robotContainer.endEffector.stop())));
                algaeReef3.onTrue(new DeAlgaeReef3(robotContainer.elevator, robotContainer.pivot,
                                robotContainer.endEffector));
                algaeReef2.onTrue(new DeAlgaeReef2(robotContainer.elevator, robotContainer.pivot,
                                robotContainer.endEffector));
                toggleClimber.toggleOnTrue(new ConditionalCommand(new DeployClimb(robotContainer.climber),
                                new RetractClimb(robotContainer.climber),
                                robotContainer.climber::climberAtHomePosition));

                raiseClimber.onTrue(new RunCommand(() -> robotContainer.climber.moveUp(), robotContainer.climber))
                                .onFalse(new RunCommand(() -> robotContainer.climber.stop(), robotContainer.climber));
                lowerClimber.onTrue(new RunCommand(() -> robotContainer.climber.moveDown(), robotContainer.climber))
                                .onFalse(new RunCommand(() -> robotContainer.climber.stop(), robotContainer.climber));

                toggleClimber.toggleOnTrue(new ConditionalCommand(
                                new LockPivotAndElevator(robotContainer.elevator, robotContainer.pivot,
                                                robotContainer.intake).withTimeout(15)
                                                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming),
                                new LockPivotAndElevator(robotContainer.elevator, robotContainer.pivot,
                                                robotContainer.intake),
                                robotContainer.climber::climberAtHomePosition));

                pivotClockwise.onTrue(new InstantCommand(robotContainer.pivot::start, robotContainer.pivot)).onFalse(
                                new InstantCommand(robotContainer.pivot::holdPivotArmManual, robotContainer.pivot));
                pivotCounterClockwise.onTrue(new InstantCommand(robotContainer.pivot::reverse, robotContainer.pivot))
                                .onFalse(new InstantCommand(robotContainer.pivot::holdPivotArmManual,
                                                robotContainer.pivot));

                raiseElevator.whileTrue(new InstantCommand(robotContainer.elevator::moveUp, robotContainer.elevator)
                                .andThen(new WaitUntilCommand(() -> robotContainer.elevator.ElevatorRaiseLimitHit()))
                                .andThen(new InstantCommand(robotContainer.elevator::holdHeight,
                                                robotContainer.elevator)))
                                .onFalse(new InstantCommand(robotContainer.elevator::holdHeight,
                                                robotContainer.elevator));
                lowerElevator.whileTrue(new InstantCommand(robotContainer.elevator::moveDown, robotContainer.elevator))
                                .onFalse(new InstantCommand(robotContainer.elevator::holdHeight,
                                                robotContainer.elevator));
                lockWheels.onTrue(new InstantCommand(robotContainer.drivetrain::lock, robotContainer.drivetrain));
        }
}