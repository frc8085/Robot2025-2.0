
package frc.robot.io;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FakeConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.windmill.*;
import frc.robot.commands.scoring.*;
import frc.robot.commands.sequences.RemoveAlgaeL2;
import frc.robot.commands.sequences.RemoveAlgaeL3;
import frc.robot.commands.manipulator.coral.*;
import frc.robot.commands.manipulator.algae.*;
import frc.robot.commands.movement.*;
import frc.robot.commands.states.*;
import frc.robot.commands.climber.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.windmill.elevator.*;

public class IO {

        public void init(RobotContainer robotContainer) {

                // Additional Buttons to allow for alternate button pushes
                final Trigger altButtonDriver = Keymap.Layout.driverRightBumper;
                final Trigger altButtonOperator = Keymap.Layout.operatorRightBumper;

                // Initialization
                final Trigger zeroArm = Keymap.Layout.operatorStartButton;
                final Trigger zeroHeadingButton = Keymap.Layout.driverStartButton;

                // final Trigger zeroElevator = operatorController.start();
                final Trigger limelightTrigger1 = Keymap.Layout.driverXButton;
                final Trigger limelightTrigger2 = Keymap.Layout.driverBButton;

                // Driver operations
                final Trigger ejectCoral = Keymap.Layout.driverAButton;
                final Trigger pickUpCoral = Keymap.Layout.driverLeftTriggerButton;
                final Trigger ejectAlgae = Keymap.Layout.driverYButton;
                final Trigger shootAlgaeNetBlue = Keymap.Layout.driverLeftBumper;
                // final Trigger left = Keymap.Layout.driverDownButton;
                // final Trigger right = Keymap.Layout.driverUpButton;
                final Trigger gorobotrelative = Keymap.Controllers.driverController.leftStick();
                final Trigger raiseClimber = Keymap.Layout.driverRightButton;
                final Trigger lowerClimber = Keymap.Layout.driverLeftButton;
                final Trigger goSlow = Keymap.Layout.driverBackButton;

                // Operator Controls
                final Trigger manualCoral = Keymap.Layout.operatorRightTriggerButton;
                final Trigger manualAlgae = Keymap.Layout.operatorLeftTriggerButton;
                final Trigger toggleClimber = Keymap.Layout.operatorBackButton;

                // Operator Set Position Controls
                final Trigger home = Keymap.Layout.operatorLeftBumper;
                final Trigger algaeGround = Keymap.Layout.operatorDownButton;
                final Trigger algaeReef2 = Keymap.Layout.operatorRightButton;
                final Trigger algaeReef3 = Keymap.Layout.operatorUpButton;
                final Trigger algaeProcessor = Keymap.Layout.operatorLeftButton;
                final Trigger coralDropOff4 = Keymap.Layout.operatorYButton;
                final Trigger coralDropOff3 = Keymap.Layout.operatorXButton;
                final Trigger coralDropOff2 = Keymap.Layout.operatorBButton;
                final Trigger coralDropOff1 = Keymap.Layout.operatorAButton;

                // Set Left Joystick for manual elevator/pivot movement
                final Trigger raiseElevator = Keymap.Controllers.operatorController.axisLessThan(1, -0.25);
                final Trigger lowerElevator = Keymap.Controllers.operatorController.axisGreaterThan(1, 0.25);
                final Trigger pivotClockwise = Keymap.Controllers.operatorController.axisGreaterThan(4, 0.25);
                final Trigger pivotCounterClockwise = Keymap.Controllers.operatorController.axisLessThan(4, -0.25);

                // Initialization
                // Zero elevator - carriage must be below stage 1 or it will zero where it is
                // zeroElevator.onTrue(new ZeroElevator(robotContainer.elevator));
                // zeroArm.onTrue(new ZeroElevator(robotContainer.elevator));
                // zeroArm.and(altButtonOperator)
                //                 .onTrue(new InitializePivotAndElevator(robotContainer.pivot, robotContainer.elevator));

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

                gorobotrelative.onTrue(new InstantCommand(() -> {
                        if (FakeConstants.fieldRelative) {
                                FakeConstants.fieldRelative = false;
                        } else {
                                FakeConstants.fieldRelative = true;
                        }
                }));

                goSlow.toggleOnTrue(new SwerveDriveTeleopRoboRelativeSlow(robotContainer.drivetrain)).toggleOnFalse(
                                new SwerveDriveTeleop(robotContainer.drivetrain));


                manualCoral.onTrue(new SetIntakeAngle(robotContainer.intake, Constants.IntakeConstants.kIntakeOutAngle))
                                .onFalse(new SetIntakeAngle(robotContainer.intake, Constants.IntakeConstants.kIntakeInAngle));

                // // commands that go with driver operations
                // ejectCoral.onTrue(new EjectCoral(robotContainer.coral, robotContainer.elevator,
                //                 robotContainer.pivot));
                // ejectCoral.and(altButtonDriver).onTrue(new DropCoral(robotContainer.coral,
                //                 robotContainer.elevator, robotContainer.pivot, robotContainer.drivetrain));

                // pickUpCoral.onTrue(new PickUpCoralFromSource(robotContainer.coral,
                //                 robotContainer.elevator, robotContainer.pivot, false));
                // pickUpCoral.and(altButtonDriver)
                //                 .onTrue(new PickUpCoralFromSource(robotContainer.coral, robotContainer.elevator,
                //                                 robotContainer.pivot, true));

                // ejectAlgae.onTrue(new EjectAlgae(robotContainer.algae));
                // shootAlgaeNetBlue.onTrue(new SwerveDriveAlignBarge(robotContainer.drivetrain));
                // shootAlgaeNetBlue.and(altButtonDriver).onTrue(new ScoreAlgaeNet(robotContainer.algae,
                //                 robotContainer.elevator, robotContainer.pivot, robotContainer.coral, false));

                // raiseClimber.onTrue(new RunCommand(() -> robotContainer.climber.moveUp(),
                //                 robotContainer.climber))
                //                 .onFalse(new RunCommand(() -> robotContainer.climber.stop(),
                //                                 robotContainer.climber));
                // lowerClimber.onTrue(new RunCommand(() -> robotContainer.climber.moveDown(),
                //                 robotContainer.climber))
                //                 .onFalse(new RunCommand(() -> robotContainer.climber.stop(),
                //                                 robotContainer.climber));

                // manualCoral.onTrue(new SequentialCommandGroup(
                //                 new ToCoralSourceManual(robotContainer.elevator, robotContainer.pivot, false),
                //                 new RunCommand(() -> robotContainer.coral.pickup(), robotContainer.coral)))
                //                 .onFalse(new SequentialCommandGroup(
                //                                 new InstantCommand(() -> robotContainer.coral.stop(),
                //                                                 robotContainer.coral),
                //                                 new WaitCommand(0.25),
                //                                 new ToHomeCommand(robotContainer.elevator, robotContainer.pivot)));

                // manualAlgae.onTrue(new RunCommand(() -> robotContainer.algae.pickup(), robotContainer.algae))
                //                 .onFalse(new InstantCommand(robotContainer.algae::holdAlgae));

                // toggleClimber.toggleOnTrue(new ConditionalCommand(
                //                 new DeployClimb(robotContainer.climber),
                //                 new RetractClimb(robotContainer.climber),
                //                 robotContainer.climber::climberAtHomePosition));

                // toggleClimber.toggleOnTrue(new ConditionalCommand(
                //                 new LockPivotAndElevatorCommand(robotContainer.elevator,
                //                                 robotContainer.pivot).withTimeout(15)
                //                                 .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming),
                //                 new ToHomeCommand(robotContainer.elevator, robotContainer.pivot),
                //                 robotContainer.climber::climberAtHomePosition));

                // // Set Move to Positions
                // home.onTrue(new Windmill(robotContainer.elevator, robotContainer.pivot,
                //                 Constants.Windmill.WindmillState.Home,
                //                 false));

                // algaeGround.onTrue(new PickUpAlgaeFromGround(robotContainer.algae, robotContainer.elevator,
                //                 robotContainer.pivot));

                // algaeReef2.onTrue(new RemoveAlgaeL2(robotContainer.elevator, robotContainer.pivot, robotContainer.algae,
                //                 false));

                // algaeReef3.onTrue(new RemoveAlgaeL3(robotContainer.elevator, robotContainer.pivot, robotContainer.algae,
                //                 false));

                // algaeProcessor.onTrue(new ToAlgaeGround(robotContainer.elevator, robotContainer.pivot));

                // coralDropOff1.onTrue(new ToCoralDropOff1(robotContainer.elevator, robotContainer.pivot, false));

                // coralDropOff2.onTrue(new ToCoralDropOff2(robotContainer.elevator, robotContainer.pivot, false));

                // coralDropOff3.onTrue(new ToCoralDropOff3(robotContainer.elevator, robotContainer.pivot, false));

                // coralDropOff4.onTrue(new ScoreCoralL4(robotContainer.elevator, robotContainer.pivot,
                //                 robotContainer.coral, false));

                // algaeReef3.and(altButtonOperator).onTrue(
                //                 new RemoveAlgaeL3(robotContainer.elevator, robotContainer.pivot, robotContainer.algae,
                //                                 true));

                // coralDropOff1.and(altButtonOperator)
                //                 .onTrue(new ScoreCoralL1(robotContainer.elevator, robotContainer.pivot,
                //                                 robotContainer.coral, true));
                // coralDropOff2.and(altButtonOperator)
                //                 .onTrue(new ScoreCoralL2(robotContainer.elevator, robotContainer.pivot,
                //                                 robotContainer.coral, true));
                // coralDropOff3.and(altButtonOperator)
                //                 .onTrue(new ScoreCoralL3(robotContainer.elevator, robotContainer.pivot,
                //                                 robotContainer.coral, true));
                // coralDropOff4.and(altButtonOperator)
                //                 .onTrue(new ScoreCoralL4(robotContainer.elevator, robotContainer.pivot,
                //                                 robotContainer.coral,
                //                                 true));

                // // commands that go with manual elevator/pivot movement
                // pivotClockwise
                //                 .onTrue(new InstantCommand(robotContainer.pivot::start, robotContainer.pivot))
                //                 .onFalse(new InstantCommand(robotContainer.pivot::holdPivotArmManual,
                //                                 robotContainer.pivot));
                // pivotCounterClockwise.onTrue(new InstantCommand(robotContainer.pivot::reverse, robotContainer.pivot))
                //                 .onFalse(new InstantCommand(robotContainer.pivot::holdPivotArmManual,
                //                                 robotContainer.pivot));
                // raiseElevator.whileTrue(
                //                 new InstantCommand(robotContainer.elevator::moveUp, robotContainer.elevator)
                //                                 .andThen(new WaitUntilCommand(
                //                                                 () -> robotContainer.elevator.ElevatorRaiseLimitHit()))
                //                                 .andThen(new InstantCommand(robotContainer.elevator::holdHeight,
                //                                                 robotContainer.elevator)))
                //                 .onFalse(new InstantCommand(robotContainer.elevator::holdHeight,
                //                                 robotContainer.elevator));
                // lowerElevator.whileTrue(
                //                 new InstantCommand(robotContainer.elevator::moveDown, robotContainer.elevator)
                //                                 .andThen(new WaitUntilCommand(
                //                                                 () -> robotContainer.elevator.ElevatorLowerLimitHit()))
                //                                 .andThen(new InstantCommand(robotContainer.elevator::holdHeight,
                //                                                 robotContainer.elevator)))
                //                 .onFalse(new InstantCommand(robotContainer.elevator::holdHeight,
                //                                 robotContainer.elevator));
        }
}