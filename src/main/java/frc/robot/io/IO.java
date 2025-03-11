

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
import frc.robot.RobotSubsystems;
import frc.robot.commands.windmill.*;
import frc.robot.commands.scoring.*;
import frc.robot.commands.manipulator.coral.*;
import frc.robot.commands.manipulator.algae.*;
import frc.robot.commands.movement.*;
import frc.robot.commands.states.*;
import frc.robot.commands.climber.*;
import frc.robot.commands.windmill.elevator.*;



public class IO {

    public void init() {


        // Additional Buttons to allow for alternate button pushes
        final Trigger altButtonDriver =  Keymap.Layout.driverBackButton;
        final Trigger altButtonOperator = Keymap.Layout.operatorRightBumper;

        //Initialization
        final Trigger zeroArm = Keymap.Layout.operatorStartButton;
        final Trigger zeroHeadingButton = Keymap.Layout.driverStartButton;

        // final Trigger zeroElevator = operatorController.start();
        final Trigger limelightTrigger1 = Keymap.Layout.driverXButton;
        final Trigger limelightTrigger2 = Keymap.Layout.driverBButton;
 
        // Driver operations
        final Trigger ejectCoral = Keymap.Layout.driverAButton;
        final Trigger pickUpCoral = Keymap.Layout.driverLeftTriggerButton;
        final Trigger ejectAlgae =  Keymap.Layout.driverYButton;
        final Trigger shootAlgaeNetBlue =  Keymap.Layout.driverLeftBumper;
        final Trigger shootAlgaeNetYellow =  Keymap.Layout.driverRightBumper;
        final Trigger left =  Keymap.Layout.driverDownButton;
        final Trigger right =  Keymap.Layout.driverUpButton;
        final Trigger gorobotrelative =  Keymap.Controllers.driverController.leftStick();
        final Trigger raiseClimber =  Keymap.Layout.driverRightButton;
        final Trigger lowerClimber =  Keymap.Layout.driverLeftButton;
        

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
                
                
        //Initialization
        // Zero elevator - carriage must be below stage 1 or it will zero where it is
        // zeroElevator.onTrue(new ZeroElevator(RobotSubsystems.elevator));
                zeroArm.onTrue(new ZeroElevator(RobotSubsystems.elevator));
                zeroArm.and(altButtonOperator).onTrue(new InitializePivotAndElevator(RobotSubsystems.pivot, RobotSubsystems.elevator));

                // Reset heading of robot for field relative drive
                zeroHeadingButton.onTrue(new InstantCommand(() -> RobotSubsystems.drivetrain.zeroHeading(), RobotSubsystems.drivetrain));

                // // Limelight Buttons


                // RobotSubsystems.limelightTrigger1.onTrue(
                // new ParallelRaceGroup(new WaitCommand(1), new
                // AlignToAprilTagBlue(RobotSubsystems.drivetrain,
                // RobotSubsystems.limelight)));

                // RobotSubsystems.limelightTrigger2.onTrue(
                // new ParallelRaceGroup(new WaitCommand(4), new
                // AlignToAprilTagYellow(RobotSubsystems.drivetrain,
                // RobotSubsystems.limelight)));
                limelightTrigger1.onTrue(new AlignToAprilTagBlue(RobotSubsystems.drivetrain, RobotSubsystems.limelight));
                limelightTrigger2.onTrue(
                                new AlignToAprilTagYellow(RobotSubsystems.drivetrain, RobotSubsystems.limelight));


                gorobotrelative.onTrue(new InstantCommand(() -> {
                        if (FakeConstants.fieldRelative) {
                                FakeConstants.fieldRelative = false;
                        } else {
                                FakeConstants.fieldRelative = true;
                        }
                }));
                // commands that go with driver operations
                ejectCoral.onTrue(new EjectCoral(RobotSubsystems.coral, RobotSubsystems.elevator,
                                RobotSubsystems.pivot, RobotSubsystems.drivetrain));
                ejectCoral.and(altButtonDriver).onTrue(new DropCoral(RobotSubsystems.coral,
                                RobotSubsystems.elevator, RobotSubsystems.pivot, RobotSubsystems.drivetrain));
                pickUpCoral.onTrue(new PickUpCoralFromSource(RobotSubsystems.coral,
                                RobotSubsystems.elevator, RobotSubsystems.pivot, false));
                pickUpCoral.and(altButtonDriver)
                                .onTrue(new PickUpCoralFromSource(RobotSubsystems.coral, RobotSubsystems.elevator,
                                                RobotSubsystems.pivot,
                                                true));

                ejectAlgae.onTrue(new EjectAlgae(RobotSubsystems.algae));
                shootAlgaeNetBlue.onTrue(new ScoreAlgaeNetBlue(RobotSubsystems.algae,
                                RobotSubsystems.elevator, RobotSubsystems.pivot,
                                RobotSubsystems.coral));
                shootAlgaeNetYellow.onTrue(new ScoreAlgaeNetYellow(RobotSubsystems.algae,
                                RobotSubsystems.elevator, RobotSubsystems.pivot,
                                RobotSubsystems.coral));

                left.onTrue(new AutoPositionLeftRight(RobotSubsystems.drivetrain, RobotSubsystems.limelight, false,
                                (RobotSubsystems.limelight.hasTarget("RobotSubsystems.limelight-yellow"))));
                right.onTrue(new AutoPositionLeftRight(RobotSubsystems.drivetrain, RobotSubsystems.limelight, true,
                                (RobotSubsystems.limelight.hasTarget("RobotSubsystems.limelight-yellow"))));
                raiseClimber.onTrue(new RunCommand(() -> RobotSubsystems.climber.moveUp(),
                                RobotSubsystems.climber))
                                .onFalse(new RunCommand(() -> RobotSubsystems.climber.stop(),
                                                RobotSubsystems.climber));
                lowerClimber.onTrue(new RunCommand(() -> RobotSubsystems.climber.moveDown(),
                                RobotSubsystems.climber))
                                .onFalse(new RunCommand(() -> RobotSubsystems.climber.stop(),
                                                RobotSubsystems.climber));

                manualCoral.onTrue(new SequentialCommandGroup(
                                new ToCoralSourceManual(RobotSubsystems.elevator, RobotSubsystems.pivot, false),
                                new RunCommand(() -> RobotSubsystems.coral.pickup(), RobotSubsystems.coral)))
                                .onFalse(new SequentialCommandGroup(
                                                new InstantCommand(() -> RobotSubsystems.coral.stop(), RobotSubsystems.coral),
                                                new WaitCommand(0.25),
                                                new ToHomeCommand(RobotSubsystems.elevator, RobotSubsystems.pivot, RobotSubsystems.coral)));

                manualAlgae.onTrue(new RunCommand(() -> RobotSubsystems.algae.pickup(), RobotSubsystems.algae))
                                .onFalse(new InstantCommand(RobotSubsystems.algae::holdAlgae));


                toggleClimber.toggleOnTrue(new ConditionalCommand(
                                new DeployClimb(RobotSubsystems.climber),
                                new RetractClimb(RobotSubsystems.climber),
                                RobotSubsystems.climber::climberAtHomePosition));

                toggleClimber.toggleOnTrue(new ConditionalCommand(
                                new LockPivotAndElevatorCommand(RobotSubsystems.elevator,
                                                RobotSubsystems.pivot).withTimeout(15)
                                                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming),
                                new ToHomeCommand(RobotSubsystems.elevator, RobotSubsystems.pivot, RobotSubsystems.coral),
                                RobotSubsystems.climber::climberAtHomePosition));

                // initializeInputs.onTrue(new ResetOperatorInputs());

                home.onTrue(new Windmill(RobotSubsystems.elevator, RobotSubsystems.pivot, Constants.Windmill.WindmillState.Home,
                                false));

                algaeGround.onTrue(new PickUpAlgaeFromGround(RobotSubsystems.algae, RobotSubsystems.elevator, RobotSubsystems.pivot));

                algaeReef2.onTrue(new SequentialCommandGroup(new ToAlgaeL2(RobotSubsystems.elevator, RobotSubsystems.pivot, false),
                                new PickUpAlgae(RobotSubsystems.algae), new WaitCommand(.25),
                                new Windmill(RobotSubsystems.elevator, RobotSubsystems.pivot,
                                                Constants.Windmill.WindmillState.Home,
                                                false)));
                algaeReef3.onTrue(new SequentialCommandGroup(new ToAlgaeL3(RobotSubsystems.elevator, RobotSubsystems.pivot, false),
                                new PickUpAlgae(RobotSubsystems.algae), new WaitCommand(.25),
                                new Windmill(RobotSubsystems.elevator, RobotSubsystems.pivot,
                                                Constants.Windmill.WindmillState.AlgaeHoldHeight,
                                                false)));

                algaeProcessor.onTrue(new ToAlgaeGround(RobotSubsystems.elevator, RobotSubsystems.pivot));

                coralDropOff1.onTrue(new ScoreCoralL1(RobotSubsystems.elevator, RobotSubsystems.pivot, RobotSubsystems.coral, false));

                coralDropOff2.onTrue(new ScoreCoralL2(RobotSubsystems.elevator, RobotSubsystems.pivot, RobotSubsystems.coral, false));

                coralDropOff3.onTrue(new ScoreCoralL3(RobotSubsystems.elevator, RobotSubsystems.pivot, RobotSubsystems.coral, false));

                coralDropOff4.onTrue(new ScoreCoralL4(RobotSubsystems.elevator, RobotSubsystems.pivot, RobotSubsystems.coral, RobotSubsystems.drivetrain,
                                false));

                algaeReef2.and(altButtonOperator)
                                .onTrue(new SequentialCommandGroup(
                                                new ToAlgaeL2(RobotSubsystems.elevator, RobotSubsystems.pivot, true),
                                                new PickUpAlgae(RobotSubsystems.algae), new WaitCommand(.25),
                                                new Windmill(RobotSubsystems.elevator, RobotSubsystems.pivot,
                                                                Constants.Windmill.WindmillState.Home,
                                                                true)));
                algaeReef3.and(altButtonOperator).onTrue(
                                new SequentialCommandGroup(new ToAlgaeL3(RobotSubsystems.elevator, RobotSubsystems.pivot, true),
                                                new PickUpAlgae(RobotSubsystems.algae), new WaitCommand(.25),
                                                new Windmill(RobotSubsystems.elevator, RobotSubsystems.pivot,
                                                                Constants.Windmill.WindmillState.AlgaeHoldHeight,
                                                                true)));

                coralDropOff1.and(altButtonOperator)
                                .onTrue(new ScoreCoralL1(RobotSubsystems.elevator, RobotSubsystems.pivot, RobotSubsystems.coral, true));
                coralDropOff2.and(altButtonOperator)
                                .onTrue(new ScoreCoralL2(RobotSubsystems.elevator, RobotSubsystems.pivot, RobotSubsystems.coral, true));
                coralDropOff3.and(altButtonOperator)
                                .onTrue(new ScoreCoralL3(RobotSubsystems.elevator, RobotSubsystems.pivot, RobotSubsystems.coral, true));
                coralDropOff4.and(altButtonOperator)
                                .onTrue(new ScoreCoralL4(RobotSubsystems.elevator, RobotSubsystems.pivot, RobotSubsystems.coral,
                                                RobotSubsystems.drivetrain, true));


                // commands that go with manual elevator/pivot movement
                pivotClockwise
                                .onTrue(new InstantCommand(RobotSubsystems.pivot::start, RobotSubsystems.pivot))
                                .onFalse(new InstantCommand(RobotSubsystems.pivot::holdPivotArmManual, RobotSubsystems.pivot));
                pivotCounterClockwise.onTrue(new InstantCommand(RobotSubsystems.pivot::reverse, RobotSubsystems.pivot))
                                .onFalse(new InstantCommand(RobotSubsystems.pivot::holdPivotArmManual, RobotSubsystems.pivot));
                raiseElevator.whileTrue(
                                new InstantCommand(RobotSubsystems.elevator::moveUp, RobotSubsystems.elevator)
                                                .andThen(new WaitUntilCommand(
                                                                () -> RobotSubsystems.elevator.ElevatorRaiseLimitHit()))
                                                .andThen(new InstantCommand(RobotSubsystems.elevator::holdHeight,
                                                                RobotSubsystems.elevator)))
                                .onFalse(new InstantCommand(RobotSubsystems.elevator::holdHeight, RobotSubsystems.elevator));
                lowerElevator.whileTrue(
                                new InstantCommand(RobotSubsystems.elevator::moveDown, RobotSubsystems.elevator)
                                                .andThen(new WaitUntilCommand(
                                                                () -> RobotSubsystems.elevator.ElevatorLowerLimitHit()))
                                                .andThen(new InstantCommand(RobotSubsystems.elevator::holdHeight,
                                                                RobotSubsystems.elevator)))
                                .onFalse(new InstantCommand(RobotSubsystems.elevator::holdHeight,
                                                RobotSubsystems.elevator));
    }
}