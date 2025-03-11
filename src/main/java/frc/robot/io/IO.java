

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
        // zeroElevator.onTrue(new ZeroElevator(RobotContainer.elevator));
                zeroArm.onTrue(new ZeroElevator(RobotContainer.elevator));
                zeroArm.and(altButtonOperator).onTrue(new InitializePivotAndElevator(RobotContainer.pivot, RobotContainer.elevator));

                // Reset heading of robot for field relative drive
                zeroHeadingButton.onTrue(new InstantCommand(() -> RobotContainer.drivetrain.zeroHeading(), RobotContainer.drivetrain));

                // // Limelight Buttons


                // RobotContainer.limelightTrigger1.onTrue(
                // new ParallelRaceGroup(new WaitCommand(1), new
                // AlignToAprilTagBlue(RobotContainer.drivetrain,
                // RobotContainer.limelight)));

                // RobotContainer.limelightTrigger2.onTrue(
                // new ParallelRaceGroup(new WaitCommand(4), new
                // AlignToAprilTagYellow(RobotContainer.drivetrain,
                // RobotContainer.limelight)));
                limelightTrigger1.onTrue(new AlignToAprilTagBlue(RobotContainer.drivetrain, RobotContainer.limelight));
                limelightTrigger2.onTrue(
                                new AlignToAprilTagYellow(RobotContainer.drivetrain, RobotContainer.limelight));


                gorobotrelative.onTrue(new InstantCommand(() -> {
                        if (FakeConstants.fieldRelative) {
                                FakeConstants.fieldRelative = false;
                        } else {
                                FakeConstants.fieldRelative = true;
                        }
                }));
                // commands that go with driver operations
                ejectCoral.onTrue(new EjectCoral(RobotContainer.coral, RobotContainer.elevator,
                                RobotContainer.pivot, RobotContainer.drivetrain));
                ejectCoral.and(altButtonDriver).onTrue(new DropCoral(RobotContainer.coral,
                                RobotContainer.elevator, RobotContainer.pivot, RobotContainer.drivetrain));
                pickUpCoral.onTrue(new PickUpCoralFromSource(RobotContainer.coral,
                                RobotContainer.elevator, RobotContainer.pivot, false));
                pickUpCoral.and(altButtonDriver)
                                .onTrue(new PickUpCoralFromSource(RobotContainer.coral, RobotContainer.elevator,
                                                RobotContainer.pivot,
                                                true));

                ejectAlgae.onTrue(new EjectAlgae(RobotContainer.algae));
                shootAlgaeNetBlue.onTrue(new ScoreAlgaeNetBlue(RobotContainer.algae,
                                RobotContainer.elevator, RobotContainer.pivot,
                                RobotContainer.coral));
                shootAlgaeNetYellow.onTrue(new ScoreAlgaeNetYellow(RobotContainer.algae,
                                RobotContainer.elevator, RobotContainer.pivot,
                                RobotContainer.coral));

                left.onTrue(new AutoPositionLeftRight(RobotContainer.drivetrain, RobotContainer.limelight, false,
                                (RobotContainer.limelight.hasTarget("RobotContainer.limelight-yellow"))));
                right.onTrue(new AutoPositionLeftRight(RobotContainer.drivetrain, RobotContainer.limelight, true,
                                (RobotContainer.limelight.hasTarget("RobotContainer.limelight-yellow"))));
                raiseClimber.onTrue(new RunCommand(() -> RobotContainer.climber.moveUp(),
                                RobotContainer.climber))
                                .onFalse(new RunCommand(() -> RobotContainer.climber.stop(),
                                                RobotContainer.climber));
                lowerClimber.onTrue(new RunCommand(() -> RobotContainer.climber.moveDown(),
                                RobotContainer.climber))
                                .onFalse(new RunCommand(() -> RobotContainer.climber.stop(),
                                                RobotContainer.climber));

                manualCoral.onTrue(new SequentialCommandGroup(
                                new ToCoralSourceManual(RobotContainer.elevator, RobotContainer.pivot, false),
                                new RunCommand(() -> RobotContainer.coral.pickup(), RobotContainer.coral)))
                                .onFalse(new SequentialCommandGroup(
                                                new InstantCommand(() -> RobotContainer.coral.stop(), RobotContainer.coral),
                                                new WaitCommand(0.25),
                                                new ToHomeCommand(RobotContainer.elevator, RobotContainer.pivot, RobotContainer.coral)));

                manualAlgae.onTrue(new RunCommand(() -> RobotContainer.algae.pickup(), RobotContainer.algae))
                                .onFalse(new InstantCommand(RobotContainer.algae::holdAlgae));


                toggleClimber.toggleOnTrue(new ConditionalCommand(
                                new DeployClimb(RobotContainer.climber),
                                new RetractClimb(RobotContainer.climber),
                                RobotContainer.climber::climberAtHomePosition));

                toggleClimber.toggleOnTrue(new ConditionalCommand(
                                new LockPivotAndElevatorCommand(RobotContainer.elevator,
                                                RobotContainer.pivot).withTimeout(15)
                                                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming),
                                new ToHomeCommand(RobotContainer.elevator, RobotContainer.pivot, RobotContainer.coral),
                                RobotContainer.climber::climberAtHomePosition));

                // initializeInputs.onTrue(new ResetOperatorInputs());

                home.onTrue(new Windmill(RobotContainer.elevator, RobotContainer.pivot, Constants.Windmill.WindmillState.Home,
                                false));

                algaeGround.onTrue(new PickUpAlgaeFromGround(RobotContainer.algae, RobotContainer.elevator, RobotContainer.pivot));

                algaeReef2.onTrue(new SequentialCommandGroup(new ToAlgaeL2(RobotContainer.elevator, RobotContainer.pivot, false),
                                new PickUpAlgae(RobotContainer.algae), new WaitCommand(.25),
                                new Windmill(RobotContainer.elevator, RobotContainer.pivot,
                                                Constants.Windmill.WindmillState.Home,
                                                false)));
                algaeReef3.onTrue(new SequentialCommandGroup(new ToAlgaeL3(RobotContainer.elevator, RobotContainer.pivot, false),
                                new PickUpAlgae(RobotContainer.algae), new WaitCommand(.25),
                                new Windmill(RobotContainer.elevator, RobotContainer.pivot,
                                                Constants.Windmill.WindmillState.AlgaeHoldHeight,
                                                false)));

                algaeProcessor.onTrue(new ToAlgaeGround(RobotContainer.elevator, RobotContainer.pivot));

                coralDropOff1.onTrue(new ScoreCoralL1(RobotContainer.elevator, RobotContainer.pivot, RobotContainer.coral, false));

                coralDropOff2.onTrue(new ScoreCoralL2(RobotContainer.elevator, RobotContainer.pivot, RobotContainer.coral, false));

                coralDropOff3.onTrue(new ScoreCoralL3(RobotContainer.elevator, RobotContainer.pivot, RobotContainer.coral, false));

                coralDropOff4.onTrue(new ScoreCoralL4(RobotContainer.elevator, RobotContainer.pivot, RobotContainer.coral, RobotContainer.drivetrain,
                                false));

                algaeReef2.and(altButtonOperator)
                                .onTrue(new SequentialCommandGroup(
                                                new ToAlgaeL2(RobotContainer.elevator, RobotContainer.pivot, true),
                                                new PickUpAlgae(RobotContainer.algae), new WaitCommand(.25),
                                                new Windmill(RobotContainer.elevator, RobotContainer.pivot,
                                                                Constants.Windmill.WindmillState.Home,
                                                                true)));
                algaeReef3.and(altButtonOperator).onTrue(
                                new SequentialCommandGroup(new ToAlgaeL3(RobotContainer.elevator, RobotContainer.pivot, true),
                                                new PickUpAlgae(RobotContainer.algae), new WaitCommand(.25),
                                                new Windmill(RobotContainer.elevator, RobotContainer.pivot,
                                                                Constants.Windmill.WindmillState.AlgaeHoldHeight,
                                                                true)));

                coralDropOff1.and(altButtonOperator)
                                .onTrue(new ScoreCoralL1(RobotContainer.elevator, RobotContainer.pivot, RobotContainer.coral, true));
                coralDropOff2.and(altButtonOperator)
                                .onTrue(new ScoreCoralL2(RobotContainer.elevator, RobotContainer.pivot, RobotContainer.coral, true));
                coralDropOff3.and(altButtonOperator)
                                .onTrue(new ScoreCoralL3(RobotContainer.elevator, RobotContainer.pivot, RobotContainer.coral, true));
                coralDropOff4.and(altButtonOperator)
                                .onTrue(new ScoreCoralL4(RobotContainer.elevator, RobotContainer.pivot, RobotContainer.coral,
                                                RobotContainer.drivetrain, true));


                // commands that go with manual elevator/pivot movement
                pivotClockwise
                                .onTrue(new InstantCommand(RobotContainer.pivot::start, RobotContainer.pivot))
                                .onFalse(new InstantCommand(RobotContainer.pivot::holdPivotArmManual, RobotContainer.pivot));
                pivotCounterClockwise.onTrue(new InstantCommand(RobotContainer.pivot::reverse, RobotContainer.pivot))
                                .onFalse(new InstantCommand(RobotContainer.pivot::holdPivotArmManual, RobotContainer.pivot));
                raiseElevator.whileTrue(
                                new InstantCommand(RobotContainer.elevator::moveUp, RobotContainer.elevator)
                                                .andThen(new WaitUntilCommand(
                                                                () -> RobotContainer.elevator.ElevatorRaiseLimitHit()))
                                                .andThen(new InstantCommand(RobotContainer.elevator::holdHeight,
                                                                RobotContainer.elevator)))
                                .onFalse(new InstantCommand(RobotContainer.elevator::holdHeight, RobotContainer.elevator));
                lowerElevator.whileTrue(
                                new InstantCommand(RobotContainer.elevator::moveDown, RobotContainer.elevator)
                                                .andThen(new WaitUntilCommand(
                                                                () -> RobotContainer.elevator.ElevatorLowerLimitHit()))
                                                .andThen(new InstantCommand(RobotContainer.elevator::holdHeight,
                                                                RobotContainer.elevator)))
                                .onFalse(new InstantCommand(RobotContainer.elevator::holdHeight,
                                                RobotContainer.elevator));
    }
}