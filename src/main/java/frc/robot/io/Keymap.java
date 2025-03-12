package frc.robot.io;

import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Keymap {

        public static class Controllers {

                public static CommandXboxController driverController = new CommandXboxController(
                                OIConstants.kDriverControllerPort);
                public static CommandXboxController operatorController = new CommandXboxController(
                                OIConstants.kOperatorControllerPort);

        }

        // public static Controllers Controllers = new Controllers();

        public static class Layout {

                // public static double driverLeftX = Controllers.driverController.getLeftX();
                // public static double driverLeftY = Controllers.driverController.getLeftY();
                // public static double driverRightX = Controllers.driverController.getRightX();
                // public static double driverRightY = Controllers.driverController.getRightY();
                // public static double driverLeftTrigger =
                // Controllers.driverController.getLeftTriggerAxis();
                // public static double driverRightTrigger =
                // Controllers.driverController.getRightTriggerAxis();
                public static Trigger driverLeftTriggerButton = Controllers.driverController.leftTrigger();
                public static Trigger driverRightTriggerButton = Controllers.driverController.rightTrigger();
                public static Trigger driverLeftBumper = Controllers.driverController.leftBumper();
                public static Trigger driverRightBumper = Controllers.driverController.rightBumper();
                public static Trigger driverAButton = Controllers.driverController.a();
                public static Trigger driverBButton = Controllers.driverController.b();
                public static Trigger driverXButton = Controllers.driverController.x();
                public static Trigger driverYButton = Controllers.driverController.y();
                public static Trigger driverStartButton = Controllers.driverController.start();
                public static Trigger driverBackButton = Controllers.driverController.back();
                public static Trigger driverUpButton = Controllers.driverController.povUp();
                public static Trigger driverDownButton = Controllers.driverController.povDown();
                public static Trigger driverRightButton = Controllers.driverController.povRight();
                public static Trigger driverLeftButton = Controllers.driverController.povLeft();
                public static Trigger driverLeftStickButton = Controllers.driverController.leftStick();
                public static Trigger driverRightStickButton = Controllers.driverController.rightStick();

                // public static double operatorLeftX =
                // Controllers.operatorController.getLeftX();
                // public static double operatorLeftY =
                // Controllers.operatorController.getLeftY();
                // public static double operatorRightX =
                // Controllers.operatorController.getRightX();
                // public static double operatorRightY =
                // Controllers.operatorController.getRightY();
                // public static double operatorLeftTrigger =
                // Controllers.operatorController.getLeftTriggerAxis();
                // public static double operatorRightTrigger =
                // Controllers.operatorController.getRightTriggerAxis();
                public static Trigger operatorLeftTriggerButton = Controllers.operatorController.leftTrigger();
                public static Trigger operatorRightTriggerButton = Controllers.operatorController.rightTrigger();
                public static Trigger operatorLeftBumper = Controllers.operatorController.leftBumper();
                public static Trigger operatorRightBumper = Controllers.operatorController.rightBumper();
                public static Trigger operatorAButton = Controllers.operatorController.a();
                public static Trigger operatorBButton = Controllers.operatorController.b();
                public static Trigger operatorXButton = Controllers.operatorController.x();
                public static Trigger operatorYButton = Controllers.operatorController.y();
                public static Trigger operatorStartButton = Controllers.operatorController.start();
                public static Trigger operatorBackButton = Controllers.operatorController.back();
                public static Trigger operatorUpButton = Controllers.operatorController.povUp();
                public static Trigger operatorDownButton = Controllers.operatorController.povDown();
                public static Trigger operatorRightButton = Controllers.operatorController.povRight();
                public static Trigger operatorLeftButton = Controllers.operatorController.povLeft();
                public static Trigger operatorLeftStickButton = Controllers.operatorController.leftStick();
                public static Trigger operatorRightStickButton = Controllers.operatorController.rightStick();

        }

        // public static Layout layout = new Layout();

}