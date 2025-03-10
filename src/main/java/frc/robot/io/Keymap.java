package frc.robot.io;

import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Keymap {

    public static class Controllers {

        public CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
        public CommandXboxController operatorController = new CommandXboxController(
                OIConstants.kOperaterControllerPort);

    }

    // public static Controllers controllers = new Controllers();

    public static class Layout {
        double driverLeftX = Keymap.Controllers.driverController.getLeftX();
        double driverLeftY = controllers.driverController.getLeftY();
        double driverRightX = controllers.driverController.getRightX();
        double driverRightY = controllers.driverController.getRightY();
        double driverLeftTrigger = controllers.driverController.getLeftTriggerAxis();
        double driverRightTrigger = controllers.driverController.getRightTriggerAxis();
        boolean DriverXButton = controllers.driverController.getHID().getXButton();
        boolean DriverBButton = controllers.driverController.getHID().getBButton();
        boolean DriverYButton = controllers.driverController.getHID().getYButton();
        boolean DriverAButton = controllers.driverController.getHID().getAButton();
        boolean DriverRightBumper = controllers.driverController.getHID().getRightBumperButton();
        boolean DriverLeftBumper = controllers.driverController.getHID().getLeftBumperButton();

        double OperatorLeftX = controllers.driverController.getLeftX();
        double OperatorLeftY = controllers.driverController.getLeftY();
        double OperatorRightX = controllers.driverController.getRightX();
        double OperatorRightY = controllers.driverController.getRightY();
    }

    // public static Layout layout = new Layout();

}