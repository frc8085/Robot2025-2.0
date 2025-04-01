package frc.robot.subsystems.Elevator;

public final class ElevatorConstants {

    // Elevator CanIDs
    public static final int kElevatorCanId = 23;
    public static final int kElevatorCancoderCanId = 33;

    public static final double kElevatorMotorGearRatio = 5;
    public static double kElevatorSpeed = .25; // .25
    public static double kElevatorP = 2; // 2
    public static double kElevatorI = 0; // 0
    public static double kElevatorD = 0; // 0
    public static double kElevatorS = 0.1; // 0
    public static double kElevatorG = 0; // 0
    public static double kElevatorV = 0.12; // 0.4
    public static double kElevatorA = 0; // 0.02
    public static double kElevatorStage2FF = 0; // 0.19 / .27

    public static double kElevatorMMVelo = 200;
    public static double kElevatorMMAcc = 110;
    public static double kElevatorMMJerk = 1600;

    public static double kElevatorAdjustment = -3;
    // Elevator Heights for different states
    public static double ropeAdjustmentFactor = .93;
    public static double kElevatorHomeHeight = 45;
    public static double kElevatorAutoTravelHeight = 64;
    public static double kElevatorAlgaeHoldHeight = 47;
    public static double kElevatorCoralHandoffHeight = 40;
    // public static double kElevatorCoralPickupAlternateHeight = 20;
    // public static double kElevatorCoralPickupHigher = 25;
    // public static double kElevatorCoralPickupAltHigher = 20;
    public static double kElevatorCoralDropOff1Height = 30; // 17
    public static double kElevatorCoralDropOff2Height = 10; // 47 * ropeAdjustmentFactor + kElevatorAdjustment
    public static double kElevatorCoralScore2Height = 7;
    public static double kElevatorCoralDropOff3Height = 20; // 75 * ropeAdjustmentFactor + kElevatorAdjustment
    public static double kElevatorCoralScore3Height = 15;
    public static double kElevatorCoralDropOff4Height = 55; // changed from 130
    public static double kElevatorCoralScore4Height = 45;
    // Drop off 4 was 130 before but 120 seems to be our max height
    // public static double kElevatorAlgaePickUpFloorHeight = 19;
    public static double kElevatorAlgaeReef2Height = 40;
    public static double kElevatorAlgaeReef3Height = 65;
    // public static double kElevatorAlgaePickUpFloorFlipHeight = 19;
    // public static double kElevatorReef2FlipHeight = 25;
    // public static double kElevatorReef3FlipHeight = 45;
    public static double kElevatorNetHeight = 99; // changed from 120

    // Determine what actual height values these are and/or what encoder readings
    // Stage Height refers to top of stage value
    public static final double kElevatorMin = 0; // adjusting for climber
    public static final double kElevatorZero = 20;
    public static final double kElevatorStage1Height = 20; // zero position value
    public static final double kElevatorMax = 60; // 108 hard max

    /// The minimum height of the elevator that the pivot arm can swing through
    public static final double kElevatorSafeHeightMax = 40;
    public static final double kElevatorSafeHeightMin = 10;

    // The maximum height that the robot can safely travel at
    public static final double kElevatorSafeTravelHeight = 50;
    public static final double kElevatorSafeMidSpeedTravelHeight = 60;
    // this variable determines the minimum height at which the drivetrain speed
    // will be
    // its
    // slowest
    public static final double kElevatorMinTravelHeight = 80;

    // the Elevator tolerance
    public static final double kElevatorTolerance = 5;
}
