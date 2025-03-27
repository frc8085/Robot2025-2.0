package frc.robot.subsystems.Elevator;

public final class ElevatorConstants {

    // Elevator CanIDs
    public static final int kElevatorCanId = 23;
    public static final int kElevatorCancoderCanId = 33;

    public static final double kElevatorMotorGearRatio = 5;
    public static double kElevatorSpeed = .25;
    public static double kElevatorP = 4;
    public static double kElevatorI = 0;
    public static double kElevatorD = 0;
    public static double kElevatorV = .12;
    public static double kElevatorA = .01;
    public static double kElevatorStage2FF = 0.19;
    public static double kElevatorStage3FF = 0.38;

    public static double kElevatorMMVelo = 120;
    public static double kElevatorMMAcc = 140;
    public static double kElevatorMMJerk = 1600;

    public static double kElevatorAdjustment = -3;
    // Elevator Heights for different states
    public static double ropeAdjustmentFactor = .93;
    public static double kElevatorHomeHeight = 30;
    public static double kElevatorAlgaeHoldHeight = 47;
    public static double kElevatorCoralPickupHeight = 25;
    public static double kElevatorCoralPickupAlternateHeight = 20;
    public static double kElevatorCoralPickupHigher = 25;
    public static double kElevatorCoralPickupAltHigher = 20;
    public static double kElevatorCoralDropOff1Height = 17;
    public static double kElevatorCoralDropOff2Height = 47 * ropeAdjustmentFactor + kElevatorAdjustment;
    public static double kElevatorCoralDropOff3Height = 75 * ropeAdjustmentFactor + kElevatorAdjustment;
    public static double kElevatorCoralDropOff4Height = 118; // changed from 130
    // Drop off 4 was 130 before but 120 seems to be our max height
    public static double kElevatorAlgaePickUpFloorHeight = 15;
    public static double kElevatorReef2Height = 40 * ropeAdjustmentFactor;
    public static double kElevatorReef3Height = 70 * ropeAdjustmentFactor;
    public static double kElevatorAlgaePickUpFloorFlipHeight = 15;
    public static double kElevatorReef2FlipHeight = 25;
    public static double kElevatorReef3FlipHeight = 45;
    public static double kElevatorNetHeight = 110; // changed from 120

    // Determine what actual height values these are and/or what encoder readings
    // Stage Height refers to top of stage value
    public static final double kElevatorMin = 15; // adjusting for climber
    public static final double kElevatorStage1Height = 41; // zero position value
    public static final double kElevatorStage2Height = 85 * ropeAdjustmentFactor;
    public static final double kElevatorMax = 120; // 120 hard max

    /// The minimum height of the elevator that the pivot arm can swing through
    public static final double kElevatorSafeHeightMax = 55;
    public static final double kElevatorSafeHeightMin = 15;

    // The maximum height that the robot can safely travel at
    public static final double kElevatorSafeTravelHeight = 50;
    public static final double kElevatorSafeMidSpeedTravelHeight = 50;
    // this variable determines the minimum height at which the drivetrain speed
    // will be
    // its
    // slowest
    public static final double kElevatorMinTravelHeight = 80;

    // the Elevator tolerance
    public static final double kElevatorTolerance = 5;
}
