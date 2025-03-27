package frc.robot.subsystems.Pivot;

import edu.wpi.first.math.geometry.Rotation2d;

public final class PivotArmConstants {

    public static final double kPivotMotorGearRatio = 27;
    public static double kPivotArmSpeed = .10;

    public static final double kPivotCancoderOffset = -.461670125;

    public static final double kPivotArmP = 1.4; // 0.7
    public static final double kPivotArmI = 0; // 0.0
    public static final double kPivotArmD = 0.1; // 0.1
    public static final double kPivotArmS = 0.07;
    public static final double kPivotArmV = 0.2; // 0.2
    public static final double kPivotArmA = 0.015; // 0.015
    public static final double kPivotArmFF = -0.13;

    public static double kPivotArmMMVelo = 40;
    public static double kPivotArmMMAcc = 30;
    public static double kPivotArmMMJerk = 1600;

    public static final Rotation2d kPivotArmMin = Rotation2d.fromDegrees(-110);
    public static final Rotation2d kPivotArmMax = Rotation2d.fromDegrees(120);

    public static final Rotation2d kPivotArmMaxManual = Rotation2d.fromDegrees(460);
    public static final Rotation2d kPivotArmMinManual = Rotation2d.fromDegrees(-270);

    /// The min/max angle of the pivot that will be rotating through the path of the
    /// elevator
    public static final Rotation2d kPivotArmSwingThroughMax = Rotation2d.fromDegrees(35);
    public static final Rotation2d kPivotArmSwingThroughMin = Rotation2d.fromDegrees(-35);

    // the Tolerance for pivot command motion
    public static final Rotation2d kPivotTolerance = Rotation2d.fromDegrees(5);
    public static final double kPivotToleranceRotations = kPivotTolerance.getRotations();

    // Pivot Angles for different states
    public static final double kPivotHome = 45;
    public static final double kPivotClimb = 90;
    public static final double kPivotCoralPickup = 120;
    public static final double kPivotCoralDropOff1 = -100;
    public static final double kPivotCoralDropOff = -72;
    public static final double kPivotCoralDropOff4 = -52;
    public static final double kPivotAlgaePickUpFloor = 116;
    public static final double kPivotReef = 102;
    public static final double kPivotAlgaePickUpFloorFlip = 90;
    public static final double kPivotReef2Flip = -35;
    public static final double kPivotReef3Flip = -25;
    public static final double kPivotAlgaeNetBlue = 60;
    public static final double kPivotAlgaeNetYellow = 0;
    public static final double kPivotAlgaeNetRaise = 25;
}
