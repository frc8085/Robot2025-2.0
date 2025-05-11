package frc.robot.subsystems.Pivot;

import edu.wpi.first.math.geometry.Rotation2d;

public final class PivotArmConstants {

    // CanIDs for Pivot
    public static final int kPivotArmCanId = 25;
    public static final int kPivotArmCancoderCanId = 16;

    public static final double kPivotMotorGearRatio = 1; // Gear ratio is 27:1 but it's on the motor now
    public static double kPivotArmSpeed = .10;

    public static final double MOTOR_POSITION_TO_ANGLE_SCALING_FACTOR = 2 * Math.PI;

    public static final double kPivotMotorSensorToMechanismRatio = 3.0;
    public static final double kPivotMotorRotorToSensorRatio = 9.0;

    public static final double kPivotCancoderVerticalOffset = -.039795;// -0.032471
    public static final double kPivotCancoderOffset = 0.456543;

    public static final double kPivotArmP = 12; // 7
    public static final double kPivotArmI = 0; // 0.0
    public static final double kPivotArmD = 0; // 0
    public static final double kPivotArmS = 0.35;
    public static final double kPivotArmV = .75; // 0.3
    public static final double kPivotArmA = 0; // 0.015
    public static final double kPivotArmFF = .375; // .354
    public static final double kPivotArmG = 0; // 0

    public static final double kPivotArmGravityV = 0;

    public static double kPivotArmMMVelo = 80;
    public static double kPivotArmMMAcc = 80;
    public static double kPivotArmMMJerk = 1600;

    public static final Rotation2d kPivotArmMin = Rotation2d.fromDegrees(-290);
    public static final Rotation2d kPivotArmMax = Rotation2d.fromDegrees(290);

    public static final Rotation2d kPivotArmMaxManual = Rotation2d.fromDegrees(270);
    public static final Rotation2d kPivotArmMinManual = Rotation2d.fromDegrees(-270);

    /// The min/max angle of the pivot that will be rotating through the path of the
    /// elevator
    public static final Rotation2d kPivotArmSwingThroughMax = Rotation2d.fromDegrees(35);
    public static final Rotation2d kPivotArmSwingThroughMin = Rotation2d.fromDegrees(-35);

    // the Tolerance for pivot command motion
    public static final Rotation2d kPivotTolerance = Rotation2d.fromDegrees(10);
    public static final double kPivotToleranceRotations = kPivotTolerance.getRotations();

    // Pivot Angles for different states
    public static final double kPivotHome = 0;
    public static final double kPivotClimb = 90;
    public static final double kPivotCoralPickup = 0;
    // public static final double kPivotCoralDropOff1 = -45;
    public static final double kPivotCoralDropOff2And3 = 220;
    public static final double kPivotCoralScore2And3 = 235;
    public static final double kPivotCoralDropOff4 = 215;
    public static final double kPivotCoralScore4 = 235;
    // public static final double kPivotAlgaePickUpFloor = 116;
    public static final double kPivotAlgaeReef = 90;
    // public static final double kPivotAlgaePickUpFloorFlip = 90;
    // public static final double kPivotReef2Flip = -35;
    // public static final double kPivotReef3Flip = -25;
    public static final double kPivotAlgaeNet = 180;
    // public static final double kPivotAlgaeNetYellow = 0;
    // public static final double kPivotAlgaeNetRaise = 25;
    public static final double kPivotAlgaeProcessor = 108;

    public static final double kPivotCoralScoreHome = 180;
    public static final double kPivotTravel = -179;
}
