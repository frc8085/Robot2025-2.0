package frc.robot.subsystems.Coral;

public final class CoralConstants {

    public static final int kCoralCanId = 21;

    public static final int coralCurrentLimit = 40;
    public static final double kCoralSpeed = 1;
    public static final double kCoralSlowSpeed = .5;

    // open loop motor values
    public static double kCoralMinOutput = -0.25;
    public static double kCoralMaxOutput = 0.25;

    // direction motor runs
    public static boolean kCoralInverted = true;

    // PID not used
    public static double kCoralP = 0.5;
    public static double kCoralI = 0.0;
    public static double kCoralD = 0.0;
    public static double kCoralFF = 0.0;

    // Beam Break Sensor
    public static int kIRPort = 3;

    // Current Limit for Coral Stall
    public static double kCoralCurrentLimit = 20;
}
