package frc.robot.subsystems.EndEffector;

public final class EndConstants {
    public static final int EndEffectCurrentLimit = 40;
    public static final double kEndEffectSpeed = 0.75;
    public static final double kEndEffectSlowSpeed = .5;

    // open loop motor values
    public static double kEndEffectMinOutput = -0.25;
    public static double kEndEffectMaxOutput = 0.25;

    // direction motor runs
    public static boolean kEndEffectInverted = true;

    // PID not used
    public static double kEndEffectP = 0.5;
    public static double kEndEffectI = 0.0;
    public static double kEndEffectD = 0.0;
    public static double kEndEffectFF = 0.0;

    // Beam Break Sensor
    public static int kIRPort = 3;

    // Current Limit for Coral Stall
    public static double kEndEffectCurrentLimitIdle = 15;
    public static double kEndEffectCurrentLimitHandOff = 50;
    public static double kEndEffectCurrentLimitEject = 20;

}
