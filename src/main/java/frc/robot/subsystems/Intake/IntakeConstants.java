package frc.robot.subsystems.Intake;

import edu.wpi.first.math.geometry.Rotation2d;

public final class IntakeConstants {

    public static final int kIntakeDeployCanId = 27;
    public static final int kIntakeOuterRollerCanId = 28;
    public static final int kIntakeInnerRollerCanId = 29;
    public static final int kIntakeDeployCanCoderCanId = 17;
    public static final double kCanCoderOffset = -0.083984;

    public static final double kIntakeSpeed = 1;
    public static final double kIntakeHandoffSpeed = 1;
    public static final double kIntakeInnerSpeed = .3;
    public static final double kIntakeRestingSpeed = .4;

    public static final double kIntakeDeployGearRatio = 48 / 1.0; // this is not used other than for zeroing with
                                                                  // cancoder

    public static final double kIntakeDeployP = 5; // 0.7
    public static final double kIntakeDeployI = 0; // 0.0
    public static final double kIntakeDeployD = 0; // 0.1
    public static final double kIntakeDeployS = 0.0;
    public static final double kIntakeDeployV = 0.09; // 0.2
    public static final double kIntakeDeployA = 0.0; // 0.015

    public static final double kIntakeDeployMMVelo = 140;
    public static final double kIntakeDeployMMAcc = 200;
    public static final double kIntakeDeployMMJerk = 1600;

    public static final int kLeftLightSensorDIO = 4;
    public static final int kRightLightSensorDIO = 5;

    public static final Rotation2d kIntakeInAngle = Rotation2d.fromRotations(40);
    public static final Rotation2d kIntakeEjectAngle = Rotation2d.fromRotations(28);
    public static final Rotation2d kIntakeOutAngle = Rotation2d.fromRotations(4);

    public static final Rotation2d kIntakeTolerance = Rotation2d.fromDegrees(15);
}
