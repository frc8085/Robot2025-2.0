package frc.robot.subsystems.Coral;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants;

public final class CoralConfig {
    public static final SparkFlexConfig coralConfig = new SparkFlexConfig();

    static {

        coralConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(Constants.MotorDefaultsConstants.NeoVortexCurrentLimit);
    }
}
