package frc.robot.subsystems.Algae;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants;

public final class AlgaeConfig {
    public static final SparkFlexConfig algaeConfig = new SparkFlexConfig();

    static {
        algaeConfig.idleMode(IdleMode.kBrake)
                .smartCurrentLimit(Constants.MotorDefaultsConstants.NeoVortexCurrentLimit);

        algaeConfig.encoder
                .positionConversionFactor(AlgaeConstants.kAlgaePositionConversionFactor)
                .velocityConversionFactor(AlgaeConstants.kAlgaeVelocityConversionFactor);

        algaeConfig.closedLoop
                .outputRange(AlgaeConstants.kAlgaeMinOutput, AlgaeConstants.kAlgaeMaxOutput)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(AlgaeConstants.kAlgaeP, AlgaeConstants.kAlgaeI, AlgaeConstants.kAlgaeD,
                        AlgaeConstants.kAlgaeFF);
    }
}
