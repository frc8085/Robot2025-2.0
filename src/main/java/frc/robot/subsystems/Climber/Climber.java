package frc.robot.subsystems.Climber;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants;

public final class Climber {
        public static final SparkFlexConfig climberConfig = new SparkFlexConfig();

        static {

                climberConfig
                                .idleMode(IdleMode.kBrake)
                                .inverted(true)
                                .smartCurrentLimit(Constants.MotorDefaultsConstants.NeoVortexCurrentLimit);
                climberConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                // These are example gains you may need to them for your own robot!
                                .pid(ClimberConstants.kWinchP, ClimberConstants.kWinchI,
                                                ClimberConstants.kWinchD)
                                .velocityFF(ClimberConstants.kWinchFF)
                                .outputRange(ClimberConstants.kWinchMinOutput,
                                                ClimberConstants.kWinchMaxOutput);
        }
}
