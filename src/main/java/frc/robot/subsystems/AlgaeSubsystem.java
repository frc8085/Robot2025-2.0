package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.MotorDefaultsConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class AlgaeSubsystem extends SubsystemBase{
    
    // import motor id
    private final SparkMax m_algaeMotor = new SparkMax(CanIdConstants.kAlgaeCanId, MotorDefaultsConstants.Neo550MotorType);

    SparkMaxConfig config = new SparkMaxConfig();


    public AlgaeSubsystem () {

        // Apply the respective configurations to the SPARKS. Reset parameters before
        // applying the configuration to bring the SPARK to a known good state. Persist
        // the settings to the SPARK to avoid losing them on a power cycle.
        m_algaeMotor.configure(Configs.AlgaeManipulator.algaeConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

            config
                .inverted(AlgaeConstants.kAlgaeInverted)
                .idleMode(IdleMode.kBrake);
            config.encoder
                .positionConversionFactor(AlgaeConstants.kAlgaePositionConversionFactor)
                .velocityConversionFactor(AlgaeConstants.kAlgaeVelocityConversionFactor);
            config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(AlgaeConstants.kAlgaeP, AlgaeConstants.kAlgaeI, AlgaeConstants.kAlgaeD);
                
                m_algaeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void pickup() {
    m_algaeMotor.set(AlgaeConstants.kAlgaeSpeed);
  }

  public void stop() {
    m_algaeMotor.set(0);
  }

  public void eject() {
    m_algaeMotor.set(-AlgaeConstants.kAlgaeSpeed);
  }

}
