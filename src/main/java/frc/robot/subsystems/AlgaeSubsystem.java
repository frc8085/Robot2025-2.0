package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.MotorDefaultsConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;

public class AlgaeSubsystem extends SubsystemBase{
    
    // import motor id
    private final SparkMax m_coralMotor = new SparkMax(CanIdConstants.kAlgaeCanId, MotorDefaultsConstants.Neo550MotorType);

    private double kSpeed = AlgaeConstants.speed;

    public AlgaeSubsystem () {

// Apply the respective configurations to the SPARKS. Reset parameters before
        // applying the configuration to bring the SPARK to a known good state. Persist
        // the settings to the SPARK to avoid losing them on a power cycle.
        m_coralMotor.configure(Configs.CoralManipulator.coralConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

  }

  public void pickup() {
    m_coralMotor.set(kSpeed);
  }

  public void stop() {
    m_coralMotor.set(0);
  }

  public void eject() {
    m_coralMotor.set(-kSpeed);
  }

    }
}
