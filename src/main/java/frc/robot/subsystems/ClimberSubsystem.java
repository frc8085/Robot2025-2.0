package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;


import frc.robot.Configs;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.MotorDefaultsConstants;

public class ClimberSubsystem {
    
    // motor id
    private final SparkFlex m_climberMotor = new SparkFlex(CanIdConstants.kClimberCanId, MotorDefaultsConstants.NeoVortexMotorType);

    private double kSpeed = ClimberConstants.kClimberSpeed;

    public ClimberSubsystem () {
    
    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_climberMotor.configure(Configs.Climber.climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  // forgot what it was called
  public void start() {
    m_climberMotor.set(kSpeed);
  }

  public void reverse() {
    m_climberMotor.set(-kSpeed);
  }

  public void stop() {
    m_climberMotor.set(0);
  }
  

}
