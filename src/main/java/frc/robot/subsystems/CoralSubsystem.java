// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.MotorDefaultsConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


public class CoralSubsystem extends SubsystemBase {
 
    // import motor id
    private final SparkMax m_coralMotor = new SparkMax(CanIdConstants.kCoralCanId, MotorDefaultsConstants.Neo550MotorType);
        
    SparkMaxConfig config = new SparkMaxConfig();
 
    public CoralSubsystem () {
    
        // Apply the respective configurations to the SPARKS. Reset parameters before
        // applying the configuration to bring the SPARK to a known good state. Persist
        // the settings to the SPARK to avoid losing them on a power cycle.
        m_coralMotor.configure(Configs.CoralManipulator.coralConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

            config
                .inverted(CoralConstants.kCoralInverted)
                .idleMode(IdleMode.kBrake);
            config.encoder
                .positionConversionFactor(CoralConstants.kCoralPositionConversionFactor)
                .velocityConversionFactor(CoralConstants.kCoralVelocityConversionFactor);
            config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(CoralConstants.kCoralP, CoralConstants.kCoarlI, CoralConstants.kCoralD);
                
                m_coralMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

public double kSpeed = CoralConstants.kCoralSpeed;

  public void pickup() {
    m_coralMotor.set(CoralConstants.kCoralSpeed);
  }

  public void stop() {
    m_coralMotor.set(0);
  }

  public void eject() {
    m_coralMotor.set(-CoralConstants.kCoralSpeed);
  }

}