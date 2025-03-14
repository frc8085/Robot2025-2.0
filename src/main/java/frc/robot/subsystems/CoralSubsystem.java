// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.MotorDefaultsConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;

public class CoralSubsystem extends SubsystemBase {

  // import motor id
  private final SparkMax m_coralMotor = new SparkMax(CanIdConstants.kCoralCanId,
      MotorDefaultsConstants.NeoVortexMotorType);

  private double kSpeed = CoralConstants.kCoralSpeed;

  // light sensor
  DigitalInput lightSensor = new DigitalInput(CoralConstants.kIRPort);

  // robot starts with Coral
  private boolean coralTrue = true;

  public CoralSubsystem() {

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_coralMotor.configure(Configs.CoralManipulator.coralConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

  }

  public double getCurrent() {
    return m_coralMotor.getOutputCurrent();
  }

  public Boolean isCoralDetected() {
    return lightSensor.get();
  }

  /* When a coral is picked up, it's in the robot */
  public void coralPickedUp() {
    coralTrue = true;
  }

  /* Once a note is shot, it's not in robot */
  public void coralEjected() {
    coralTrue = false;
  }

  /* Give us a state when the note is in robot */
  public boolean coralInRobot() {
    return coralTrue;
  }

  public void pickup() {
    m_coralMotor.set(-kSpeed);
  }

  public void stop() {
    m_coralMotor.set(0);
  }

  public void eject() {
    m_coralMotor.set(kSpeed);
  }

  public void drop() {
    m_coralMotor.set(CoralConstants.kCoralSlowSpeed);
  }

  public void periodic() {
    // Put Indicator on Dashboard that a Note is in the Robot
    if (Constants.TuningModeConstants.kCoralTuning) {
      SmartDashboard.putBoolean("Coral Detected", isCoralDetected());
    }
  }

}