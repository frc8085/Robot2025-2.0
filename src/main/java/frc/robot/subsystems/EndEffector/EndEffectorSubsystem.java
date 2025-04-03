// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.MotorDefaultsConstants;
import frc.robot.subsystems.Coral.CoralConfig;
import frc.robot.subsystems.Coral.CoralConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;

public class EndEffectorSubsystem extends SubsystemBase {

  // import motor id
  private final SparkMax m_endMotor = new SparkMax(CoralConstants.kCoralCanId,
      MotorDefaultsConstants.NeoMotorType);

  private double kSpeed = EndConstants.kEndEffectSpeed;

  // light sensor
  // DigitalInput lightSensor = new DigitalInput(CoralConstants.kIRPort);

  // robot starts with Coral
  // private boolean coralTrue = true;

  public EndEffectorSubsystem() {

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    this.m_endMotor.configure(CoralConfig.coralConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    this.stop();
  }

  public double getCurrent() {
    return this.m_endMotor.getOutputCurrent();
  }

  // public Boolean isCoralDetected() {
  // return lightSensor.get();
  // }

  // /* When a coral is picked up, it's in the robot */
  // public void coralPickedUp() {
  // coralTrue = true;
  // }

  // /* Once a note is shot, it's not in robot */
  // public void coralEjected() {
  // coralTrue = false;
  // }

  /* Give us a state when the note is in robot */
  public boolean coralInRobot() {
    // return this.getCurrent() > EndConstants.kEndEffectCurrentLimitIdle;
    return false;
  }

  public void pickup() {
    this.m_endMotor.set(kSpeed);
  }

  public void stop() {
    this.m_endMotor.set(0.0);
  }

  public void holdCoral() {
    this.m_endMotor.set(.25);
  }

  public void eject() {
    this.m_endMotor.set(-kSpeed);
  }

  public void drop() {
    this.m_endMotor.set(-EndConstants.kEndEffectSlowSpeed);
  }

  public void periodic() {
    // Put Indicator on Dashboard that a Note is in the Robot
    // if (Constants.TuningModeConstants.kCoralTuning) {
    // SmartDashboard.putBoolean("Coral Detected", isCoralDetected());
    SmartDashboard.putNumber("End Effector Current", this.getCurrent());
    // }
  }

}