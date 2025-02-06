package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MotorDefaultsConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX m_elevatorMotor = new TalonFX(24, "rio"); //change deviceID and canbus
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    private double kSpeed = ElevatorConstants.kElevatorSpeed;

    public ElevatorSubsystem() {
        //Add configurations to Configs.java
        m_elevatorMotor.getConfigurator().apply(config);
        config.Slot0.kP = ElevatorConstants.kElevatorP;
        config.Slot0.kI = ElevatorConstants.kElevatorI;
        config.Slot0.kD = ElevatorConstants.kElevatorD;
    }

    public void moveUp() {
        m_elevatorMotor.set(kSpeed);
      }
    
      public void stop() {
        m_elevatorMotor.set(0);
      }
    
      public void moveDown() {
        m_elevatorMotor.set(-kSpeed);
      }
}
