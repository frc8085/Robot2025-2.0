package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class PivotSubsystem {
  private final TalonFX m_pivotMotor new talonFX();
    TalonFXConfiguration config = new TalonFXConfiguration();
    private final CANcoder m_elevatorEncoder = new CANcoder(23);

}
