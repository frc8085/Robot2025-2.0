package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.MotorDefaultsConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimberSubsystem  extends SubsystemBase {
    private final SparkFlex m_winchMotor = new SparkFlex(0, null);
    private SparkAbsoluteEncoder m_winchEncoder;

    private double kSpeed = ClimberConstants.kWinchSpeed;

    public ClimberSubsystem() {
        m_winchMotor.configure(Configs.ClimberManipulator.climberConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }
}
