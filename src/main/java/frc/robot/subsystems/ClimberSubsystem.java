package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.MotorDefaultsConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkFlex m_winchMotor = new SparkFlex(CanIdConstants.kClimberCanId,
            MotorDefaultsConstants.NeoVortexMotorType);
    private SparkLimitSwitch m_winchLowerLimit;

    private double kSpeed = ClimberConstants.kWinchSpeed;

    public ClimberSubsystem() {
        m_winchMotor.configure(Configs.Climber.climberConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_winchLowerLimit = m_winchMotor.getReverseLimitSwitch();
    }

    public boolean climberAtHomePosition() {
        return m_winchLowerLimit.isPressed();
    }

    // turn off climber
    public void stop() {
        m_winchMotor.set(0);
    }

    // open loop move climber up & down
    public void moveDown() {
        m_winchMotor.set(-kSpeed);
    }

    public void moveUp() {
        m_winchMotor.set(1);
    }

    public void periodic() {
        if (Constants.TuningModeConstants.kClimberTuning) {
            SmartDashboard.putBoolean("Climber Limit Switch", climberAtHomePosition());
        }
    }

}
