package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    private final TalonFX m_climberMotor = new TalonFX(Constants.CanIdConstants.kClimberCanId, "rio");
    TalonFXConfiguration config = new TalonFXConfiguration();

    private double kSpeed = ClimberConstants.kWinchSpeed;

    // get encoder data
    private StatusSignal<Angle> climberPosition;

    // Limit Switches
    DigitalInput climberLimitSwitch = new DigitalInput(5);

    public ClimberSubsystem() {

        // Setting Motor Configs
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // get sensor readings
        climberPosition = m_climberMotor.getPosition();

        // Apply Configs
        m_climberMotor.getConfigurator().apply(config);

    }

    public boolean climberAtHomePosition() {

        return climberLimitSwitch.get();

    }

    // approximately 150 rotations from limit switch to deployed

    // read the current climber encoder position
    public double getCurrentMotorPosition() {
        climberPosition.refresh();
        return climberPosition.getValueAsDouble();
    }

    // turn off climber
    public void stop() {
        m_climberMotor.set(0);
    }

    // open loop move climber up & down
    public void moveDown() {
        m_climberMotor.set(-kSpeed);
    }

    public void moveUp() {
        m_climberMotor.set(1);
    }

    public void periodic() {
        SmartDashboard.putBoolean("Climber Limit Switch", climberAtHomePosition());
        SmartDashboard.putNumber("Climber Encoder", getCurrentMotorPosition());
    }

}
