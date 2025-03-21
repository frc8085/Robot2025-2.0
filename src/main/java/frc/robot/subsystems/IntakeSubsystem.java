package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.MotorDefaultsConstants;
import frc.robot.Constants.TuningModeConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax m_liftMotor = new SparkMax(CanIdConstants.kIntakeLiftCanId,
            MotorDefaultsConstants.NeoMotorType);
    SparkMaxConfig config = new SparkMaxConfig();
    private RelativeEncoder m_liftEncoder;
    private SparkClosedLoopController m_liftPIDController;

    private final SparkMax m_OuterMotor = new SparkMax(CanIdConstants.kIntakeOuterCanId,
            MotorDefaultsConstants.NeoMotorType);
    private final SparkMax m_InnerMotor = new SparkMax(CanIdConstants.kIntakeInnerCanId,
            MotorDefaultsConstants.NeoMotorType);

    DigitalInput lightSensor1 = new DigitalInput(IntakeConstants.kIRPort1);
    DigitalInput lightSensor2 = new DigitalInput(IntakeConstants.kIRPort2);

    private double kOuterSpeed = IntakeConstants.kOuterSpeed;
    private double kInnerSpeed = IntakeConstants.kInnerSpeed;

    public IntakeSubsystem() {

        if (IntakeConstants.kInOutInverted == true) {
            kOuterSpeed = -IntakeConstants.kOuterSpeed;
            kInnerSpeed = -IntakeConstants.kInnerSpeed;
        }

        m_liftEncoder = m_liftMotor.getEncoder();
        m_liftPIDController = m_liftMotor.getClosedLoopController();
        m_liftMotor.configure(Configs.IntakeConfigs.liftConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        // not sure if two configs were entirely necessary, since they are the exact
        // same, but I did it anyway
        m_OuterMotor.configure(Configs.IntakeConfigs.outerConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_InnerMotor.configure(Configs.IntakeConfigs.innerConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void intakeDown() {
        m_liftPIDController.setReference(IntakeConstants.kLiftDownPosition, ControlType.kPosition);
    }

    public void intakeUp() {
        m_liftPIDController.setReference(IntakeConstants.kLiftUpPosition, ControlType.kPosition);
    }

    public void intakeOn() {
        m_OuterMotor.set(kOuterSpeed);
        m_InnerMotor.set(kInnerSpeed);
    }

    public void intakeOff() {
        m_OuterMotor.set(0);
        m_InnerMotor.set(0);
    }

    public boolean isCoralDetected() {
        return lightSensor1.get() && lightSensor2.get();
    }
}