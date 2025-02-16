// package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.PivotArmConstants;

// public class PivotSubsystem extends SubsystemBase {
// private final TalonFX m_pivotMotor = new TalonFX(25, "rio");
// TalonFXConfiguration config = new TalonFXConfiguration();
// private final CANcoder m_pivotEncoder = new CANcoder(35);

// private MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);

// public PivotSubsystem() {

// m_pivotMotor.getConfigurator().apply(config);
// var slot0Configs = new Slot0Configs();
// slot0Configs.kP = PivotArmConstants.kPivotArmP;
// slot0Configs.kI = PivotArmConstants.kPivotArmI;
// slot0Configs.kD = PivotArmConstants.kPivotArmD;
// slot0Configs.kV = PivotArmConstants.kPivotArmV;
// slot0Configs.kA = PivotArmConstants.kPivotArmA;

// config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
// config.MotionMagic.MotionMagicCruiseVelocity = 25;
// config.MotionMagic.MotionMagicAcceleration = 60;
// config.MotionMagic.MotionMagicJerk = 1600;
// // expirementing with encoder
// // config.Feedback.

// m_pivotMotor.getConfigurator().apply(slot0Configs);

// }

// public void setPos(Rotation2d angle) {
// motionMagicControl.Position = angle.getRotations();
// m_pivotMotor.setControl(motionMagicControl);
// SmartDashboard.putNumber("rotation2d value", angle.getRotations());
// }

// public void setRotorPos(Rotation2d angle) {
// m_pivotMotor.setPosition(angle.getRotations());
// }

// public double getCurrentPosition() {
// return m_pivotMotor.get();
// }

// public void periodic() {
// // Get motor readings
// SmartDashboard.putNumber("currentPosition", getCurrentPosition());
// // get a number to
// double target = SmartDashboard.getNumber("Pivot Target", 0);

// setPos(Rotation2d.fromRotations(target));

// }

// public void start() {
// m_pivotMotor.set(0.2);
// }

// public void reverse() {
// m_pivotMotor.set(-0.2)
// }

// public void stop(0) {

// }
// }
