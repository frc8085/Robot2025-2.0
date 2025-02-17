package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class Elevator extends Command {

    private final ElevatorSubsystem m_elevatorSubsystem;
    // private double m_speed;
    private double m_targetHeight;
    private double m_tolerance = 1;

    public Elevator(ElevatorSubsystem elevatorSubsystem, double target_height) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_targetHeight = target_height;
        addRequirements(m_elevatorSubsystem);
    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.setPos(m_targetHeight);
    }

    @Override
    public boolean isFinished() {
        return m_elevatorSubsystem.atTarget(m_tolerance);
    }
}