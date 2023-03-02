package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class IdleElevatorCommand extends CommandBase {
    private final ElevatorSubsystem m_elevatorSubsystem;

    private double m_currentElevatorPosition;
    private double m_currentWinchPosition;
    PIDController m_pid;

    public IdleElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;
        
        m_pid = new PIDController(1, 0, 0);

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        m_currentElevatorPosition = m_elevatorSubsystem.getElevatorAbsPosition();
        m_currentWinchPosition = m_elevatorSubsystem.getWinchAbsPosition();

    }

    @Override
    public void execute() {
        m_elevatorSubsystem.extend(m_pid.calculate(m_elevatorSubsystem.getElevatorAbsPosition(), m_currentElevatorPosition));
        m_elevatorSubsystem.rotate(m_pid.calculate(m_elevatorSubsystem.getWinchAbsPosition(), m_currentWinchPosition));
    }
}
