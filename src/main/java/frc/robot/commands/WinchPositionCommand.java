package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class WinchPositionCommand extends CommandBase {
    private final ElevatorSubsystem m_elevatorSubsystem;

    private double m_startAngle;
    private double m_targetAngle;
    private final double m_power;

    public WinchPositionCommand(ElevatorSubsystem elevatorSubsystem, String position, double power) {
        m_elevatorSubsystem = elevatorSubsystem;
        if (position.equals("DRIVE")) {
            m_targetAngle = 80;
        } else if (position.equals("OUT")) {
            m_targetAngle = 53;
        } else if (position.equals("IN")) {
            m_targetAngle = 10;
        }
        m_power = power;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        m_startAngle = m_elevatorSubsystem.getWinchAbsPosition();
        m_elevatorSubsystem.rotate(Math.copySign(m_power, -(m_targetAngle - m_startAngle)));
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(m_elevatorSubsystem.getWinchAbsPosition() - m_startAngle) < Math.abs(m_targetAngle - m_startAngle)) {
            return false;
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.rotate(0);
    }
}
