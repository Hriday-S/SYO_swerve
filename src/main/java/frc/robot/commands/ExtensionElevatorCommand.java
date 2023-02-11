package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ExtensionElevatorCommand extends CommandBase {
    private final ElevatorSubsystem m_elevatorSubsystem;

    private final double m_elevatorPulleyDistance;
    private final double m_power;

    public ExtensionElevatorCommand(ElevatorSubsystem elevatorSubsystem, double elevatorPulleyDistance, double power) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_elevatorPulleyDistance = elevatorPulleyDistance;
        m_power = power;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        m_elevatorSubsystem.extend(Math.copySign(m_power, m_elevatorPulleyDistance));
    }

    @Override
    public boolean isFinished() {
        if (m_elevatorSubsystem.getDistanceTravelled() < Math.abs(m_elevatorPulleyDistance)) {
            return false;
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.extend(0);
        m_elevatorSubsystem.updatePositions();
    }
}
