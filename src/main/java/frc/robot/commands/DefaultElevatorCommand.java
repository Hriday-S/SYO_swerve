package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class DefaultElevatorCommand extends CommandBase {
    private final ElevatorSubsystem m_elevatorSubsystem;

    private final double m_elevatorPulleySpeed;
    private final double m_winchSpeed;

    public DefaultElevatorCommand(ElevatorSubsystem elevatorSubsystem, double elevatorPulleySpeed, double winchSpeed) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_elevatorPulleySpeed = elevatorPulleySpeed;
        m_winchSpeed = winchSpeed;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        m_elevatorSubsystem.extend(m_elevatorPulleySpeed);
        m_elevatorSubsystem.rotate(m_winchSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.extend(0);
        m_elevatorSubsystem.rotate(0);
    }
}
