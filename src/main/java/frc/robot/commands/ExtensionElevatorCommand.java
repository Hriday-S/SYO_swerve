package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ExtensionElevatorCommand extends CommandBase {
    private final ElevatorSubsystem m_elevatorSubsystem;

    private final double m_elevatorPulleyDistance;

    public ExtensionElevatorCommand(ElevatorSubsystem elevatorSubsystem, double elevatorPulleyDistance) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_elevatorPulleyDistance = elevatorPulleyDistance;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        m_elevatorSubsystem.extend(Math.copySign(0.25, m_elevatorPulleyDistance));
    }

    @Override
    public boolean isFinished() {
        if (m_elevatorSubsystem.getDistanceTravelled() < m_elevatorPulleyDistance) {
            return false;
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.extend(0);
    }
}
