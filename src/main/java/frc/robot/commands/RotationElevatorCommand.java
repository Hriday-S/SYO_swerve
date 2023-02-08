package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class RotationElevatorCommand extends CommandBase {
    private final ElevatorSubsystem m_elevatorSubsystem;

    private final double m_elevatorRotationAngle;

    public RotationElevatorCommand(ElevatorSubsystem elevatorSubsystem, double elevatorRotationAngle) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_elevatorRotationAngle = elevatorRotationAngle;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        m_elevatorSubsystem.rotate(Math.copySign(0.25, m_elevatorRotationAngle));
    }

    @Override
    public boolean isFinished() {
        if (m_elevatorSubsystem.getAngleRotated() < m_elevatorRotationAngle) {
            return false;
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.rotate(0);
    }
}
