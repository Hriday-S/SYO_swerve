package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class RotationElevatorCommand extends CommandBase {
    private final ElevatorSubsystem m_elevatorSubsystem;

    private final double m_elevatorRotationAngle;
    private double m_startingAngle;
    private final double m_power;

    public RotationElevatorCommand(ElevatorSubsystem elevatorSubsystem, double elevatorRotationAngle, double power) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_elevatorRotationAngle = elevatorRotationAngle;
        m_power = power;

        addRequirements(elevatorSubsystem);
    }

    public void init() {
        m_startingAngle = m_elevatorSubsystem.getWinchAbsPosition();
    }

    int i = 1;
    @Override
    public void execute() {
        if (i == 1) {
            init();
            i++;
        }

        m_elevatorSubsystem.rotate(Math.copySign(m_power, m_elevatorRotationAngle));
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(m_elevatorSubsystem.getWinchAbsPosition() - m_startingAngle) < Math.abs(m_elevatorRotationAngle)) {
            return false;
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.rotate(0);
    }
}
