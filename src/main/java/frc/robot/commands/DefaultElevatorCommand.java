package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class DefaultElevatorCommand extends CommandBase {
    private final ElevatorSubsystem m_elevatorSubsystem;

    private final DoubleSupplier m_elevatorPulleySpeed;
    private final DoubleSupplier m_winchSpeed;

    public DefaultElevatorCommand(ElevatorSubsystem elevatorSubsystem, DoubleSupplier elevatorPulleySpeed, DoubleSupplier winchSpeed) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_elevatorPulleySpeed = elevatorPulleySpeed;
        m_winchSpeed = winchSpeed;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        m_elevatorSubsystem.extend(m_elevatorPulleySpeed.getAsDouble());
        m_elevatorSubsystem.rotate(m_winchSpeed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.extend(0);
        m_elevatorSubsystem.rotate(0);
    }
}
