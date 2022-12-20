package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class DefaultClimberCommand extends CommandBase
{
    private final ClimberSubsystem m_climberSubsystem;
    private final double power;
    
    public DefaultClimberCommand(ClimberSubsystem climberSubsystem, double power)
    {
        m_climberSubsystem = climberSubsystem;
        this.power = power;
    }

    @Override
    public void execute()
    {
        m_climberSubsystem.rotate(power);
    }

    @Override
    public void end(boolean interrupted)
    {
        m_climberSubsystem.rotate(0);
    }
}
