package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class DefaultIntakeCommand extends CommandBase {
    private final IntakeSubsystem m_intakeSubsystem;
    
    private final double m_wristSpeed;

    public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem, double wristSpeed) {
        m_intakeSubsystem = intakeSubsystem;
        m_wristSpeed = wristSpeed;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        m_intakeSubsystem.rotate(m_wristSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.rotate(0);
    }
}
