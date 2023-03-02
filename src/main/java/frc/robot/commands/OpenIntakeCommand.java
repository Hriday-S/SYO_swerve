package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class OpenIntakeCommand extends CommandBase {
    private final IntakeSubsystem m_intakeSubsystem;
    
    public OpenIntakeCommand(IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;

        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void execute() {
        m_intakeSubsystem.open();
        end(false);
    }
}