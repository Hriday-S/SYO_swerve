package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class RotationIntakeCommand extends CommandBase {
    private final IntakeSubsystem m_intakeSubsystem;
    
    private final double m_intakeAngle;
    private final double m_wristSpeed;

    public RotationIntakeCommand(IntakeSubsystem intakeSubsystem, double intakeAngle, double wristSpeed) {
        m_intakeSubsystem = intakeSubsystem;
        m_intakeAngle = intakeAngle;
        m_wristSpeed = wristSpeed;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        m_intakeSubsystem.rotate(Math.copySign(m_wristSpeed, m_intakeAngle));
    }

    @Override
    public boolean isFinished() {
        if (m_intakeSubsystem.getAngleRotated() < Math.abs(m_intakeAngle)) {
            return false;
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.rotate(0);
    }
}
