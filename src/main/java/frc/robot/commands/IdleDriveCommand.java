package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class IdleDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private boolean m_isTimeRecorded;
    private long m_waitTime;
    private long m_recordedTime;

    public IdleDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               long msecs) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_isTimeRecorded = false;
        this.m_waitTime = msecs;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        if (!m_isTimeRecorded) {
            m_recordedTime = System.currentTimeMillis();
            m_isTimeRecorded = true;
        }
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {
        if (System.currentTimeMillis() < m_recordedTime + m_waitTime) {
            return false;
        }
        return true;
    }
}
