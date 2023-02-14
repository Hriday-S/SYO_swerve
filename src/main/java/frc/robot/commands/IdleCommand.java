package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class IdleCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final ElevatorSubsystem m_elevatorSubsystem;

    private boolean m_isTimeRecorded;
    private long m_waitTime;
    private long m_recordedTime;

    public IdleCommand(DrivetrainSubsystem drivetrainSubsystem,
                                ElevatorSubsystem elevatorSubsystem,
                                long msecs) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_elevatorSubsystem = elevatorSubsystem;
        this.m_isTimeRecorded = false;
        this.m_waitTime = msecs;

        addRequirements(drivetrainSubsystem, elevatorSubsystem);
    }

    @Override
    public void execute() {
        if (!m_isTimeRecorded) {
            m_recordedTime = System.currentTimeMillis();
            m_isTimeRecorded = true;
        }
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        m_elevatorSubsystem.extend(0);
        m_elevatorSubsystem.rotate(0);
    }

    @Override
    public boolean isFinished() {
        if (System.currentTimeMillis() < m_recordedTime + m_waitTime) {
            return false;
        }
        return true;
    }
}
