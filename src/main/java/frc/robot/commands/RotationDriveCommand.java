package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class RotationDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final double m_angleDistance;
    private final DoubleSupplier m_rotationSupplier;

    public RotationDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               double angle, 
                               double power) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_angleDistance = Math.abs(angle);
        this.m_rotationSupplier = () -> Math.copySign(1, angle) * power * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                    0,
                    0,
                    m_rotationSupplier.getAsDouble(),
                    m_drivetrainSubsystem.getGyroscopeRotation()
            )
        );
    }

    @Override
    public boolean isFinished() {
        if (m_drivetrainSubsystem.getAngleTravelled() < m_angleDistance) {
            return false;
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        m_drivetrainSubsystem.updateDriveEncoders();
        m_drivetrainSubsystem.updateAngle();
    }
}
