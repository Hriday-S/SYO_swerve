package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BalanceCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final PIDController m_pidX;
    private final PIDController m_pidY;

    public BalanceCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_pidX = new PIDController(1, 0, 0);
        this.m_pidY = new PIDController(1, 0, 0);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        double translationXSupplier = m_pidX.calculate(-m_drivetrainSubsystem.getRoll(), 0);
        double translationYSupplier = m_pidY.calculate(-m_drivetrainSubsystem.getPitch(), 0);
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
                new ChassisSpeeds(
                        translationXSupplier * m_drivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                        translationYSupplier * m_drivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                        0
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
