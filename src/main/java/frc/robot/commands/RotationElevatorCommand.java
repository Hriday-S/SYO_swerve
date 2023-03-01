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
        m_startingAngle = calculateTheta(m_elevatorSubsystem.getWinchAbsPosition());
    }

    @Override
    public void execute() {
        int i = 1;
        if (i == 1) {
            init();
            i++;
        }

        m_elevatorSubsystem.rotate(Math.copySign(m_power, m_elevatorRotationAngle));
    }

    @Override
    public boolean isFinished() {
        if (calculateTheta(m_elevatorSubsystem.getDistanceRotated()) - m_startingAngle < Math.abs(m_elevatorRotationAngle)) {
            return false;
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.rotate(0);
        m_elevatorSubsystem.updatePositions();
    }

    public double calculateTheta(double distance) {
        double theta = Math.pow(0.83, 2) + Math.pow(0.91, 2) - Math.pow(distance, 2);
        theta /= (2 * 0.83 * 0.91);
        if (theta > 1) {
            theta = 1;
        } 
        else {}
        theta = Math.acos(theta);
        return Math.toDegrees(theta);
    }
}
