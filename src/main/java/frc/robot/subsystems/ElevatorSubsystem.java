package frc.robot.subsystems;

import static frc.robot.Constants.ELEVATOR_PULLEY_MOTOR;
import static frc.robot.Constants.WINCH_MOTOR;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private CANSparkMax m_elevatorPulley;
    private CANSparkMax m_winch;
    private double m_elevatorPulleySpeed = 0;
    private double m_winchSpeed = 0;

    private RelativeEncoder m_elevatorPulleyEncoder;
    private RelativeEncoder m_winchEncoder;

    public ElevatorSubsystem() {
        m_elevatorPulley = new CANSparkMax(ELEVATOR_PULLEY_MOTOR, MotorType.kBrushless);
        m_winch = new CANSparkMax(WINCH_MOTOR, MotorType.kBrushless);
        m_elevatorPulley.setIdleMode(IdleMode.kBrake);
        m_winch.setIdleMode(IdleMode.kBrake);
        m_elevatorPulley.setInverted(true);

        m_elevatorPulleyEncoder = m_elevatorPulley.getEncoder();
        m_winchEncoder = m_winch.getEncoder();
        m_elevatorPulleyEncoder.setPositionConversionFactor(0.01); // Convert to meters
        m_winchEncoder.setPositionConversionFactor(0.05); // Convert from motor rotations to gear rotations
    }

    public void extend(double elevatorPulleySpeed) {
        m_elevatorPulleySpeed = elevatorPulleySpeed;
    }

    public void rotate(double winchSpeed) {
        m_winchSpeed = winchSpeed;
    }

    public double getElevatorAbsPosition() {
        return m_elevatorPulleyEncoder.getPosition();
    }

    public double getWinchAbsPosition() {
        return 90 - calculateTheta(m_winchEncoder.getPosition() * Math.PI * calculateDiameter());
    }

    // Only resets when a match starts
    public void resetEncoders() {
        m_elevatorPulleyEncoder.setPosition(0);
        m_winchEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        m_elevatorPulley.set(m_elevatorPulleySpeed);
        m_winch.set(m_winchSpeed);
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

    public double calculateDiameter() {
        return 0.043 - (0.0018 * m_winchEncoder.getPosition());
    }
}
