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
    private double m_elevatorPulleyPosition = 0;
    private double m_winchPosition = 0;

    public ElevatorSubsystem() {
        m_elevatorPulley = new CANSparkMax(ELEVATOR_PULLEY_MOTOR, MotorType.kBrushless);
        m_winch = new CANSparkMax(WINCH_MOTOR, MotorType.kBrushless);
        m_elevatorPulley.setIdleMode(IdleMode.kBrake);
        m_winch.setIdleMode(IdleMode.kBrake);
        m_elevatorPulley.setInverted(true);

        m_elevatorPulleyEncoder = m_elevatorPulley.getEncoder();
        m_winchEncoder = m_winch.getEncoder();
        m_elevatorPulleyEncoder.setPositionConversionFactor(0.01); // FIXME Convert to meters
        m_winchEncoder.setPositionConversionFactor(0.002356); // Convert from rotations to meters
        m_elevatorPulleyEncoder.setPosition(m_elevatorPulleyPosition);
        m_winchEncoder.setPosition(m_winchPosition);
    }

    public void extend(double elevatorPulleySpeed) {
        m_elevatorPulleySpeed = elevatorPulleySpeed;
    }

    public void rotate(double winchSpeed) {
        m_winchSpeed = winchSpeed;
    }

    public void updatePositions() {
        m_elevatorPulleyPosition = m_elevatorPulleyEncoder.getPosition();
        m_winchPosition = m_winchEncoder.getPosition();
    }

    public double getElevatorAbsPosition() {
        return m_elevatorPulleyEncoder.getPosition();
    }

    public double getWinchAbsPosition() {
        return m_winchEncoder.getPosition();
    }

    public double getDistanceTravelled() {
        return Math.abs(m_elevatorPulleyEncoder.getPosition() - m_elevatorPulleyPosition);
    }

    public double getDistanceRotated() {
        return Math.abs(m_winchEncoder.getPosition() - m_winchPosition);
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
}
