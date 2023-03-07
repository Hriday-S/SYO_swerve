package frc.robot.subsystems;

import static frc.robot.Constants.ELEVATOR_PULLEY_MOTOR;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private CANSparkMax m_elevatorPulley;
    private double m_elevatorPulleySpeed = 0;

    private RelativeEncoder m_elevatorPulleyEncoder;

    public ElevatorSubsystem() {
        m_elevatorPulley = new CANSparkMax(ELEVATOR_PULLEY_MOTOR, MotorType.kBrushless);
        m_elevatorPulley.setIdleMode(IdleMode.kBrake);
        m_elevatorPulley.setInverted(true);

        m_elevatorPulleyEncoder = m_elevatorPulley.getEncoder();
        m_elevatorPulleyEncoder.setPositionConversionFactor(0.01); // Convert to meters
    }

    public void extend(double elevatorPulleySpeed) {
        m_elevatorPulleySpeed = elevatorPulleySpeed;
    }

    public double getElevatorAbsPosition() {
        return m_elevatorPulleyEncoder.getPosition();
    }

    // Only resets when a match starts
    public void resetEncoders() {
        m_elevatorPulleyEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        m_elevatorPulley.set(m_elevatorPulleySpeed);
    }
}
