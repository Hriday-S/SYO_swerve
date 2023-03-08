package frc.robot.subsystems;

import static frc.robot.Constants.WINCH_MOTOR;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WinchSubsystem extends SubsystemBase {
    private CANSparkMax m_winch;
    private double m_winchSpeed = 0;

    private RelativeEncoder m_winchEncoder;

    public WinchSubsystem() {
        m_winch = new CANSparkMax(WINCH_MOTOR, MotorType.kBrushless);
        m_winch.setIdleMode(IdleMode.kBrake);

        m_winchEncoder = m_winch.getEncoder();
        m_winchEncoder.setPositionConversionFactor(0.037037); // Convert from motor rotations to gear rotations
    }

    public void rotate(double winchSpeed) {
        m_winchSpeed = winchSpeed;
    }

    public double getWinchAbsPosition() {
        return m_winchEncoder.getPosition();
        //return 85 - calculateTheta(m_winchEncoder.getPosition() * Math.PI * calculateDiameter());
    }

    // Only resets when a match starts
    public void resetEncoders() {
        m_winchEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        m_winch.set(m_winchSpeed);
    }

    /*
    public double calculateTheta(double distance) {
        double theta = Math.pow(0.8, 2) + Math.pow(0.96, 2) - Math.pow(distance, 2);
        theta /= (2 * 0.83 * 0.91);
        if (theta > 1) {
            theta = 1;
        }
        theta = Math.acos(theta);
        return Math.toDegrees(theta);
    }

    public double calculateDiameter() {
        return 0.04 - (0.0018 * m_winchEncoder.getPosition());
    }
    */
}
