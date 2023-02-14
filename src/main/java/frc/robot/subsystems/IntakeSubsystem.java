package frc.robot.subsystems;

import static frc.robot.Constants.INTAKE_MOTOR_1;
import static frc.robot.Constants.INTAKE_MOTOR_2;
import static frc.robot.Constants.CLAW_SOLENOID_FORWARD;
import static frc.robot.Constants.CLAW_SOLENOID_REVERSE;
import static frc.robot.Constants.RELEASE_SOLENOID;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private DoubleSolenoid m_claw;

    private CANSparkMax m_intake1;
    private CANSparkMax m_intake2;
    private MotorControllerGroup m_intake;
    private Solenoid m_release;

    public IntakeSubsystem() {
        m_claw = new DoubleSolenoid(PneumaticsModuleType.REVPH, CLAW_SOLENOID_FORWARD, CLAW_SOLENOID_REVERSE);

        m_intake1 = new CANSparkMax(INTAKE_MOTOR_1, MotorType.kBrushed);
        m_intake2 = new CANSparkMax(INTAKE_MOTOR_2, MotorType.kBrushed);
        m_intake2.setInverted(true);
        m_intake = new MotorControllerGroup(m_intake1, m_intake2);
        m_release = new Solenoid(PneumaticsModuleType.REVPH, RELEASE_SOLENOID);
    }

    public void close() {
        m_intake.set(0.3);
        m_claw.set(Value.kForward);
    }

    public void open() {
        m_intake.set(0);
        m_release.set(true);
        m_claw.set(Value.kReverse);
        m_release.set(false);
    }
}
