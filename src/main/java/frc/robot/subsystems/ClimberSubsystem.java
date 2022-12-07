package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.*;

public class ClimberSubsystem extends SubsystemBase 
{
    private CANSparkMax m_climber;
    private double power;

    public ClimberSubsystem ()
    {
        m_climber = new CANSparkMax(CLIMBER_MOTOR, MotorType.kBrushed);
        power = 0;
    }

    public void rotate (double suppliedPower)
    {
        if (Math.abs(suppliedPower) > 0.4)
        {
            power = Math.copySign(0.4, suppliedPower);
        }
        else
        {
            power = suppliedPower;
        }  
    }

    @Override
    public void periodic ()
    {
        m_climber.set(power);
    }
}
