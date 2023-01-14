// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.TranslationDriveCommand;
import frc.robot.commands.RotationDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  private final Joystick m_controller = new Joystick(0);
  private static double m_powerCap = 0.5;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_controller.getRawAxis(1), m_powerCap) * (-m_controller.getRawAxis(3) + 1) * 0.5 * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getRawAxis(0), m_powerCap) * (-m_controller.getRawAxis(3) + 1) * 0.5 * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getRawAxis(2), m_powerCap) * (-m_controller.getRawAxis(3) + 1) * 0.25 * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    // Configure the button bindings
    configureButtonBindings();
  }

  public void reset() {
    m_drivetrainSubsystem.zeroGyroscope();
    m_drivetrainSubsystem.updateDistance();
  }

  public void setIdleMode(int mode) {
    m_drivetrainSubsystem.setIdleMode(mode);
  }

  public SequentialCommandGroup autonomousCommands() {
    return new SequentialCommandGroup(
        new TranslationDriveCommand(m_drivetrainSubsystem, 1, 1, 0.4),
        new RotationDriveCommand(m_drivetrainSubsystem, 90, 0.3),
        new TranslationDriveCommand(m_drivetrainSubsystem, -1, -1, 0.4),
        new RotationDriveCommand(m_drivetrainSubsystem, -90, 0.3)
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //A button resets Gyro
    Button m_resetGyro = new Button(() -> m_controller.getRawButton(3));
    m_resetGyro.whenPressed(m_drivetrainSubsystem::zeroGyroscope);

    // Holding down both triggers activates turbo speed
    Button m_turbo = new Button(() -> m_controller.getRawButton(1));
    m_turbo.whenPressed(() -> setTurbo(1.0));
    m_turbo.whenReleased(() -> setTurbo(0.5));
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  } 

  private static double modifyAxis(double value, double limit) {
    // Deadband
    value = deadband(value, 0.125);

    // Cube the axis and set the limit
    value = Math.pow(value, 3) * limit;

    return value;
  }

  private static void setTurbo(double power) 
  {
    m_powerCap = power;
  }
}
