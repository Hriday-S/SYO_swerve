// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DefaultDriveCommand;
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
  //private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  /*
  private final AHRS m_navx = m_drivetrainSubsystem.getNavx();

  private double currentX = 0;
  private double currentY = 0;
  private double currentAngle = 0;
  */

  private final Joystick m_controller = new Joystick(0);

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
            () -> -modifyAxis(m_controller.getRawAxis(1)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getRawAxis(0)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getRawAxis(4)) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));
    //m_climberSubsystem.setDefaultCommand(new DefaultClimberCommand(m_climberSubsystem, m_controller.getRawAxis(2) - m_controller.getRawAxis(3)));

    // Configure the button bindings
    configureButtonBindings();
    //configureNavx();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //A button resets Gyro
    new Button(() -> m_controller.getRawButton(1)).whenPressed(m_drivetrainSubsystem::zeroGyroscope);
  }

  /*
  private void configureNavx() {
    m_drivetrainSubsystem.calibrateNavx();
  }

  public void translate(double x, double y, double power) {
    currentX = m_navx.getDisplacementX();
    currentY = m_navx.getDisplacementY();
    while (!(Math.abs(x - m_navx.getDisplacementX() - currentX) < 0.1 || Math.abs(y - m_navx.getDisplacementY() - currentY) < 0.1)) {
      double relativeX = m_navx.getDisplacementX() - currentX;
      double relativeY = m_navx.getDisplacementY() - currentY;
      double distance = Math.sqrt(Math.pow(x - relativeX, 2) + Math.pow(y - relativeY, 2));
      driveCommand(-modifyAxis((x - relativeX)/distance) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * power, -modifyAxis((y - relativeY)/distance) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * power, 0).schedule();
    }
    driveCommand(0, 0, 0).schedule();
  }

  public void rotate(double angle, double power) {
    currentAngle = m_navx.getAngle();
    while (!(Math.abs(angle - m_navx.getAngle() - currentAngle) < 5)) {
      double relativeAngle = m_navx.getAngle() - currentAngle;
      driveCommand(0, 0, -modifyAxis(Math.copySign(0.5, angle - relativeAngle)) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * power).schedule();
    }
    driveCommand(0, 0, 0).schedule();
  }

  public void idle(long ms) {
    long start = System.currentTimeMillis();
    while (System.currentTimeMillis() < start + ms) {
      driveCommand(0, 0, 0).schedule();
    }
  }
  
  public Command driveCommand(double x, double y, double theta) {
    return new DefaultDriveCommand(
      m_drivetrainSubsystem,
      () -> x,
      () -> y,
      () -> theta);
  }
  */

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

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
