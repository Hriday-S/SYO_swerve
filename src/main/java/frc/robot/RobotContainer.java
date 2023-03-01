// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CloseIntakeCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.TranslationDriveCommand;
import frc.robot.commands.RotationDriveCommand;
import frc.robot.commands.IdleCommand;
import frc.robot.commands.OpenIntakeCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.commands.DefaultElevatorCommand;
import frc.robot.commands.ExtensionElevatorCommand;
import frc.robot.commands.RotationElevatorCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  //private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private final Joystick m_driveController = new Joystick(0);
  private final Joystick m_operatorController = new Joystick(1);
  private static double m_drivePowerCap = 0.5;

  private final UsbCamera m_driveCamera;
  private final UsbCamera m_subsystemCamera;
  private final VideoSink m_cameraServer;

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
            () -> -modifyAxis(m_driveController.getRawAxis(1), 0.1, m_drivePowerCap) * (-m_driveController.getRawAxis(3) + 1) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_driveController.getRawAxis(0), 0.1, m_drivePowerCap) * (-m_driveController.getRawAxis(3) + 1) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_driveController.getRawAxis(2), 0.4, m_drivePowerCap) * (-m_driveController.getRawAxis(3) + 1) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    m_elevatorSubsystem.setDefaultCommand(new DefaultElevatorCommand(
            m_elevatorSubsystem, 
            () -> -m_operatorController.getRawAxis(1), 
            () -> -m_operatorController.getRawAxis(5)
    ));

    m_driveCamera = CameraServer.startAutomaticCapture(0);
    m_subsystemCamera = CameraServer.startAutomaticCapture(1);
    m_cameraServer = CameraServer.getServer();

    m_driveCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    m_subsystemCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    // Configure the button bindings
    configureButtonBindings();
  }

  public void reset() {
    m_elevatorSubsystem.resetEncoders();
    m_drivetrainSubsystem.zeroGyroscope();
    m_drivetrainSubsystem.updateDistance();
    m_elevatorSubsystem.updatePositions();
  }

  public void setIdleMode(int mode) {
    m_drivetrainSubsystem.setIdleMode(mode);
  }

  public SequentialCommandGroup autonomousCommands() {
    /*
    return new SequentialCommandGroup(
        new TranslationDriveCommand(m_drivetrainSubsystem, 0.5, 0.5, 0.25),
        new IdleDriveCommand(m_drivetrainSubsystem, 300),
        new TranslationDriveCommand(m_drivetrainSubsystem, -0.5, 0.5, 0.25),
        new IdleDriveCommand(m_drivetrainSubsystem, 300),
        new TranslationDriveCommand(m_drivetrainSubsystem, -0.5, -0.5, 0.25),
        new IdleDriveCommand(m_drivetrainSubsystem, 300),
        new TranslationDriveCommand(m_drivetrainSubsystem, 0.5, -0.5, 0.25)
    );
    */
    return new SequentialCommandGroup(
        new RotationElevatorCommand(m_elevatorSubsystem, 15, 0.1)
    );
    /*
    return new SequentialCommandGroup(
        new OpenIntakeCommand(m_intakeSubsystem),
        new IdleCommand(m_drivetrainSubsystem, m_elevatorSubsystem, 2500),
        new CloseIntakeCommand(m_intakeSubsystem)
    );
    */
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Driver bottom-left thumbpad button zeroes gyroscope
    Button m_resetGyro = new Button(() -> m_driveController.getRawButton(3));
    m_resetGyro.whenPressed(m_drivetrainSubsystem::zeroGyroscope);

    // Holding down driver trigger activates turbo speed
    Button m_turbo = new Button(() -> m_driveController.getRawButton(1));
    m_turbo.whenPressed(() -> setTurbo(1.0));
    m_turbo.whenReleased(() -> setTurbo(0.5));

    // Holding down side thumb button activates hard brakes
    Button m_brake = new Button(() -> m_driveController.getRawButton(2));
    m_brake.whenPressed(() -> setIdleMode(0));
    m_brake.whenReleased(() -> setIdleMode(1));

    // Operator 'A' button opens intake
    //Button m_openIntake = new Button(() -> m_operatorController.getRawButton(1));
    //m_openIntake.whenPressed(new OpenIntakeCommand(m_intakeSubsystem));

    // Operator 'X' button closes intake
    //Button m_closeIntake = new Button(() -> m_operatorController.getRawButton(2));
    //m_closeIntake.whenPressed(new CloseIntakeCommand(m_intakeSubsystem));

    // Driver bottom-left base button changes camera view
    Button m_driveView = new Button(() -> m_driveController.getRawButton(11));
    m_driveView.whenPressed(() -> m_cameraServer.setSource(m_driveCamera));

    // Driver bottom-right base button changes camera view
    Button m_subsystemView = new Button(() -> m_driveController.getRawButton(12));
    m_subsystemView.whenPressed(() -> m_cameraServer.setSource(m_subsystemCamera));
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

  private static double modifyAxis(double value, double deadband, double limit) {
    // Deadband
    value = deadband(value, deadband);

    // Cube the axis and set the limit
    value = Math.pow(value, 3) * limit;

    return value;
  }

  private static void setTurbo(double power) 
  {
    m_drivePowerCap = power;
  }
}
