// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drivetrain driveSubsystem;
  
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final TankDrive tankDrive;

  public Joystick pad;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    driveSubsystem = new Drivetrain();
    tankDrive = new TankDrive(this, driveSubsystem);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //this initalizes the joystick
    pad = new Joystick(0);
  }
  public double leftSpeed() {
    //this gets the speed for the left motors by returning the value of the joystick's axis, specifically axis 1 on the Xbox controller
    return pad.getRawAxis(1);
  }
  public double rightSpeed() {
    //this gets the speed for the right motors by returning the value of the joystick's axis, specifically axis 5 on the Xbox controller
    return pad.getRawAxis(5);
  }

  public Drivetrain getDriveSubsystem() {
    //this returns the drive subsystem in the robot container
    return driveSubsystem;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
