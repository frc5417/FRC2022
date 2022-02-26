// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems:
  private final Shooter shooterSubsystem;
  private final Limelight limelightSubsystem;
  private final Turret turretSubSystem;
  private final Drive drive;

  // Commands:
  private final AutoAlignTurret autoAlignTurretCommand;
  private final AutoSetShooterSpeed autoSetShooterSpeedCommand;
  private final TankDrive tankDrive;

  // Joysticks:
  private final Joystick pad;
  private final JoystickButton buttonA;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Init Subsystems:
    this.shooterSubsystem = new Shooter();
    this.limelightSubsystem = new Limelight();
    this.turretSubSystem = new Turret();
    this.drive = new Drive();
    
    // Init Commands:
    this.autoAlignTurretCommand = new AutoAlignTurret(this.limelightSubsystem, this.turretSubSystem);
    this.autoSetShooterSpeedCommand = new AutoSetShooterSpeed(this.limelightSubsystem, this.shooterSubsystem);
    this.tankDrive = new TankDrive(this, this.drive);

    // Init Joysticks:
    this.pad = new Joystick(0);
    this.buttonA = new JoystickButton(this.pad, 1);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    this.tankDrive.schedule();
    buttonA.whileHeld(new SequentialCommandGroup(
      this.autoAlignTurretCommand,
      new InstantCommand(() -> System.out.println("aligned!!"))
    ));
  }

  public Joystick getDriverPad(){
    return this.pad;
  }

  public double getDriverLeftJoystick(){
    return this.pad.getRawAxis(1);
  }

  public double getDriverRightJoystick(){
    return this.pad.getRawAxis(5);
  }

  public boolean getButtonA() {
    return this.pad.getRawButton(1);
  }

  public boolean getButtonB() {
    return this.pad.getRawButton(2);
  }

  public boolean getButtonY() {
    return this.pad.getRawButton(4);
  }

  public boolean getButtonX() {
    return this.pad.getRawButton(3);
  }

  public boolean getBumperR() {
    return this.pad.getRawButton(6);
  }

  public boolean getBumperL() {
    return this.pad.getRawButton(5);
  }

  public Shooter getShooter() {
    return this.shooterSubsystem;
  }

  public Limelight getLimelight() {
    return this.limelightSubsystem;
  }

}
