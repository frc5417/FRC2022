// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.RunIntakeSystem;
import frc.robot.commands.DeployIntakePistons;
import frc.robot.commands.RetractIntakePistons;
import frc.robot.commands.InternalPush;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Intake intake = new Intake();

  private final Command runIntakeSystem = new RunIntakeSystem(this.intake);
  private final Command deployIntakePistons = new DeployIntakePistons(this.intake);
  private final Command retractIntakePistons = new RetractIntakePistons(this.intake);
  private final Command internalPush = new InternalPush(this.intake);

  private Joystick pad;
  private JoystickButton buttonY;
  private JoystickButton buttonX;
  private JoystickButton buttonA;
  private JoystickButton buttonB;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }
  
  public boolean yButton(){
    return pad.getRawButton(4);
  }

  public boolean xButton(){
    return pad.getRawButton(3);
  }

  public boolean aButton(){
    return pad.getRawButton(1);
  }

  public boolean bButton(){
    return pad.getRawButton(2);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    this.pad = new Joystick(0);

    this.buttonY = new JoystickButton(this.pad, 4);
    this.buttonY.whileHeld(this.runIntakeSystem);

    this.buttonX = new JoystickButton(this.pad, 3);
    this.buttonX.whenPressed(this.deployIntakePistons);

    this.buttonA = new JoystickButton(this.pad, 1);
    this.buttonA.whenPressed(this.retractIntakePistons);
    
    this.buttonB = new JoystickButton(this.pad, 2);
    this.buttonB.whileHeld(this.internalPush);

  }

}
