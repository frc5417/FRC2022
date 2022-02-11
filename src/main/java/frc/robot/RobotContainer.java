// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Define joysticks & buttons
  private Joystick pad;
  private Joystick padManipulator;
  private JoystickButton xButton;

  // Define subsystems
  private final Climber climber;
  private final Drive drive;

  // Define commands
  private final AutoClimbExtend autoClimbExtend;
  private final AutoClimbExtend autoClimbExtendSlightly;
  private final AutoClimbRetract autoClimbRetract;
  private final AutoClimbRetract autoClimbRetractSlightly;
  private final ClimbAnchor climbAnchor;
  private final ClimbPivot climbPivot;
  private final AutoTankDrive autoTankDrive;

  private final SequentialCommandGroup climbCommands;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // init subsystems
    this.climber = new Climber();
    this.drive = new Drive();

    // init commands
    this.autoClimbExtend = new AutoClimbExtend(this.climber);
    this.autoClimbExtendSlightly = new AutoClimbExtend(this.climber, Constants.climberExtendSlightlyPos);
    this.autoClimbRetract = new AutoClimbRetract(this.climber);
    this.autoClimbRetractSlightly = new AutoClimbRetract(this.climber, Constants.climberRetractSlightlyPos);
    this.climbAnchor = new ClimbAnchor(this.climber, this);
    this.climbPivot = new ClimbPivot(this.climber, this);
    this.autoTankDrive = new AutoTankDrive(this.drive, this);

    this.climbCommands = new SequentialCommandGroup(
      this.autoClimbExtend,
      this.autoTankDrive,
      this.autoClimbRetract,
      this.climbAnchor,
      this.autoClimbExtendSlightly,
      this.climbPivot,
      this.autoClimbExtend,
      this.climbPivot,
      this.autoClimbRetractSlightly,
      this.climbAnchor,
      this.autoClimbRetract,
      this.climbAnchor,
      this.autoClimbExtendSlightly,
      this.climbPivot,
      this.autoClimbExtend,
      this.climbPivot,
      this.autoClimbRetractSlightly,
      this.climbAnchor,
      this.autoClimbRetract,
      this.climbAnchor
    );

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
    
    // Init the joysticks
    this.pad = new Joystick(0);
    this.padManipulator = new Joystick(1);
    this.xButton  = new JoystickButton(this.pad, 3); // Creates a new JoystickButton object for button 1

    this.xButton.toggleWhenPressed(this.climbCommands);
  }

  public boolean getButtonA() {
    return this.pad.getRawButton(1);
  }

  public boolean getButtonB() {
    return this.pad.getRawButton(2);
  }

  // Getters for subsystems:
  public Climber getClimber() {
		return this.climber;
	}

  // Getters for commands:
	public AutoClimbExtend getAutoClimbExtend() {
		return this.autoClimbExtend;
	}

	public AutoClimbRetract getAutoClimbRetract() {
		return this.autoClimbRetract;
	}

	public ClimbAnchor getClimbAnchor() {
		return this.climbAnchor;
	}

	public ClimbPivot getClimbPivot() {
		return this.climbPivot;
	}

}
