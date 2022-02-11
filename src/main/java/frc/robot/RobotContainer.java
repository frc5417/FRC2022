// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Define joysticks
  private Joystick pad;
  private Joystick padManipulator;

  // Define subsystems
  private final Climber climber;

  // Define commands
  private final AutoClimbExtend autoClimbExtend;
  private final AutoClimbRetract autoClimbRetract;
  private final ClimbAnchor climbAnchor;
  private final ClimbPivot climbPivot;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // init subsystems
    this.climber = new Climber();

    // init commands
    this.autoClimbExtend = new AutoClimbExtend(this.climber);
    this.autoClimbRetract = new AutoClimbRetract(this.climber);
    this.climbAnchor = new ClimbAnchor(this.climber, this);
    this.climbPivot = new ClimbPivot(this.climber, this);

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
