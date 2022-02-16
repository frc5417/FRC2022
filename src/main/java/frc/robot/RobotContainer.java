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
  // Define subsystems
  private final Climber climber;
  private final Intake intake;
  private final Drivetrain driveSubsystem;
  private final Shooter shooterSubsystem;
  private final Limelight limelightSubsystem;
  private final Turret turretSubSystem;

  // Define joysticks & buttons
  private Joystick pad;
  private Joystick padManipulator;
  private JoystickButton xButton;
  private JoystickButton buttonY;
  private JoystickButton buttonX;
  private JoystickButton buttonA;
  private JoystickButton buttonB;

  // Define commands
  private final AutoAlignTurret autoAlignTurretCommand;
  private final AutoSetShooterSpeed autoSetShooterSpeedCommand;
  private final AutoClimbExtend autoClimbExtend;
  private final AutoClimbExtend autoClimbExtendSlightly;
  private final AutoClimbRetract autoClimbRetract;
  private final AutoClimbRetract autoClimbRetractSlightly;
  private final ClimbAnchor climbAnchor;
  private final ClimbPivot climbPivot;
  private final AutoTankDrive autoTankDrive;
  private final Command runIntakeSystem;
  private final Command deployIntakePistons;
  private final Command retractIntakePistons;
  private final Command internalPush;

  // Command Groups:
  private final SequentialCommandGroup climbCommands;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // init subsystems
    this.climber = new Climber();
    this.intake = new Intake();
    this.driveSubsystem = new Drivetrain();
    this.shooterSubsystem = new Shooter();
    this.limelightSubsystem = new Limelight();
    this.turretSubSystem = new Turret();

    // init commands
    this.autoClimbExtend = new AutoClimbExtend(this.climber);
    this.autoClimbExtendSlightly = new AutoClimbExtend(this.climber, Constants.climberExtendSlightlyPos);
    this.autoClimbRetract = new AutoClimbRetract(this.climber);
    this.autoClimbRetractSlightly = new AutoClimbRetract(this.climber, Constants.climberRetractSlightlyPos);
    this.climbAnchor = new ClimbAnchor(this.climber, this);
    this.climbPivot = new ClimbPivot(this.climber, this);
    this.autoTankDrive = new AutoTankDrive(this.driveSubsystem, this);
    this.runIntakeSystem = new RunIntakeSystem(this.intake);
    this.deployIntakePistons = new DeployIntakePistons(this.intake);
    this.retractIntakePistons = new RetractIntakePistons(this.intake);
    this.internalPush = new InternalPush(this.intake);
    this.autoAlignTurretCommand = new AutoAlignTurret(this.limelightSubsystem, this.turretSubSystem);
    this.autoSetShooterSpeedCommand = new AutoSetShooterSpeed(this.limelightSubsystem, this.shooterSubsystem);

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
    this.padManipulator = new Joystick(1);

    this.buttonY = new JoystickButton(this.pad, 4);
    this.buttonY.whileHeld(this.runIntakeSystem);

    this.buttonX = new JoystickButton(this.pad, 3);
    this.buttonX.whenPressed(this.deployIntakePistons);

    this.buttonA = new JoystickButton(this.pad, 1);
    this.buttonA.whenPressed(this.retractIntakePistons);
    
    this.buttonB = new JoystickButton(this.pad, 2);
    this.buttonB.whileHeld(this.internalPush);

    this.xButton  = new JoystickButton(this.pad, 3);
    this.xButton.toggleWhenPressed(this.climbCommands);
  }

  public Joystick getDriverPad(){
    return this.pad;
  }

  public void makeItDrive(){
    driveSubsystem.setPower(this.pad);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //this runs two auton commands- Auton2 and Auton3 with their ramsete commands
    //return new SequentialCommandGroup(new Auton2(driveSubsystem).getRamseteCommand(), new Auton3(driveSubsystem).getRamseteCommand());
    
    //this just runs Auton2
    return new SequentialCommandGroup(new Auton2(driveSubsystem).getRamseteCommand());
    //add shooting to sequential command
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

  public Shooter getShooter() {
    return this.shooterSubsystem;
  }

  public Limelight getLimelight() {
    return this.limelightSubsystem;
  }

}
