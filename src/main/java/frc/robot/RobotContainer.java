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
  private final Drive drive;
  private final Shooter shooter;
  private final Limelight limelight;
  private final Turret turret;

  // Define joysticks & buttons
  private Joystick pad;
  private JoystickButton buttonY;
  private JoystickButton buttonX;
  private JoystickButton buttonA;
  private JoystickButton buttonB;
  private JoystickButton bumperR;
  private JoystickButton bumperL;
  private boolean dpadRight;
  private boolean dpadLeft;
  private boolean dpadUp;
  private boolean dpadDown;

  private Joystick padManipulator;
  private JoystickButton buttonXManipulator;
  private JoystickButton buttonYManipulator;
  private JoystickButton buttonAManipulator;
  private JoystickButton buttonBManipulator;
  private JoystickButton bumperRManipulator;
  private JoystickButton bumperLManipulator;
  private boolean dpadRightManipulator;
  private boolean dpadLeftManipulator;
  private boolean dpadUpManipulator;
  private boolean dpadDownManipulator;

  // Define commands
  private final AutoAlignTurret autoAlignTurret;
  private final AutoSetShooterSpeed autoSetShooterSpeed;
  private final AutoClimbExtend autoClimbExtend;
  private final AutoClimbExtend autoClimbExtendSlightly;
  private final AutoClimbRetract autoClimbRetract;
  private final AutoClimbRetract autoClimbRetractSlightly;
  private final ClimbAnchor climbAnchor;
  private final ClimbPivot climbPivot;
  private final AutoTankDrive autoTankDrive;
  private final RunIntakeSystemForward runIntakeSystemForward;
  private final RunIntakeSystemBackward runIntakeSystemBackward;
  private final DeployIntakePistons deployIntakePistons;
  private final RetractIntakePistons retractIntakePistons;
  private final ManualAlignTurret manualAlignTurret;
  private final TankDrive tankDrive;

  // Command Groups:
  private final SequentialCommandGroup climbCommands;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // init joysticks
    this.pad = new Joystick(0);
    this.padManipulator = new Joystick(1);

    // init subsystems
    this.climber = new Climber();
    this.intake = new Intake();
    this.drive = new Drive();
    this.shooter = new Shooter();
    this.limelight = new Limelight();
    this.turret = new Turret();

    // init commands
    this.autoClimbExtend = new AutoClimbExtend(this.climber);
    this.autoClimbExtendSlightly = new AutoClimbExtend(this.climber, Constants.climberExtendSlightlyPos);
    this.autoClimbRetract = new AutoClimbRetract(this.climber);
    this.autoClimbRetractSlightly = new AutoClimbRetract(this.climber, Constants.climberRetractSlightlyPos);
    this.climbAnchor = new ClimbAnchor(this.climber, this);
    this.climbPivot = new ClimbPivot(this.climber, this);
    this.autoTankDrive = new AutoTankDrive(this.drive, this);
    this.runIntakeSystemForward = new RunIntakeSystemForward(this.intake);
    this.runIntakeSystemBackward = new RunIntakeSystemBackward(this.intake);
    this.deployIntakePistons = new DeployIntakePistons(this.intake);
    this.retractIntakePistons = new RetractIntakePistons(this.intake);
    this.autoAlignTurret = new AutoAlignTurret(this.limelight, this.turret);
    this.autoSetShooterSpeed = new AutoSetShooterSpeed(this.limelight, this.shooter, this.intake);
    this.manualAlignTurret = new ManualAlignTurret(this, this.turret);
    this.tankDrive = new TankDrive(this, this.drive);

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

    initializeButtons();
    configureButtonBindings();
  }
  

  public void initializeButtons() {


    this.buttonY = new JoystickButton(this.pad, 4);
    this.buttonB = new JoystickButton(this.pad, 2);
    this.buttonA = new JoystickButton(this.pad, 1);
    this.buttonX = new JoystickButton(this.pad, 3);
    this.bumperL = new JoystickButton(this.pad, 5);
    this.bumperR = new JoystickButton(this.pad, 6);
    this.dpadDown = this.pad.getPOV() == 180;
    this.dpadUp = this.pad.getPOV() == 0;
    this.dpadLeft = this.pad.getPOV() == 270;
    this.dpadRight = this.pad.getPOV() == 90;

    this.buttonYManipulator = new JoystickButton(this.padManipulator, 4);
    this.buttonBManipulator = new JoystickButton(this.padManipulator, 2);
    this.buttonAManipulator = new JoystickButton(this.padManipulator, 1);
    this.buttonXManipulator = new JoystickButton(this.padManipulator, 3);
    this.bumperLManipulator = new JoystickButton(this.padManipulator, 5);
    this.bumperRManipulator = new JoystickButton(this.padManipulator, 6);
    this.dpadDownManipulator = this.padManipulator.getPOV() == 180;
    this.dpadUpManipulator = this.padManipulator.getPOV() == 0;
    this.dpadLeftManipulator = this.padManipulator.getPOV() == 270;
    this.dpadRightManipulator = this.padManipulator.getPOV() == 90;
  }



  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public void configureButtonBindings() {
    this.tankDrive.schedule();

    this.buttonYManipulator  = new JoystickButton(this.padManipulator, 4);
    this.buttonYManipulator.whileHeld(this.autoSetShooterSpeed);
  }

  public Joystick getDriverPad(){
    return this.pad;
  }

  public Joystick getManipulatorPad(){
    return this.padManipulator;
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
    return new SequentialCommandGroup(new Auton2(drive).getRamseteCommand());
    //add shooting to sequential command
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

  public boolean getDpadUp() {
    return this.dpadUp;
  }

  public boolean getDpadDown() {
    return this.dpadDown;
  }

  public boolean getDpadRight() {
    return this.dpadRight;
  }

  public boolean getDpadLeft() {
    return this.dpadLeft;
  }


  public boolean getMButtonA() {
    return this.padManipulator.getRawButton(1);
  }

  public boolean getMButtonB() {
    return this.padManipulator.getRawButton(2);
  }

  public boolean getMButtonY() {
    return this.padManipulator.getRawButton(4);
  }

  public boolean getMButtonX() {
    return this.padManipulator.getRawButton(3);
  }

  public boolean getMBumperR() {
    return this.padManipulator.getRawButton(6);
  }

  public boolean getMBumperL() {
    return this.padManipulator.getRawButton(5);
  }

  public boolean getMDpadUp() {
    return this.dpadUpManipulator;
  }

  public boolean getMDpadDown() {
    return this.dpadDownManipulator;
  }

  public boolean getMDpadRight() {
    return this.dpadRightManipulator;
  }

  public boolean getMDpadLeft() {
    return this.dpadLeftManipulator;
  }

  // Getters for subsystems:
  public Climber getClimber() {
		return this.climber;
	}

  public Shooter getShooter() {
    return this.shooter;
  }

  public Limelight getLimelight() {
    return this.limelight;
  }

  public Drive getDrive(){
    return this.drive;
  }

  public Intake getIntake(){
    return this.intake;
  }

  public Turret getTurret(){
    return this.turret;
  }
}