// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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
  private static Climber climber;
  private static Intake intake;
  private static Drive drive;
  private static Shooter shooter;
  private static Limelight limelight;
  private static Turret turret;

  // Define joysticks & buttons
  private static Joystick pad;
  private static JoystickButton buttonY;
  private static JoystickButton buttonX;
  private static JoystickButton buttonA;
  private static JoystickButton buttonB;
  private static JoystickButton bumperRight;
  private static JoystickButton bumperLeft;
  private static JoystickButton triggerRight;
  private static JoystickButton triggerLeft;
  private boolean dpadRight;
  private boolean dpadLeft;
  private boolean dpadUp;
  private boolean dpadDown;
  
  private static Joystick padManipulator;
  private static JoystickButton buttonXManipulator;
  private static JoystickButton buttonYManipulator;
  private static JoystickButton buttonAManipulator;
  private static JoystickButton buttonBManipulator;
  private static JoystickButton bumperRManipulator;
  private static JoystickButton bumperLManipulator;
  private boolean dpadRightManipulator;
  private boolean dpadLeftManipulator;
  private boolean dpadUpManipulator;
  private boolean dpadDownManipulator;

  private static Joystick buttonBoard;
  private static JoystickButton buttonB2;
  private static JoystickButton buttonB4; 
  private static JoystickButton buttonB6;
  private static JoystickButton buttonB8;
  private static JoystickButton buttonB10;
  private static JoystickButton buttonB12;
  private static JoystickButton buttonB14;
  private static JoystickButton buttonB16;

  // Define commands
  private static AutoAlignTurret autoAlignTurret;
  private static AutoSetShooterSpeed autoSetShooterSpeed;
  private static AutoClimbExtend autoClimbExtend;
  private static AutoClimbExtend autoClimbExtendSlightly;
  private static AutoClimbRetract autoClimbRetract;
  private static AutoClimbRetract autoClimbRetractSlightly;
  private static ClimbAnchor climbAnchor;
  private static ClimbPivot climbPivot;
  private static AutoTankDrive autoTankDrive;
  private static RunIntakeSystemForward runIntakeSystemForward;
  private static RunIntakeSystemBackward runIntakeSystemBackward;
  private static DeployIntakePistons deployIntakePistons;
  private static RetractIntakePistons retractIntakePistons;
  private static ManualAlignTurret manualAlignTurret;
  //private static Shoot shoot;
  private static StopClimb stopClimb;
  private static TankDrive tankDrive;
  private static ManualMoveClimb climbDrive;
  private static AutoAlignDrive autoAlignDrive;
  private static LowSpeedShoot lowSpeedShoot;
  // Command Groups:
  //private   SequentialCommandGroup climbCommands;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // init joysticks
    pad = new Joystick(0);
    padManipulator = new Joystick(1);
    //buttonBoard = new Joystick(2);

    // init subsystems
    climber = new Climber();     
    intake = new Intake();
    drive = new Drive();
    shooter = new Shooter();
    limelight = new Limelight();
    turret = new Turret();

    // init commands
    autoClimbExtend = new AutoClimbExtend( climber, Constants.climberExtendPos);
    autoClimbExtendSlightly = new AutoClimbExtend( climber, Constants.climberExtendSlightlyPos);
    autoClimbRetract = new AutoClimbRetract( climber, Constants.climberRetractPos);
    autoClimbRetractSlightly = new AutoClimbRetract( climber, Constants.climberRetractSlightlyPos);
    climbAnchor = new ClimbAnchor( climber, this);
    climbPivot = new ClimbPivot( climber, this);
    autoTankDrive = new AutoTankDrive(drive);
    runIntakeSystemForward = new RunIntakeSystemForward( intake);
    runIntakeSystemBackward = new RunIntakeSystemBackward( intake);
    deployIntakePistons = new DeployIntakePistons(intake, this);
    retractIntakePistons = new RetractIntakePistons( intake);
    autoAlignTurret = new AutoAlignTurret( limelight,  turret);
    autoSetShooterSpeed = new AutoSetShooterSpeed( limelight,  shooter,  intake);
    lowSpeedShoot = new LowSpeedShoot(shooter, intake);
    manualAlignTurret = new ManualAlignTurret(this,  turret);
    //shoot = new Shoot( shooter);
    stopClimb = new StopClimb( climber);
    tankDrive = new TankDrive(this,  drive);
    climbDrive = new ManualMoveClimb(climber, this);
    autoAlignDrive = new AutoAlignDrive(limelight, drive, this);
    /*
    climbCommands = new SequentialCommandGroup(
      autoClimbExtend,
      autoTankDrive,
      autoClimbRetract,
      climbAnchor,
      autoClimbExtendSlightly,
      climbPivot,
      autoClimbExtend,
      climbPivot,
      autoClimbRetractSlightly,
      climbAnchor,
      autoClimbRetract,
      climbAnchor,
      autoClimbExtendSlightly,
      climbPivot,
      autoClimbExtend,
      climbPivot,
      autoClimbRetractSlightly,
      climbAnchor,
      autoClimbRetract,
      climbAnchor
    );*/

    initializeButtons();
    configureButtonBindings();
  }
  
  public Limelight getLimelight(){
    return limelight;
  }

  public void initializeButtons() {
    buttonY = new JoystickButton( pad, 4);
    buttonB = new JoystickButton( pad, 2);
    buttonA = new JoystickButton( pad, 1);
    buttonX = new JoystickButton( pad, 3);
    bumperLeft = new JoystickButton( pad, 5);
    bumperRight = new JoystickButton( pad, 6);

    buttonYManipulator = new JoystickButton( padManipulator, 4);
    buttonBManipulator = new JoystickButton( padManipulator, 2);
    buttonAManipulator = new JoystickButton( padManipulator, 1);
    buttonXManipulator = new JoystickButton( padManipulator, 3);
    bumperLManipulator = new JoystickButton( padManipulator, 5);
    bumperRManipulator = new JoystickButton( padManipulator, 6);

    /*buttonB2 = new JoystickButton( buttonBoard, 2);
    buttonB4 = new JoystickButton( buttonBoard, 4);
    buttonB6 = new JoystickButton( buttonBoard, 6);
    buttonB8 = new JoystickButton( buttonBoard, 8);
    buttonB10 = new JoystickButton( buttonBoard, 10);
    buttonB12 = new JoystickButton( buttonBoard, 12);
    buttonB14 = new JoystickButton( buttonBoard, 14);
    buttonB16 = new JoystickButton( buttonBoard, 16);*/
     
    /*buttonB10.whenPressed(autoClimbExtend);
    buttonB12.whenPressed(autoClimbRetract);*/
    // buttonB2.whenPressed(climbCommands);
    // buttonB4.whenPressed(stopClimb);

    bumperLManipulator.whenHeld(runIntakeSystemForward);
    bumperRManipulator.whenHeld(runIntakeSystemBackward);
    buttonBManipulator.whenPressed(deployIntakePistons);

    buttonA.whileHeld(autoAlignDrive);
    buttonA.whenPressed(new InstantCommand(() -> limelight.ledOn())).whenReleased(new InstantCommand(() -> limelight.ledOff()));

    buttonYManipulator.whenHeld(autoSetShooterSpeed);
    //buttonAManipulator.whenHeld(lowSpeedShoot);
  }



  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public void configureButtonBindings() {
    dpadDown = pad.getPOV() == 180;
    dpadUp = pad.getPOV() == 0;
    dpadLeft = pad.getPOV() == 270;
    dpadRight = pad.getPOV() == 90;

    dpadDownManipulator = padManipulator.getPOV() == 180;
    dpadUpManipulator = padManipulator.getPOV() == 0;
    dpadLeftManipulator = padManipulator.getPOV() == 270;
    dpadRightManipulator = padManipulator.getPOV() == 90;
    
    climber.setActiveClamp(buttonAManipulator.get());
    climber.setPassiveClamp(buttonXManipulator.get());
    climber.setPower(padManipulator.getRawAxis(1), padManipulator.getRawAxis(5));

    tankDrive.schedule();
     
    //System.out.println("Left Motor 2: " + climber.getLeftMotor2Encoder().getPosition());
    //System.out.println("Right Motor 2: " + climber.getRightMotor2Encoder().getPosition());
  
  }

  public Joystick getDriverPad(){
    return  pad;
  }

  public Joystick getManipulatorPad(){
    return  padManipulator;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    limelight.ledOff();
    //this runs two auton commands- Auton2 and Auton3 with their ramsete commands
    //return new SequentialCommandGroup(new Auton2(driveSubsystem).getRamseteCommand(), new Auton3(driveSubsystem).getRamseteCommand());
    
    //this just runs Auton2
    return new StupidAuton(drive, shooter, intake, limelight);
  }

  public double getDriverLeftJoystick(){
    return  pad.getRawAxis(1);
  }

  public double getDriverRightJoystick(){
    return  pad.getRawAxis(5);
  }

  public boolean getButtonA() {
    return  pad.getRawButton(1);
  }

  public boolean getButtonB() {
    return  pad.getRawButton(2);
  }

  public boolean getButtonY() {
    return  pad.getRawButton(4);
  }

  public boolean getButtonX() {
    return  pad.getRawButton(3);
  }

  public boolean getBumperR() {
    return  pad.getRawButton(6);
  }

  public boolean getBumperL() {
    return  pad.getRawButton(5);
  }

  public boolean getDpadUp() {
    return  dpadUp;
  }

  public boolean getDpadDown() {
    return  dpadDown;
  }

  public boolean getDpadRight() {
    return  dpadRight;
  }

  public boolean getDpadLeft() {
    return  dpadLeft;
  }


  public boolean getMButtonA() {
    return  padManipulator.getRawButton(1);
  }

  public boolean getMButtonB() {
    return  padManipulator.getRawButton(2);
  }

  public boolean getMButtonY() {
    return  padManipulator.getRawButton(4);
  }

  public boolean getMButtonX() {
    return  padManipulator.getRawButton(3);
  }

  public boolean getMBumperR() {
    return  padManipulator.getRawButton(6);
  }

  public boolean getMBumperL() {
    return  padManipulator.getRawButton(5);
  }

  public boolean getMDpadUp() {
    return  dpadUpManipulator;
  }

  public boolean getMDpadDown() {
    return  dpadDownManipulator;
  }

  public boolean getMDpadRight() {
    return  dpadRightManipulator;
  }

  public boolean getMDpadLeft() {
    return  dpadLeftManipulator;
  }

  /*public boolean getButtonB2(){
    return buttonB2.get();
  }

  public boolean getButtonB4(){
    return buttonB4.get();
  }

  public boolean getButtonB6(){
    return buttonB6.get();
  }

  public boolean getButtonB8(){
    return buttonB8.get();
  }

  public boolean getButtonB10(){
    return buttonB10.get();
  }

  public boolean getButtonB12(){
    return buttonB12.get();
  }

  public boolean getButtonB14(){
    return buttonB14.get();
  }

  public boolean getButtonB16(){
    return buttonB16.get();
  }*/

  public Climber getClimberSubsystem() {
    return climber;
  }

}