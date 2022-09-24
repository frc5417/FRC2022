// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoAlignDrive;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class StupidAuton2 extends CommandBase {
  /** Creates a new StupidAuton. */
  private final Drive drive;
  private final Shooter shoot;
  private final Intake intake;
  private final Limelight limelight;
  private final AutoAlignDrive autoAlign;
  private final AutoSetShooterSpeed autoSpeed;
  private int count;
  public StupidAuton2(Drive drive, Shooter shoot, Intake intake, Limelight limelight) {
    this.drive = drive;
    this.shoot = shoot;
    this.intake = intake;
    this.limelight = limelight;
    autoAlign = new AutoAlignDrive(limelight, drive);
    autoSpeed = new AutoSetShooterSpeed(shoot, intake);
    addRequirements(drive, shoot, intake, limelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    limelight.ledOn();
    intake.deployPistons();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drive forwards to second ball????
    if(count <= 500){
      drive.rawMotorPower(0, 0);
    }
    else if(count <= 1750){
      drive.rawMotorPower(-.3, -.3);
      intake.runIntakeSystem(1);
    }
    else if(count <= 2000){
      drive.rawMotorPower(0, 0);
    }
    // align for second ball
    else if(count <= 3000){
      autoAlign.initialize();
    }
    // shoot second ball
    else if(count <= 9000){
      drive.setPower(0, 0);
      autoSpeed.execute();
    }
    count+=20;


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setPower(0, 0);
    shoot.setPower(0);
    intake.runIntestine(0); 
    intake.runIntakeSystem(0);           
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (count > 9500){
      return true;
    }
    return false;
  }
}
