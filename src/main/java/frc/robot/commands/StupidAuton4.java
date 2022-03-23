// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class StupidAuton4 extends CommandBase {
  /** Creates a new StupidAuton. */
  private final Drive drive;
  private final Shooter shoot;
  private final Intake intake;
  private final Limelight limelight;
  private int count;
  private double[] wheelSpeeds;
  public StupidAuton4(Drive drive, Shooter shoot, Intake intake, Limelight limelight) {
    this.drive = drive;
    this.shoot = shoot;
    this.intake = intake;
    this.limelight = limelight;
    addRequirements(drive, shoot, intake, limelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Drive forwards and intake 
    if(count <= 1000){
      shoot.setVelocity(0);
      intake.runIntestine(1);
      drive.setPower(0.6, 0.6);
    }
    // Go backwards a little
    else if(count <= 1250){
      intake.runIntestine(0);
      drive.setPower(-0.6, -0.6);
    }

    // Turn 360
    else if(count <= 1900){
      drive.setPower(-0.6, 0.6);
      intake.runIntestine(0);
    }

    // Go forward a bit
    else if(count <= 2250) {
      drive.setPower(0.6, 0.6);
    }
    // Shoot one ball
    else if(count <= 3000) {
      drive.setPower(0.0, 0.0);
      shoot.setVelocity(3200);
    }
    // Shoot second ball 
    else if(count <= 3750) {
      shoot.setVelocity(3200);
    }
    //Turn off shooter, turn 90 deg. left
    else if(count <= 4010) {
      shoot.setVelocity(0);
      drive.setPower(-0.6, 0.6);
    }
    // Go forward, intake 1 ball from human player
    else if(count <= 6710) {
      intake.runIntestine(1);
      drive.setPower(0.6, 0.6);
    }
    //Wait for human player
    else if(count <= 7710) {
      drive.setPower(0.0, 0.0);
    }
    // 335 degree right turn to ball
    else if(count <= 8315) {
      intake.runIntestine(0);
      drive.setPower(0.6, -0.6);
    }
    // Intake ball
    else if (count <= 10065) {
      intake.runIntestine(1);
      drive.setPower(0.6, 0.6);
    }
    // Turn left 15 deg.
    else if (count <= 10095) {
      intake.runIntestine(0);
      drive.setPower(-0.6, 0.6);
    }
    // Shoot one ball 
    else if ( count <= 10895) {
      drive.setPower(0.0, 0.0);
      shoot.setVelocity(3500);
    }

    // Shoot second ball
    else if ( count <= 11695) {
      shoot.setVelocity(3500);
    }
    // Stops robot
    else{
      drive.setPower(0, 0);
      shoot.setPower(0);
      intake.runIntestine(0);
    }
    count+=20;


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setPower(0, 0);
    shoot.setPower(0);
    intake.runIntestine(0);            
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (count > 13000){
      return true;
    }
    return false;
  }
}
