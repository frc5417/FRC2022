// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
  private int count;
  private double[] wheelSpeeds;

  public StupidAuton2(Drive drive, Shooter shoot, Intake intake, Limelight limelight) {
    this.drive = drive;
    this.shoot = shoot;
    this.intake = intake;
    this.limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, shoot, intake, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    if(count <= 250) {
      shoot.setVelocity(2200);
    }
    else if(count <= 2000) {
      shoot.setVelocity(2200);
      intake.runIntestine(1);
    }
    else if(count <= 2750) {                                                                                                                                    
      drive.setPower(.3, .3);
      shoot.setPower(0);
      intake.runIntestine(0);
    }
    else {
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
    if (count > 3000){
      return true;
    }
    return false;
  }
}
