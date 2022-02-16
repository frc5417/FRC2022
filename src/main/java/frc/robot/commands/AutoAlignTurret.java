// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class AutoAlignTurret extends CommandBase {

  private final Limelight limelight;
  private final Turret turret;
  private boolean isAligned = false;

  /** Creates a new AutoAlignTurret. */
  public AutoAlignTurret(Limelight limelight, Turret turret) {
    this.limelight = limelight;
    this.turret = turret;

    addRequirements(limelight, turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.isAligned = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turretAdjust = 0;
    if(!this.limelight.getV()){
      this.isAligned = false;
      return;
    }
    double x = this.limelight.getX();
    if(x > Constants.limeLightErrorAllowed){
      turretAdjust = Constants.kPturn*(-x) + Constants.minCommand;
      this.isAligned = false;
    }else{
      this.isAligned = true;
    }
    if(!this.isAligned) {
      this.turret.setPower(turretAdjust);
    }
    System.out.println("aligning...");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.turret.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isAligned;
  }
}
