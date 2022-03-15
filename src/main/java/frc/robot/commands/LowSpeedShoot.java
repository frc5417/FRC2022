// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class LowSpeedShoot extends CommandBase {
  /** Creates a new LowSpeedShoot. */

  private final Shooter shooter;
  private final Intake intake;
  private double setPointVariable;
  public LowSpeedShoot(Shooter shooter, Intake intake) {

    this.shooter = shooter;
    this.intake = intake;
    this.setPointVariable = 1500;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.shooter.setVelocity(setPointVariable);

    if((shooter.getShooter1().getEncoder().getVelocity() <= setPointVariable+250) && (shooter.getShooter1().getEncoder().getVelocity() >= setPointVariable-250)){
      this.intake.runIntestine(1);
    }
    else{
      this.intake.runIntestine(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
