// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class AutoSetShooterSpeed extends CommandBase {
  private final Limelight limelight;
  private final Shooter shooter;

  /** Creates a new AutoSetShooterSpeed. */
  public AutoSetShooterSpeed(Limelight limelight, Shooter shooter) {
    this.limelight = limelight;
    this.shooter = shooter;

    addRequirements(limelight, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setVelocity(limelight.estimateDistance());
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
