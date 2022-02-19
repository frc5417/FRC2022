// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class AutoSetShooterSpeed extends CommandBase {
  private final Limelight limelight;
  private final Shooter shooter;
  private final Intake intake;
  private double setPointVariable;

  /** Creates a new AutoSetShooterSpeed. */
  public AutoSetShooterSpeed(Limelight limelight, Shooter shooter, Intake intake) {
    this.limelight = limelight;
    this.shooter = shooter;
    this.intake = intake;
    this.setPointVariable = 0.0;

    addRequirements(limelight, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.setPointVariable = limelight.estimateDistance();

    this.shooter.setVelocity(4000);

    if((shooter.getShooter1().getEncoder().getVelocity() <= 4000+500) && (shooter.getShooter1().getEncoder().getVelocity() >= 4000-500)){
      this.intake.runIntake();
      this.intake.runIntestine();
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
