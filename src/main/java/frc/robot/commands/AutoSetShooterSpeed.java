// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
      
    setPointVariable = 2512.5;

    this.shooter.setVelocity(setPointVariable);
    //System.out.print("Power: ");
    //System.out.println(shooter.getShooter1().getEncoder().getVelocity());
    
    if((shooter.getShooter1().getEncoder().getVelocity() <= setPointVariable+250) && (shooter.getShooter1().getEncoder().getVelocity() >= setPointVariable-250)){
      this.intake.runIntestine(1);
    }
    else{
      this.intake.runIntestine(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooter.setPower(0);
    this.intake.runIntestine(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}