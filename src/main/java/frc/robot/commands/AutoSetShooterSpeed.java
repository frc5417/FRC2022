// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;

public class AutoSetShooterSpeed extends CommandBase {
  private final Shooter shooter;
  private final Intake intake;
  private double setPointVariable;
  private double[] pastSpeeds;
  private final int numberSpeeds = 50;
  private int counter = 0;

  /** Creates a new AutoSetShooterSpeed. */
  public AutoSetShooterSpeed(Shooter shooter, Intake intake) {
    this.shooter = shooter;
    this.intake = intake;
    this.setPointVariable = 0.0;
    pastSpeeds = new double[numberSpeeds];
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      
    setPointVariable = Constants.shootsetPointVar;
    System.out.println("Target Speed: " + setPointVariable);
    this.shooter.setVelocity(setPointVariable);
    //System.out.print("Power: ");
    //System.out.println(shooter.getShooter1().getEncoder().getVelocity());
    pastSpeeds[counter % numberSpeeds] = shooter.getShooter1().getEncoder().getVelocity();
    double avg = 0;
    for(int i = 0; i < numberSpeeds; i++) avg += pastSpeeds[i];
    avg /= numberSpeeds;
    System.out.println("Running average: " + avg);
    if((avg <= setPointVariable+100) && (avg >= setPointVariable-100)){
      this.intake.runIntestine(1);
    }
    else{
      this.intake.runIntestine(0);
    }
    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooter.setPower(0);
    this.intake.runIntestine(0);
    for(int i = 0; i < numberSpeeds; i++) pastSpeeds[i] = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
