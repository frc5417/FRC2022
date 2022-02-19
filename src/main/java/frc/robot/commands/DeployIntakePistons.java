// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;


public class DeployIntakePistons extends CommandBase {
  /** Creates a new DeployIntakePistons. */
  private final Intake intake;

  public DeployIntakePistons(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.intake.deployPistons();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.intake.retractPistons();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
