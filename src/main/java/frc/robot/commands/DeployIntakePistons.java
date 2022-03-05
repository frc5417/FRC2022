// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;


public class DeployIntakePistons extends CommandBase {
  /** Creates a new DeployIntakePistons. */
  private final Intake intake;
  private final RobotContainer robotContainer;
  private boolean currentState;
  private boolean commandState;

  public DeployIntakePistons(Intake intake, RobotContainer container) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.robotContainer = container;
    currentState = true;
    commandState = false;
    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (currentState == true){
      this.intake.deployPistons();
      currentState = false;
      commandState = true;
    }
    else if (currentState == false){
      this.intake.retractPistons();
      currentState = true;
      commandState = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!robotContainer.getMButtonB() && commandState){
      commandState = false;
      return true;
    }
    return false;
  }
}
